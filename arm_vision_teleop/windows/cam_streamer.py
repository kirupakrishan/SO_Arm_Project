"""Run this on WINDOWS: stream the laptop webcam into WSL for arm_vision_teleop.

WSL2's kernel has no camera drivers, so the webcam is read here, on Windows, and
JPEG frames are sent over TCP to the `camera_bridge` ROS node in WSL. WSL2 forwards
Windows' localhost to WSL, so the default --host localhost just works.

Setup (once, in PowerShell):   py -m pip install opencv-python
Run:                           py cam_streamer.py            (Ctrl+C to stop)
Options:                       --device 1  --width 1280 --height 720  --preview

Wire format (must match arm_vision_teleop/stream_protocol.py):
    b'FRM1' + 4-byte big-endian length + JPEG bytes
"""

import argparse
import socket
import struct
import sys
import time

try:
    import cv2
except ImportError:
    sys.exit('OpenCV missing. In PowerShell run:  py -m pip install opencv-python')

HEADER = struct.Struct('>4sI')
MAGIC = b'FRM1'


def open_camera(args):
    # DirectShow opens laptop webcams quickly and honours the resolution request.
    cap = cv2.VideoCapture(args.device, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if not cap.isOpened():
        sys.exit(f'Cannot open camera {args.device}. Is another app (Teams, Zoom, '
                 f'Camera) using it? Try --device 1.')
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'Camera {args.device} open at {w}x{h}')
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='localhost', help='WSL address (default localhost)')
    ap.add_argument('--port', type=int, default=5005)
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--quality', type=int, default=80, help='JPEG quality 1-100')
    ap.add_argument('--preview', action='store_true', help='show a local preview window')
    args = ap.parse_args()

    cap = open_camera(args)
    params = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
    period = 1.0 / args.fps
    try:
        while True:
            try:
                sock = socket.create_connection((args.host, args.port), timeout=3)
            except OSError:
                print(f'Waiting for camera_bridge on {args.host}:{args.port} '
                      f'(is the ROS launch running in WSL?) ...')
                time.sleep(2)
                continue
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f'Connected to {args.host}:{args.port}, streaming. Ctrl+C to stop.')
            sent, t0 = 0, time.time()
            try:
                while True:
                    start = time.time()
                    ok, frame = cap.read()
                    if not ok:
                        print('Camera read failed, retrying...')
                        time.sleep(0.5)
                        continue
                    ok, jpeg = cv2.imencode('.jpg', frame, params)
                    if not ok:
                        continue
                    data = jpeg.tobytes()
                    sock.sendall(HEADER.pack(MAGIC, len(data)) + data)
                    sent += 1
                    if time.time() - t0 >= 5:
                        print(f'{sent / (time.time() - t0):.1f} fps')
                        sent, t0 = 0, time.time()
                    if args.preview:
                        cv2.imshow('cam_streamer (q to quit)', frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            return
                    time.sleep(max(0.0, period - (time.time() - start)))
            except OSError as e:
                print(f'Connection lost ({e}); reconnecting...')
            finally:
                sock.close()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

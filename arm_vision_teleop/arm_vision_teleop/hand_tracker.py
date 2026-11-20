"""ROS 2 node: webcam image -> hand features with MediaPipe Hand Landmarker.

Subscribes  /camera/image_raw            (sensor_msgs/Image, bgr8 or rgb8)
Publishes   /hand/position               (geometry_msgs/PointStamped)
                x = palm u (0 left .. 1 right), y = palm v (0 top .. 1 bottom),
                z = hand size (wrist -> middle-finger base, fraction of image height)
            /hand/openness               (std_msgs/Float32, 0 pinched .. 1 spread)
            /hand/engaged                (std_msgs/Bool: hand visible and not a fist)
            /hand/debug_image            (sensor_msgs/Image, landmarks + status overlay)
mode:=arm also runs MediaPipe Pose (elbow as the base) and publishes
            /arm/pose                    (sensor_msgs/JointState used as named values:
                forearm_x/y/z = elbow->wrist unit vector, camera axes (x right, y down,
                z away from camera); elbow_angle, wrist_flex, wrist_roll in radians)

Runs with the ~/RobotArm/.venv_vision interpreter (mediapipe + OpenCV).
"""

import array
import os
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float32

from arm_vision_teleop import arm_geometry as ag
from arm_vision_teleop.hand_geometry import Ema, compute_features

MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/hand_landmarker/'
             'hand_landmarker/float16/latest/hand_landmarker.task')
DEFAULT_MODEL = os.path.expanduser('~/.cache/arm_vision_teleop/hand_landmarker.task')
POSE_URL = ('https://storage.googleapis.com/mediapipe-models/pose_landmarker/'
            'pose_landmarker_full/float16/latest/pose_landmarker_full.task')
DEFAULT_POSE_MODEL = os.path.expanduser(
    '~/.cache/arm_vision_teleop/pose_landmarker_full.task')
ARM_FIELDS = ['forearm_x', 'forearm_y', 'forearm_z', 'elbow_angle', 'wrist_flex',
              'wrist_roll']


def image_to_bgr(msg):
    if msg.encoding not in ('bgr8', 'rgb8'):
        raise ValueError(f'unsupported encoding {msg.encoding} (need bgr8/rgb8)')
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = arr[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return img if msg.encoding == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def bgr_to_image(bgr, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    # array.array('B') takes rclpy's fast path; bytes would be validated element by element.
    msg.data = array.array('B', np.ascontiguousarray(bgr).tobytes())
    return msg


class HandTracker(Node):

    def __init__(self):
        super().__init__('hand_tracker')
        p = self.declare_parameter
        model = os.path.expanduser(p('model_path', DEFAULT_MODEL).value)
        self.mirror = bool(p('mirror', True).value)
        self.pinch_closed = float(p('pinch_closed', 0.25).value)
        self.pinch_open = float(p('pinch_open', 0.9).value)
        self.curl_ratio = float(p('curl_ratio', 1.3).value)
        self.debounce = int(p('engage_debounce_frames', 3).value)
        self.ema = Ema(float(p('smoothing_alpha', 0.5).value))
        self.publish_debug = bool(p('publish_debug_image', True).value)
        # Preview window drawn straight from this process (OpenCV/Qt): no 900 KB
        # images through DDS and no extra viewer app, so it keeps up at full rate.
        self.show_window = bool(p('show_window', True).value)
        self.window = 'Hand teleop (camera)'
        self.window_open = False
        self.fps = 0.0
        self.last_frame_t = None
        self.proc_ms = 0.0
        self.latency_ms = 0.0
        min_conf = float(p('min_confidence', 0.6).value)
        self.mode = p('mode', 'hand').value
        if self.mode not in ('hand', 'arm'):
            raise ValueError("mode must be 'hand' or 'arm'")
        self.min_visibility = float(p('min_visibility', 0.5).value)
        self.arm_alpha = float(p('arm_smoothing_alpha', 0.4).value)
        pose_model = os.path.expanduser(p('pose_model_path', DEFAULT_POSE_MODEL).value)

        self.landmarker = self._load_model(model, min_conf)
        self.pose = self._load_pose(pose_model, min_conf) if self.mode == 'arm' else None
        self.arm = None          # smoothed arm features (dict) or None
        self.arm_2d = None       # (shoulder, elbow, wrist) pixel coords for drawing
        self.arm_status = ''
        self.connections = [(c.start, c.end) for c in
                            self._vision.HandLandmarksConnections.HAND_CONNECTIONS]
        self.last_ts_ms = 0
        self.engaged = False
        self.vote = 0            # consecutive frames disagreeing with `engaged`

        self.pos_pub = self.create_publisher(PointStamped, '/hand/position', 10)
        self.open_pub = self.create_publisher(Float32, '/hand/openness', 10)
        self.eng_pub = self.create_publisher(Bool, '/hand/engaged', 10)
        self.arm_pub = self.create_publisher(JointState, '/arm/pose', 10)
        self.dbg_pub = self.create_publisher(Image, '/hand/debug_image', 2)
        self.create_subscription(Image, p('image_topic', '/camera/image_raw').value,
                                 self._on_image,
                                 # Reliable: big best-effort images get dropped whole when one
                                 # UDP fragment is lost. Depth 1: always process the newest frame.
                                 QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.n_frames, self.busy_s = 0, 0.0
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f'Tracker ready ({self.mode} mode). Open hand = move, pinch = gripper, '
            'fist = pause.' + (' Keep your elbow, wrist and hand in view.'
                               if self.mode == 'arm' else ''))

    def _load_model(self, path, min_conf):
        from mediapipe.tasks.python import BaseOptions, vision
        self._vision = vision
        if not os.path.exists(path):
            self.get_logger().info(f'Downloading hand model to {path} ...')
            os.makedirs(os.path.dirname(path), exist_ok=True)
            urllib.request.urlretrieve(MODEL_URL, path + '.part')
            os.replace(path + '.part', path)
        opts = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=path),
            running_mode=vision.RunningMode.VIDEO, num_hands=1,
            min_hand_detection_confidence=min_conf,
            min_hand_presence_confidence=min_conf,
            min_tracking_confidence=min_conf)
        return vision.HandLandmarker.create_from_options(opts)

    def _load_pose(self, path, min_conf):
        vision = self._vision
        from mediapipe.tasks.python import BaseOptions
        if not os.path.exists(path):
            self.get_logger().info(f'Downloading pose model to {path} ...')
            os.makedirs(os.path.dirname(path), exist_ok=True)
            urllib.request.urlretrieve(POSE_URL, path + '.part')
            os.replace(path + '.part', path)
        opts = vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=path),
            running_mode=vision.RunningMode.VIDEO, num_poses=1,
            min_pose_detection_confidence=min_conf,
            min_pose_presence_confidence=min_conf,
            min_tracking_confidence=min_conf)
        return vision.PoseLandmarker.create_from_options(opts)

    # ------------------------------------------------------------------
    def _report(self):
        if self.n_frames:
            self.get_logger().info(f'Tracking {self.n_frames / 30.0:.1f} fps, '
                                   f'{1000 * self.busy_s / self.n_frames:.0f} ms/frame')
        else:
            self.get_logger().warn('No camera frames in the last 30 s.')
        self.n_frames, self.busy_s = 0, 0.0

    def _on_image(self, msg):
        import mediapipe as mp
        t_start = time.monotonic()
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        if self.mirror:
            bgr = cv2.flip(bgr, 1)
        h, w = bgr.shape[:2]

        ts = max(int(time.monotonic() * 1000), self.last_ts_ms + 1)  # must increase
        self.last_ts_ms = ts
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        result = self.landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

        feats = None
        if result.hand_landmarks:
            lm = [(p.x, p.y) for p in result.hand_landmarks[0]]
            try:
                feats = compute_features(lm, w, h, self.pinch_closed, self.pinch_open,
                                         self.curl_ratio)
            except ValueError:
                feats = None

        arm_ok = True
        if self.mode == 'arm':
            pose = self.pose.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)
            arm_ok = self._update_arm(pose, result, w, h, msg.header)

        want = feats is not None and not feats.fist and arm_ok
        if not want and feats is None:
            self.vote = self.debounce        # hand lost: disengage immediately
        elif want != self.engaged:
            self.vote += 1
        else:
            self.vote = 0
        if self.vote >= self.debounce:
            self.engaged = want
            self.vote = 0

        if feats is None:
            self.ema.reset()
        else:
            u, v, size, op = self.ema((feats.u, feats.v, feats.size, feats.openness))
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x, pt.point.y, pt.point.z = float(u), float(v), float(size)
            self.pos_pub.publish(pt)
            self.open_pub.publish(Float32(data=float(op)))
        self.eng_pub.publish(Bool(data=self.engaged))

        busy = time.monotonic() - t_start
        self.n_frames += 1
        self.busy_s += busy
        self._update_stats(busy, msg)
        want_topic = self.publish_debug and self.dbg_pub.get_subscription_count() > 0
        if self.show_window or want_topic:
            self._draw(bgr, result, feats)
            if want_topic:
                self.dbg_pub.publish(bgr_to_image(bgr, msg.header))
            if self.show_window:
                self._show(bgr)

    def _update_arm(self, pose, hand, w, h, header):
        """Arm features from pose + hand landmarks; publishes /arm/pose. True if usable."""
        self.arm_2d = None
        if not pose.pose_landmarks:
            return self._arm_lost('NO ARM (pose not found)')
        if not hand.hand_landmarks:
            return self._arm_lost('')
        img_lm = pose.pose_landmarks[0]
        world = [np.array([p.x, p.y, p.z]) for p in pose.pose_world_landmarks[0]]
        hw = hand.hand_landmarks[0][0]
        side = ag.pick_side([(p.x, p.y) for p in img_lm], (hw.x, hw.y))
        s_i, e_i, w_i = ag.POSE_SIDES[side]
        self.arm_2d = [(int(img_lm[i].x * w), int(img_lm[i].y * h)) for i in (s_i, e_i, w_i)]
        vis = lambda i: (img_lm[i].visibility or 0.0)  # noqa: E731
        if vis(e_i) < self.min_visibility or vis(w_i) < self.min_visibility:
            return self._arm_lost('ELBOW NOT IN VIEW')
        try:
            forearm = ag.unit(world[w_i] - world[e_i])
            # Elbow angle needs the shoulder; NaN tells the follower to use fixed reach.
            elbow = (ag.joint_angle(world[s_i], world[e_i], world[w_i])
                     if vis(s_i) >= self.min_visibility else float('nan'))
        except ValueError:
            return self._arm_lost('ARM DEGENERATE')
        try:
            hwl = [np.array([p.x, p.y, p.z]) for p in hand.hand_world_landmarks[0]]
            label = hand.handedness[0][0].category_name
            normal = ag.palm_normal(hwl, label, mirrored=self.mirror)
            flex = ag.wrist_flex(forearm, hwl[ag.H_MIDDLE_MCP] - hwl[ag.H_WRIST], normal)
            roll = ag.wrist_roll(forearm, normal)
        except ValueError:
            # Hand bent ~90 deg: palm normal along the forearm, wrist angles undefined.
            # Keep tracking the forearm and hold the last wrist values.
            prev = self.arm or {}
            flex, roll = prev.get('wrist_flex', 0.0), prev.get('wrist_roll', 0.0)
        sample = {'forearm_x': forearm[0], 'forearm_y': forearm[1], 'forearm_z': forearm[2],
                  'elbow_angle': elbow, 'wrist_flex': flex, 'wrist_roll': roll}
        if self.arm is None:
            self.arm = sample
        else:
            a = self.arm_alpha
            sample['wrist_roll'] = ag.unwrap_near(roll, self.arm['wrist_roll'])
            for k, v in sample.items():
                prev = self.arm[k]
                self.arm[k] = v if (np.isnan(prev) or np.isnan(v)) else a * v + (1 - a) * prev
        f = ag.unit([self.arm['forearm_x'], self.arm['forearm_y'], self.arm['forearm_z']])
        self.arm['forearm_x'], self.arm['forearm_y'], self.arm['forearm_z'] = f
        js = JointState()
        js.header = header
        js.name = list(ARM_FIELDS)
        js.position = [float(self.arm[k]) for k in ARM_FIELDS]
        self.arm_pub.publish(js)
        self.arm_status = ''
        return True

    def _arm_lost(self, status):
        self.arm = None
        self.arm_status = status
        return False

    def _update_stats(self, busy, msg):
        now = time.monotonic()
        if self.last_frame_t is not None:
            inst = 1.0 / max(now - self.last_frame_t, 1e-3)
            self.fps = inst if self.fps == 0.0 else 0.9 * self.fps + 0.1 * inst
        self.last_frame_t = now
        self.proc_ms = 0.9 * self.proc_ms + 0.1 * busy * 1000
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0:  # time from camera_bridge receiving the frame to now
            lat = (self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000
            self.latency_ms = 0.9 * self.latency_ms + 0.1 * lat

    def _show(self, img):
        try:
            if self.window_open and cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                self.get_logger().info('Preview window closed (tracking continues).')
                self.show_window = False
                return
            cv2.imshow(self.window, img)
            self.window_open = True
            cv2.waitKey(1)
        except cv2.error as e:
            self.get_logger().warn(f'Cannot open a preview window ({e}); '
                                   'tracking continues without it.')
            self.show_window = False

    def _draw(self, img, result, feats):
        h, w = img.shape[:2]
        if result.hand_landmarks:
            pts = [(int(p.x * w), int(p.y * h)) for p in result.hand_landmarks[0]]
            for a, b in self.connections:
                cv2.line(img, pts[a], pts[b], (255, 255, 255), 2)
            for q in pts:
                cv2.circle(img, q, 4, (0, 140, 255), -1)
            cv2.line(img, pts[4], pts[8], (0, 255, 255), 2)
        if self.arm_2d is not None:
            s, e, wr = self.arm_2d
            cv2.line(img, s, e, (255, 120, 0), 4)
            cv2.line(img, e, wr, (255, 200, 0), 5)
            cv2.circle(img, e, 9, (0, 0, 255), -1)   # elbow = base
        if self.mode == 'arm' and self.arm is not None:
            el = self.arm['elbow_angle']
            el_txt = '--' if np.isnan(el) else f'{np.degrees(el):3.0f}'
            cv2.putText(img, f"elbow {el_txt}  flex {np.degrees(self.arm['wrist_flex']):+4.0f}  "
                             f"roll {np.degrees(self.arm['wrist_roll']):+4.0f} deg",
                        (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        if self.mode == 'arm' and self.arm_status:
            text, color = self.arm_status, (0, 0, 255)
        elif feats is None:
            text, color = 'NO HAND', (0, 0, 255)
        elif self.engaged:
            text, color = 'ENGAGED', (0, 200, 0)
        else:
            text, color = 'PAUSED (fist)', (0, 165, 255)
        cv2.rectangle(img, (0, 0), (w, 36), (0, 0, 0), -1)
        cv2.putText(img, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if feats is not None and self.ema.value is not None:
            op = self.ema.value[3]
            cv2.putText(img, f'gripper {op * 100:3.0f}%', (w - 230, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.rectangle(img, (w - 60, 8), (w - 60 + int(50 * op), 28), (0, 255, 255), -1)
            cv2.rectangle(img, (w - 60, 8), (w - 10, 28), (255, 255, 255), 1)
        # Diagnostics: if fps is low here, the camera / streamer is the bottleneck.
        cv2.putText(img, f'{self.fps:4.1f} fps  track {self.proc_ms:3.0f} ms  '
                         f'latency {self.latency_ms:3.0f} ms', (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    def close(self):
        self.landmarker.close()
        if self.pose is not None:
            self.pose.close()
        if self.window_open:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
        # else: Ctrl+C invalidated the context mid-spin; a normal shutdown.
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

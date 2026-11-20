"""Wire format between windows/cam_streamer.py and camera_bridge.

Each frame: 4-byte magic b'FRM1' + 4-byte big-endian length + JPEG bytes.
Kept dependency-free (stdlib only) so the Windows script can copy it verbatim.
"""

import struct

MAGIC = b'FRM1'
HEADER = struct.Struct('>4sI')
MAX_FRAME_BYTES = 8 * 1024 * 1024


class ProtocolError(Exception):
    pass


def pack_frame(jpeg_bytes):
    return HEADER.pack(MAGIC, len(jpeg_bytes)) + jpeg_bytes


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('stream closed')
        buf += chunk
    return bytes(buf)


def recv_frame(sock):
    """Read one frame from a connected socket; returns the JPEG bytes."""
    magic, length = HEADER.unpack(_recv_exact(sock, HEADER.size))
    if magic != MAGIC:
        raise ProtocolError(f'bad magic {magic!r} (not a cam_streamer client?)')
    if length == 0 or length > MAX_FRAME_BYTES:
        raise ProtocolError(f'bad frame length {length}')
    return _recv_exact(sock, length)

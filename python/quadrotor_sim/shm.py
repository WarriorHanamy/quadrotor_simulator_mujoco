"""Shared memory backend for the quadrotor simulator.

Uses ctypes struct layouts matching include/sim_schema.h exactly.
Depends on mmap (Linux /dev/shm), no additional libraries.
"""

import ctypes
import mmap
import os
import struct
import time
from typing import Optional


# ---------------------------------------------------------------------------
# Path constants (aligned with sim_schema.h)
# ---------------------------------------------------------------------------

SHM_BASE_DIR = "/dev/shm/quadrotor_sim"
SHM_STATE_FILE = SHM_BASE_DIR + "/state"
SHM_CTRL_FILE = SHM_BASE_DIR + "/ctrl"
SHM_IMAGE_FILE = SHM_BASE_DIR + "/image"

# ---------------------------------------------------------------------------
# FourCC for rgb8
# ---------------------------------------------------------------------------

ENCODING_RGB8 = 0x72363862  # 'rgb8' in little-endian

# ---------------------------------------------------------------------------
# Ctypes structs — binary-identical to sim_schema.h
# ---------------------------------------------------------------------------


class QuadrotorControlC(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint64),
        ("thrust", ctypes.c_double),
        ("torque", ctypes.c_double * 3),
        ("timestamp_ns", ctypes.c_uint64),
        ("_pad", ctypes.c_uint8 * 16),
    ]
    _pack_ = 1


class QuadrotorStateC(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint64),
        ("time", ctypes.c_double),
        ("position", ctypes.c_double * 3),
        ("orientation", ctypes.c_double * 4),
        ("linear_velocity", ctypes.c_double * 3),
        ("angular_velocity", ctypes.c_double * 3),
        ("linear_acceleration", ctypes.c_double * 3),
        ("timestamp_ns", ctypes.c_uint64),
        ("_pad", ctypes.c_uint8 * 40),
    ]
    _pack_ = 1


class ImageHeaderC(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint64),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("encoding", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("_pad", ctypes.c_uint8 * 36),
    ]
    _pack_ = 1


# Verify sizes
assert ctypes.sizeof(QuadrotorControlC) == 64, "QuadrotorControlC size mismatch"
assert ctypes.sizeof(QuadrotorStateC) == 192, "QuadrotorStateC size mismatch"
assert ctypes.sizeof(ImageHeaderC) == 64, "ImageHeaderC size mismatch"


# ---------------------------------------------------------------------------
# Seqlock helpers
# ---------------------------------------------------------------------------


def shm_read(src_fields, dst_struct):
    """Read ctypes struct under seqlock protocol.

    Returns True if a consistent snapshot was obtained.
    """
    before = src_fields["sequence"]
    if before & 1:
        return False
    # Memory barrier: cast to bytes via buffer protocol forces read completion
    ctypes.memmove(
        ctypes.addressof(dst_struct),
        ctypes.addressof(src_fields),
        ctypes.sizeof(dst_struct),
    )
    after = src_fields["sequence"]
    return before == after


# ---------------------------------------------------------------------------
# ShmReader / ShmWriter
# ---------------------------------------------------------------------------


class ShmReader:
    """Read-only shared memory reader with seqlock protection."""

    def __init__(self, filename: str, struct_cls, size: int):
        self._filename = filename
        self._size = size
        self._struct_cls = struct_cls
        self._fd: int = -1
        self._mm: Optional[mmap.mmap] = None
        self._obj: Optional[ctypes.Structure] = None

    def attach(self):
        """Open and mmap an existing shared memory segment (read-only)."""
        if self._fd >= 0:
            return
        self._fd = os.open(self._filename, os.O_RDONLY)
        self._mm = mmap.mmap(self._fd, self._size, access=mmap.ACCESS_READ)
        self._obj = self._struct_cls.from_buffer_copy(self._mm)  # type: ignore  # used for type info only

    def detach(self):
        if self._mm is not None:
            self._mm.close()
            self._mm = None
            self._obj = None
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def read(self, dst) -> bool:
        """Attempt to read a consistent snapshot via seqlock. Returns True on success."""
        if self._mm is None:
            return False
        # Read sequence from mmap
        before = struct.unpack_from("Q", self._mm, 0)[0]
        if before & 1:
            return False
        # Copy full struct from mmap into dst
        buf = bytes(self._mm[: ctypes.sizeof(dst)])
        ctypes.memmove(ctypes.addressof(dst), buf, len(buf))
        # Re-read sequence
        after = struct.unpack_from("Q", self._mm, 0)[0]
        return before == after


class ShmWriter:
    """Write-shared memory writer with seqlock protection."""

    def __init__(self, filename: str, struct_cls, size: int):
        self._filename = filename
        self._size = size
        self._struct_cls = struct_cls
        self._fd: int = -1
        self._mm: Optional[mmap.mmap] = None
        self._obj: Optional[ctypes.Structure] = None

    def create(self):
        """Create or open a shared memory segment for writing."""
        flags = os.O_CREAT | os.O_RDWR
        self._fd = os.open(self._filename, flags, 0o644)
        os.ftruncate(self._fd, self._size)
        self._mm = mmap.mmap(self._fd, self._size, access=mmap.ACCESS_WRITE)
        self._obj = self._struct_cls.from_buffer(self._mm)  # type: ignore
        # Initialize
        ctypes.memset(ctypes.addressof(self._obj), 0, self._size)

    def open(self):
        """Open existing shared memory segment for read/write."""
        self._fd = os.open(self._filename, os.O_RDWR)
        self._mm = mmap.mmap(self._fd, self._size, access=mmap.ACCESS_WRITE)
        self._obj = self._struct_cls.from_buffer(self._mm)  # type: ignore

    def destroy(self):
        """Close and optionally remove the segment."""
        self.detach()
        try:
            os.unlink(self._filename)
        except OSError:
            pass

    def detach(self):
        self._obj = None  # release buffer reference before close
        if self._mm is not None:
            self._mm.flush()
            self._mm.close()
            self._mm = None
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    @property
    def raw(self):
        if self._mm is None:
            return None
        return self._struct_cls.from_buffer_copy(self._mm)  # type: ignore

    def write_begin(self):
        """Bump sequence to odd, memory barrier."""
        self._obj.sequence += 1
        # Memory barrier via write-back
        self._mm.flush()

    def write_end(self):
        """Bump sequence to even, memory barrier."""
        self._mm.flush()
        self._obj.sequence += 1

    def write_state(self, time_s, pos, orient, lin_vel, ang_vel, lin_acc):
        """Atomically write a full QuadrotorState."""
        self.write_begin()
        obj = self._obj
        obj.time = time_s
        obj.position[0], obj.position[1], obj.position[2] = pos
        (
            obj.orientation[0],
            obj.orientation[1],
            obj.orientation[2],
            obj.orientation[3],
        ) = orient
        obj.linear_velocity[0], obj.linear_velocity[1], obj.linear_velocity[2] = lin_vel
        obj.angular_velocity[0], obj.angular_velocity[1], obj.angular_velocity[2] = (
            ang_vel
        )
        (
            obj.linear_acceleration[0],
            obj.linear_acceleration[1],
            obj.linear_acceleration[2],
        ) = lin_acc
        obj.timestamp_ns = int(time.monotonic_ns())
        self.write_end()

    def write_control(self, thrust, torque_x, torque_y, torque_z):
        """Atomically write a full QuadrotorControl."""
        self.write_begin()
        obj = self._obj
        obj.thrust = thrust
        obj.torque[0] = torque_x
        obj.torque[1] = torque_y
        obj.torque[2] = torque_z
        obj.timestamp_ns = int(time.monotonic_ns())
        self.write_end()

    @property
    def raw(self):
        return self._obj

    def read(self, dst) -> bool:
        """Read consistent snapshot from the writable segment."""
        if self._mm is None:
            return False
        before = struct.unpack_from("Q", self._mm, 0)[0]
        if before & 1:
            return False
        buf = bytes(self._mm[: ctypes.sizeof(dst)])
        ctypes.memmove(ctypes.addressof(dst), buf, len(buf))
        after = struct.unpack_from("Q", self._mm, 0)[0]
        return before == after

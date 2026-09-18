#!/usr/bin/env python3
"""
fairino_8083_listener.py

Pure listen-only client for the Fairino controller's port-8083 status feedback
stream (per "COLLABORATIVE ROBOT 8083 PORT STATUS FEEDBACK V3.9.8").

This is genuinely read-only by construction: after the initial TCP connect,
this client never calls send() at all. The controller pushes a frame every
100ms (configurable 8-100ms in system settings) and this client only recv()s,
validates, and parses. There is no command path here for motion or any other
write -- unlike Robot.RPC()/XML-RPC, there is nothing to accidentally misuse.

Frame format (Table 2-1):
    0x5A5A | CNT (u8) | LEN (u16) | DATA (LEN bytes) | CHECKSUM (u16)
    - Frame Header: fixed 0x5A5A (2 bytes; same byte repeated, so endianness
      doesn't matter for detecting it)
    - Frame Count: rolling 0-255 counter
    - Length: byte length of DATA
    - Checksum: sum of ALL bytes from frame header through data content,
      truncated to uint16 (assumed simple modulo-65536 sum; the doc doesn't
      spell out overflow behavior beyond "sum all bytes" -- verify against a
      live capture if frames start getting rejected)

ASSUMPTION FLAGGED: struct field byte order is assumed little-endian, since
that's standard for the x86 control-box CPUs Fairino uses. This has NOT been
verified against a live frame capture. If parsed values look like garbage
(e.g. joint angles in the billions, or the checksum never matches), the first
thing to try is switching every '<' below to '>'.

Data content layout is transcribed field-by-field from Table 2-2 (main
fields), Table 2-3 (per-axis external axis struct, x4), and Table 2-4
(welding struct). Verified: struct.calcsize of DATA_FMT == 650, matching the
sum of per-field byte counts listed in the doc.
"""
import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

FRAME_HEADER = 0x5A5A

# Data content format (see docstring above for field-by-field derivation)
DATA_FMT = (
    "<"
    "B"      # program_state
    "B"      # error_code
    "B"      # robot_mode
    "6d"     # jt_cur_pos[6]        deg
    "6d"     # tl_cur_pos[6]        mm/deg
    "i"      # toolNum
    "6d"     # jt_cur_tor[6]        N*m
    "20s"    # program_name[20]
    "B"      # prog_total_line
    "B"      # prog_cur_line
    "B"      # cl_dgt_output_h
    "B"      # cl_dgt_output_l
    "B"      # tl_dgt_output_l
    "B"      # cl_dgt_input_h
    "B"      # cl_dgt_input_l
    "B"      # tl_dgt_input_l
    "6d"     # FT_data[6]           N / N*m
    "B"      # FT_ActStatus
    "B"      # EmergencyStop
    "i"      # robot_motion_done
    "B"      # gripper_motion_done
    "B"      # servo_id
    "i"      # servo_errcode
    "i"      # servo_state
    "d"      # servo_actual_pos
    "f"      # servo_actual_speed
    "f"      # servo_actual_torque
    "B"      # exaxis_out_slimit_error
    + "ddi9B" * 4  # exaxis_status[4]: (pos, speed, errcode, 9 status bytes) x4
    + "B"    # exaxis_active_flag
    "B"      # exaxis_motion_status
    "2H"     # cl_analog_input[2]
    "H"      # tl_analog_input
    "2H"     # cl_analog_output[2]
    "H"      # tl_analog_output
    "B"      # gripper_fault_id
    "H"      # gripper_fault
    "H"      # gripper_active
    "B"      # gripper_position
    "b"      # gripper_speed
    "b"      # gripper_current
    "i"      # gripper_temp
    "i"      # gripper_voltage
    "f"      # gripper_rotNum
    "B"      # gripper_rotSpeed
    "B"      # gripper_rotTorque
    "i"      # main_errcode
    "i"      # sub_errcode
    "2B"     # welding_state: breakOffState, weldArcState
    "i"      # smartToolState
    "6d"     # toolCoord[6]
    "6d"     # wobjCoord[6]
    "6d"     # exToolCoord[6]
    "6d"     # exAxisCoord[6]
    "d"      # load
    "3d"     # loadCog[3]
)
DATA_LEN = struct.calcsize(DATA_FMT)
assert DATA_LEN == 650, f"format/table mismatch: {DATA_LEN} != 650"

ERROR_CODES = {
    0: "No faults", 1: "Drive failure", 2: "Soft limit exceeded",
    3: "Collision failure", 4: "Singular pose", 5: "Slave error",
    6: "Command point incorrect", 7: "IO error", 8: "Axle device error",
    9: "File error", 10: "Parameter incorrect",
    11: "Extension shaft exceeds soft limit", 12: "Joint configuration warning",
}


@dataclass
class RobotStatus:
    program_state: int
    error_code: int
    robot_mode: int
    jt_cur_pos: tuple
    tl_cur_pos: tuple
    toolNum: int
    jt_cur_tor: tuple
    program_name: str
    prog_total_line: int
    prog_cur_line: int
    emergency_stop: bool
    robot_motion_done: bool
    main_errcode: int
    sub_errcode: int
    # keep the raw unpacked tuple around for anything not surfaced above
    raw: tuple = field(repr=False, default=())

    @property
    def error_description(self) -> str:
        return ERROR_CODES.get(self.error_code, f"Unknown code {self.error_code}")


def _checksum(payload: bytes) -> int:
    """Sum of all bytes from frame header through data content, mod 65536."""
    return sum(payload) & 0xFFFF


def _parse_status(data: bytes) -> RobotStatus:
    vals = struct.unpack(DATA_FMT, data)
    idx = 0

    def take(n=1):
        nonlocal idx
        v = vals[idx:idx + n] if n > 1 else vals[idx]
        idx += n
        return v

    program_state = take()
    error_code = take()
    robot_mode = take()
    jt_cur_pos = take(6)
    tl_cur_pos = take(6)
    toolNum = take()
    jt_cur_tor = take(6)
    program_name = take().split(b"\x00", 1)[0].decode(errors="replace")
    prog_total_line = take()
    prog_cur_line = take()
    take(6)          # cl/tl digital IO bytes (26-31) -- not surfaced by default
    take(6)          # FT_data[6] (32-37) -- not surfaced by default
    take()           # FT_ActStatus (38)
    emergency_stop = bool(take())      # (39)
    robot_motion_done = bool(take())   # (40)
    take()           # gripper_motion_done (41)
    take()           # servo_id (42)
    take()           # servo_errcode (43)
    take()           # servo_state (44)
    take()           # servo_actual_pos (45)
    take()           # servo_actual_speed (46)
    take()           # servo_actual_torque (47)
    take()           # exaxis_out_slimit_error (48)
    take(12 * 4)     # exaxis_status[4] (49) -- "ddi9B" = 12 values/axis, x4 axes
    take()           # exaxis_active_flag (50)
    take()           # exaxis_motion_status (51)
    take(2)          # cl_analog_input[2] (52)
    take()           # tl_analog_input (53)
    take(2)          # cl_analog_output[2] (54)
    take()           # tl_analog_output (55)
    take()           # gripper_fault_id (56)
    take()           # gripper_fault (57)
    take()           # gripper_active (58)
    take()           # gripper_position (59)
    take()           # gripper_speed (60)
    take()           # gripper_current (61)
    take()           # gripper_temp (62)
    take()           # gripper_voltage (63)
    take()           # gripper_rotNum (64)
    take()           # gripper_rotSpeed (65)
    take()           # gripper_rotTorque (66)
    main_errcode = take()   # (67)
    sub_errcode = take()    # (68)
    # fields 69-76 (welding state, smartToolState, coordinate frames, load)
    # remain available via `raw` if you need them -- add take() calls above
    # in DATA_FMT order to surface them as named attributes too.

    return RobotStatus(
        program_state=program_state,
        error_code=error_code,
        robot_mode=robot_mode,
        jt_cur_pos=jt_cur_pos,
        tl_cur_pos=tl_cur_pos,
        toolNum=toolNum,
        jt_cur_tor=jt_cur_tor,
        program_name=program_name,
        prog_total_line=prog_total_line,
        prog_cur_line=prog_cur_line,
        emergency_stop=emergency_stop,
        robot_motion_done=robot_motion_done,
        main_errcode=main_errcode,
        sub_errcode=sub_errcode,
        raw=vals,
    )


class Fairino8083Listener:
    """
    Listen-only client. Connects, reads frames forever, calls `on_status`
    with a parsed RobotStatus for each valid frame. Never sends anything.
    """

    def __init__(self, ip: str, port: int = 8083, on_status=None,
                 reconnect_backoff_max: float = 10.0):
        self.ip = ip
        self.port = port
        self.on_status = on_status or (lambda s: None)
        self.reconnect_backoff_max = reconnect_backoff_max
        self._sock: Optional[socket.socket] = None
        self._buf = bytearray()
        self._running = False

    def _connect(self):
        backoff = 1.0
        while self._running:
            try:
                s = socket.create_connection((self.ip, self.port), timeout=5.0)
                s.settimeout(2.0)
                self._sock = s
                self._buf.clear()
                print(f"[fairino_8083] connected to {self.ip}:{self.port}")
                return
            except OSError as exc:
                print(f"[fairino_8083] connect failed: {exc}; retrying in {backoff:.0f}s")
                time.sleep(backoff)
                backoff = min(backoff * 2, self.reconnect_backoff_max)

    def _close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def run(self):
        """Blocking loop. Ctrl+C or call .stop() from another thread to exit."""
        self._running = True
        self._connect()
        while self._running:
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    raise ConnectionError("peer closed connection")
                self._buf.extend(chunk)
                self._drain_frames()
            except socket.timeout:
                continue  # no data this cycle, loop again
            except OSError as exc:
                print(f"[fairino_8083] socket error: {exc}; reconnecting")
                self._close()
                self._connect()

    def stop(self):
        self._running = False
        self._close()

    def _drain_frames(self):
        # header(2) + count(1) + len(2) = 5 byte prefix before DATA
        while True:
            # resync to the next 0x5A 0x5A if the buffer doesn't start with it
            if len(self._buf) >= 2 and not (self._buf[0] == 0x5A and self._buf[1] == 0x5A):
                pos = self._buf.find(b"\x5A\x5A", 1)
                if pos == -1:
                    self._buf.clear()
                else:
                    del self._buf[:pos]
                continue

            if len(self._buf) < 5:
                return  # need more bytes for header+count+len

            _, cnt, length = struct.unpack("<HBH", self._buf[0:5])
            total_len = 5 + length + 2  # + checksum
            if len(self._buf) < total_len:
                return  # wait for the rest of the frame

            frame = bytes(self._buf[:total_len])
            del self._buf[:total_len]

            payload, chk_bytes = frame[:5 + length], frame[5 + length:5 + length + 2]
            (chk_recv,) = struct.unpack("<H", chk_bytes)
            chk_calc = _checksum(payload)
            if chk_calc != chk_recv:
                print(f"[fairino_8083] checksum mismatch (cnt={cnt}): "
                      f"got {chk_recv:#06x} expected {chk_calc:#06x}, dropping frame")
                continue

            if length != DATA_LEN:
                print(f"[fairino_8083] unexpected data length {length} "
                      f"(expected {DATA_LEN}); skipping -- controller firmware "
                      f"version may add/remove fields vs this parser")
                continue

            try:
                status = _parse_status(frame[5:5 + length])
            except struct.error as exc:
                print(f"[fairino_8083] parse error: {exc}")
                continue

            self.on_status(status)


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ip", default="192.168.58.25")
    p.add_argument("--port", type=int, default=8083)
    args = p.parse_args()

    def print_status(s: RobotStatus):
        print(
            f"state={s.program_state} mode={s.robot_mode} "
            f"err={s.error_code}({s.error_description}) "
            f"estop={s.emergency_stop} motion_done={s.robot_motion_done} "
            f"jt_pos={['%.2f' % v for v in s.jt_cur_pos]}"
        )

    listener = Fairino8083Listener(args.ip, args.port, on_status=print_status)
    try:
        listener.run()
    except KeyboardInterrupt:
        listener.stop()

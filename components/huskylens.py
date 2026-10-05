"""
HuskyLens MagicBot component (I2C or UART).

Publishes Limelight-style data to the NetworkTables "huskylens" table.

Published keys (all under /huskylens):
    tv        1 if a valid target exists, else 0
    tx        horizontal offset from crosshair to target, degrees (+ = right)
    ty        vertical offset from crosshair to target, degrees   (+ = up)
    ta        target area, % of image (0-100)
    ts        arrow angle from vertical, degrees (line tracking); 0 for blocks
    tl        pipeline latency, ms (serial request -> response round trip)
    tshort    shortest side of the target bounding box, pixels
    tlong     longest side of the target bounding box, pixels
    thor      horizontal side of the target bounding box, pixels
    tvert     vertical side of the target bounding box, pixels
    tid       HuskyLens learned ID of the primary target (-1 if none)
    getpipe   algorithm currently active on the camera
    pipeline  (writable) set this to switch HuskyLens algorithm:
                0 face recognition      4 color recognition
                1 object tracking       5 tag recognition
                2 object recognition    6 object classification
                3 line tracking
    connected 1 if the camera answered recently, else 0
    count     number of targets in the last frame

    Per-target arrays (every target in the frame, sorted like the primary):
    all_tx, all_ty, all_ta, all_id

Optional botpose (enable_botpose = True, tag recognition algorithm only):
    botpose_wpiblue  [x, y, z, roll, pitch, yaw, latency_ms, tag_count,
                      tag_span_m, avg_tag_dist_m, avg_tag_area_pct]
                     meters / degrees, WPILib blue-origin field coordinates.
                     z, roll, pitch are always 0 (this is a 2D estimate).
    NOTE: the HuskyLens reports only an axis-aligned bounding box and an ID
    for tags, not the tag corners/orientation. Position is therefore estimated
    from the box size (or its vertical position) plus the pinhole model, and
    the robot heading MUST be supplied from your gyro via set_robot_heading().

Transport (set `interface`):
    "i2c"  (default) roboRIO MXP I2C bus (i2c_port = 1), e.g. via the navX-MXP
           I2C connector, HuskyLens address 0x32. On the camera set
           General Settings -> Protocol Type -> I2C. No extra dependencies.
           Do NOT also talk to the navX over I2C (see address note in docs).
    "uart" 3.3 V TTL serial via pyserial (add "pyserial" to your robotpy
           requirements). The roboRIO RS-232 port is NOT TTL compatible, so use
           a USB-to-TTL adapter (/dev/ttyUSB0) and set the camera to UART.
"""

import math
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import ntcore
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d


# --- HuskyLens protocol constants ------------------------------------------
_HEADER = b"\x55\xaa\x11"

_CMD_REQUEST_ALL = 0x20  # request blocks + arrows
_CMD_RETURN_INFO = 0x29
_CMD_RETURN_BLOCK = 0x2A
_CMD_RETURN_ARROW = 0x2B
_CMD_REQUEST_KNOCK = 0x2C
_CMD_REQUEST_ALGORITHM = 0x2D
_CMD_RETURN_OK = 0x2E

# Native image resolution of the HuskyLens
IMG_W = 320
IMG_H = 240


class _I2CTransport:
    """
    Minimal byte-stream wrapper over wpilib.I2C so the HuskyLens framing code
    can be shared with UART. The camera streams its reply when read; we pull it
    in 16-byte chunks (what the official library does) and hand out bytes.
    Filler bytes between frames are harmless: the frame parser resyncs on the
    0x55 0xAA 0x11 header.
    """

    CHUNK = 16

    def __init__(self, port_index: int, address: int):
        import wpilib

        P = wpilib.I2C.Port
        if port_index == 0:
            port = getattr(P, "kPort0", None) or getattr(P, "kOnboard")
        else:
            port = getattr(P, "kPort1", None) or getattr(P, "kMXP")
        self._i2c = wpilib.I2C(port, address)
        self._buf = bytearray()

    def write(self, data: bytes) -> None:
        # writeBulk returns True if the transaction was aborted (NACK, etc.)
        if self._i2c.writeBulk(bytes(data)):
            raise OSError("I2C write failed (no ACK from HuskyLens?)")

    def _fetch(self) -> bytes:
        result = self._i2c.readOnly(self.CHUNK)
        # Depending on the robotpy version this is bytes, or (aborted, bytes)
        if isinstance(result, (tuple, list)):
            data = next((x for x in result if isinstance(x, (bytes, bytearray))), b"")
        else:
            data = result if isinstance(result, (bytes, bytearray)) else b""
        if not data:
            time.sleep(0.001)
        return bytes(data)

    def read(self, n: int) -> bytes:
        deadline = time.monotonic() + 0.05
        while len(self._buf) < n and time.monotonic() < deadline:
            self._buf.extend(self._fetch())
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def reset_input_buffer(self) -> None:
        self._buf.clear()

    def close(self) -> None:
        pass


@dataclass
class HuskyTarget:
    x: float  # center x (block) or head x (arrow), px
    y: float  # center y (block) or head y (arrow), px
    w: float  # width px (0 for arrows)
    h: float  # height px (0 for arrows)
    id: int
    is_arrow: bool = False
    tail_x: float = 0.0
    tail_y: float = 0.0


@dataclass
class _Snapshot:
    targets: List[HuskyTarget] = field(default_factory=list)
    latency_ms: float = 0.0
    timestamp: float = 0.0  # time.monotonic() of last good response
    algorithm: int = 0


class HuskyLens:
    # ---- configuration (override via class attributes before robot init) ---
    # Transport: "i2c" or "uart"
    interface: str = "i2c"

    # I2C settings. i2c_port: 0 = roboRIO onboard I2C (known lockup issue,
    # avoid), 1 = MXP I2C. The navX-MXP's I2C connector is a pass-through of
    # the MXP I2C bus, so a HuskyLens plugged into it is on port 1.
    i2c_port: int = 1
    i2c_address: int = 0x32  # fixed HuskyLens address

    # UART settings (only used when interface == "uart")
    port: str = "/dev/ttyUSB0"
    baudrate: int = 9600  # HuskyLens default
    default_algorithm: int = 1  # object tracking

    # Horizontal FOV of the lens in degrees. This is an approximate default;
    # calibrate for your unit (see note at bottom of file).
    hfov_deg: float = 60.0

    # Primary target selection: "largest" or "closest" (to image center)
    sort_mode: str = "largest"
    # Only consider this learned ID as the primary target (-1 = any)
    id_filter: int = -1

    # Data older than this (seconds) is treated as "no target / disconnected"
    stale_timeout: float = 0.25

    # ---- optional botpose --------------------------------------------------
    # Master switch. When False nothing botpose-related is created/published.
    enable_botpose: bool = False

    # Field tag locations. Provide EITHER an AprilTagFieldLayout (anything with
    # getTagPose(id) -> Pose3d | None) OR a dict {tag_id: (x_m, y_m, z_m)} in
    # WPILib blue-origin coordinates, z = height of the tag CENTER.
    tag_layout: Any = None
    tag_poses: Dict[int, Tuple[float, float, float]] = {}

    # Physical tag edge length in meters (FRC 6.5 in tag = 0.1651 m)
    tag_size_m: float = 0.1651

    # Camera mounting relative to robot center (x fwd, y left, z up, meters;
    # yaw CCW+ degrees, pitch UP+ degrees)
    cam_x: float = 0.0
    cam_y: float = 0.0
    cam_z: float = 0.5
    cam_yaw_deg: float = 0.0
    cam_pitch_deg: float = 0.0

    # How range to a tag is computed:
    #   "size"   - from apparent box size (needs accurate tag_size_m / hfov)
    #   "height" - from vertical angle + known tag/camera heights (like the
    #              classic Limelight ty distance trick; needs cam pitch/z)
    botpose_distance_method: str = "size"
    # Ignore tag estimates farther than this (estimates degrade with range)
    botpose_max_range_m: float = 4.0

    # -----------------------------------------------------------------------

    def setup(self) -> None:
        """MagicBot calls this after injection."""
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("huskylens")

        self._pub_tv = table.getIntegerTopic("tv").publish()
        self._pub_tx = table.getDoubleTopic("tx").publish()
        self._pub_ty = table.getDoubleTopic("ty").publish()
        self._pub_ta = table.getDoubleTopic("ta").publish()
        self._pub_ts = table.getDoubleTopic("ts").publish()
        self._pub_tl = table.getDoubleTopic("tl").publish()
        self._pub_tshort = table.getDoubleTopic("tshort").publish()
        self._pub_tlong = table.getDoubleTopic("tlong").publish()
        self._pub_thor = table.getDoubleTopic("thor").publish()
        self._pub_tvert = table.getDoubleTopic("tvert").publish()
        self._pub_tid = table.getIntegerTopic("tid").publish()
        self._pub_getpipe = table.getIntegerTopic("getpipe").publish()
        self._pub_connected = table.getIntegerTopic("connected").publish()
        self._pub_count = table.getIntegerTopic("count").publish()

        self._pub_all_tx = table.getDoubleArrayTopic("all_tx").publish()
        self._pub_all_ty = table.getDoubleArrayTopic("all_ty").publish()
        self._pub_all_ta = table.getDoubleArrayTopic("all_ta").publish()
        self._pub_all_id = table.getIntegerArrayTopic("all_id").publish()

        # Writable like the Limelight "pipeline" entry
        self._pipeline_entry = table.getIntegerTopic("pipeline").getEntry(
            self.default_algorithm
        )
        self._pipeline_entry.setDefault(self.default_algorithm)

        # Pinhole focal length in pixels derived from the horizontal FOV
        self._focal_px = (IMG_W / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

        self._lock = threading.Lock()
        self._snapshot = _Snapshot(algorithm=self.default_algorithm)
        self._requested_algorithm = int(self.default_algorithm)
        self._active_algorithm = -1

        self._ser = None  # transport (I2C wrapper or serial.Serial)
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="HuskyLensUART", daemon=True
        )
        self._thread.start()

        # Public (non-NT) results, handy for other components / autonomous
        self.has_target = False
        self.tx = 0.0
        self.ty = 0.0
        self.ta = 0.0

        # Botpose state (only used when enable_botpose is True)
        self.botpose: Optional[Pose2d] = None  # None = no valid estimate
        self.botpose_timestamp = 0.0  # FPGA seconds, capture-time estimate
        self.botpose_is_new = False  # True only on the first loop per frame
        self._last_botpose_snap = None
        self._heading_deg: Optional[float] = None
        self._pub_botpose = None
        if self.enable_botpose:
            self._pub_botpose = table.getDoubleArrayTopic("botpose_wpiblue").publish()

    # ---- MagicBot loop ----------------------------------------------------

    def execute(self) -> None:
        # Pass any pipeline change from NT to the serial thread
        requested = int(self._pipeline_entry.get())
        if 0 <= requested <= 6:
            with self._lock:
                self._requested_algorithm = requested

        with self._lock:
            snap = self._snapshot

        fresh = (time.monotonic() - snap.timestamp) < self.stale_timeout
        targets = snap.targets if fresh else []

        ordered = self._sort_targets(targets)
        primary = ordered[0] if ordered else None

        self._pub_connected.set(1 if fresh else 0)
        self._pub_getpipe.set(snap.algorithm)
        self._pub_count.set(len(ordered))
        self._pub_tl.set(snap.latency_ms)

        if primary is None:
            self.has_target = False
            self.tx = self.ty = self.ta = 0.0
            self._pub_tv.set(0)
            self._pub_tx.set(0.0)
            self._pub_ty.set(0.0)
            self._pub_ta.set(0.0)
            self._pub_ts.set(0.0)
            self._pub_tshort.set(0.0)
            self._pub_tlong.set(0.0)
            self._pub_thor.set(0.0)
            self._pub_tvert.set(0.0)
            self._pub_tid.set(-1)
        else:
            tx, ty = self._to_angles(primary.x, primary.y)
            ta = self._area_percent(primary)
            self.has_target = True
            self.tx, self.ty, self.ta = tx, ty, ta

            self._pub_tv.set(1)
            self._pub_tx.set(tx)
            self._pub_ty.set(ty)
            self._pub_ta.set(ta)
            self._pub_ts.set(self._arrow_angle(primary))
            self._pub_tshort.set(min(primary.w, primary.h))
            self._pub_tlong.set(max(primary.w, primary.h))
            self._pub_thor.set(primary.w)
            self._pub_tvert.set(primary.h)
            self._pub_tid.set(primary.id)

        angles = [self._to_angles(t.x, t.y) for t in ordered]
        self._pub_all_tx.set([a[0] for a in angles])
        self._pub_all_ty.set([a[1] for a in angles])
        self._pub_all_ta.set([self._area_percent(t) for t in ordered])
        self._pub_all_id.set([t.id for t in ordered])

        self._update_botpose(snap, fresh)

    def on_disable(self) -> None:
        pass  # keep the camera thread alive; it is cheap and keeps data fresh

    def stop(self) -> None:
        """Call from robot code if you want to shut the thread down."""
        self._running = False

    # ---- botpose ----------------------------------------------------------

    def set_robot_heading(self, heading_deg: float) -> None:
        """
        Feed the field-relative robot heading (degrees, CCW positive, 0 along
        +X of the blue-origin field). Call every loop from your drivetrain,
        e.g. self.huskylens.set_robot_heading(self.gyro.getRotation2d().degrees())
        """
        self._heading_deg = heading_deg

    def _tag_field_xyz(self, tag_id: int) -> Optional[Tuple[float, float, float]]:
        if self.tag_layout is not None:
            pose = self.tag_layout.getTagPose(tag_id)
            if pose is not None:
                return (pose.X(), pose.Y(), pose.Z())
        return self.tag_poses.get(tag_id)

    def _estimate_from_tag(self, t: HuskyTarget, tag_xyz, heading_rad: float):
        """Return (robot_x, robot_y, horizontal_dist) in field frame, or None."""
        f = self._focal_px
        u = (t.x - IMG_W / 2.0) / f  # right of center
        v = (t.y - IMG_H / 2.0) / f  # below center
        pitch = math.radians(self.cam_pitch_deg)

        # Depth Z along the optical axis
        if self.botpose_distance_method == "height":
            denom = math.sin(pitch) - v * math.cos(pitch)
            if denom <= 1e-3:
                return None
            z = (tag_xyz[2] - self.cam_z) / denom
        else:
            # min() is less inflated than max() when the tag is rotated in
            # the image plane (axis-aligned box grows up to 1.41x)
            size_px = min(t.w, t.h)
            if size_px <= 0:
                return None
            z = f * self.tag_size_m / size_px
        if z <= 0:
            return None

        # Tag in camera frame (X right, Y down, Z forward), then into the
        # level robot-aligned frame (fwd, left) accounting for camera pitch
        cx, cy = u * z, v * z
        fwd = z * math.cos(pitch) + cy * math.sin(pitch)
        left = -cx
        dist = math.hypot(fwd, left)
        if dist > self.botpose_max_range_m:
            return None

        # Camera yaw + camera offset -> tag position in robot frame
        yaw = math.radians(self.cam_yaw_deg)
        rx = fwd * math.cos(yaw) - left * math.sin(yaw) + self.cam_x
        ry = fwd * math.sin(yaw) + left * math.cos(yaw) + self.cam_y

        # Robot frame -> field frame, then subtract from known tag position
        fx = rx * math.cos(heading_rad) - ry * math.sin(heading_rad)
        fy = rx * math.sin(heading_rad) + ry * math.cos(heading_rad)
        return (tag_xyz[0] - fx, tag_xyz[1] - fy, dist)

    def _update_botpose(self, snap: _Snapshot, fresh: bool) -> None:
        if not self.enable_botpose:
            return

        self.botpose = None
        self.botpose_is_new = False
        heading = self._heading_deg
        estimates = []  # (x, y, dist, weight, tag_xy)

        # Only meaningful in tag recognition (algorithm 5), with a heading
        if fresh and snap.algorithm == 5 and heading is not None:
            h_rad = math.radians(heading)
            for t in snap.targets:
                if t.is_arrow:
                    continue
                tag_xyz = self._tag_field_xyz(t.id)
                if tag_xyz is None:
                    continue
                est = self._estimate_from_tag(t, tag_xyz, h_rad)
                if est is None:
                    continue
                estimates.append(
                    (est[0], est[1], est[2], self._area_percent(t), tag_xyz)
                )

        if not estimates:
            self._pub_botpose.set([0.0] * 11)
            return

        total_w = sum(e[3] for e in estimates) or 1.0
        x = sum(e[0] * e[3] for e in estimates) / total_w
        y = sum(e[1] * e[3] for e in estimates) / total_w
        avg_dist = sum(e[2] for e in estimates) / len(estimates)
        avg_area = sum(e[3] for e in estimates) / len(estimates)
        span = max(
            (
                math.hypot(a[4][0] - b[4][0], a[4][1] - b[4][1])
                for a in estimates
                for b in estimates
            ),
            default=0.0,
        )

        self.botpose = Pose2d(x, y, Rotation2d.fromDegrees(heading))
        age = time.monotonic() - snap.timestamp
        self.botpose_timestamp = (
            Timer.getFPGATimestamp() - age - snap.latency_ms / 1000.0
        )
        self.botpose_is_new = snap is not self._last_botpose_snap
        self._last_botpose_snap = snap

        yaw_out = (heading + 180.0) % 360.0 - 180.0
        self._pub_botpose.set(
            [
                x,
                y,
                0.0,
                0.0,
                0.0,
                yaw_out,
                snap.latency_ms,
                float(len(estimates)),
                span,
                avg_dist,
                avg_area,
            ]
        )

    # ---- math helpers -----------------------------------------------------

    def _to_angles(self, px: float, py: float):
        """Pixel -> (tx, ty) degrees. +tx = right, +ty = up (Limelight style)."""
        tx = math.degrees(math.atan2(px - IMG_W / 2.0, self._focal_px))
        ty = -math.degrees(math.atan2(py - IMG_H / 2.0, self._focal_px))
        return tx, ty

    @staticmethod
    def _area_percent(t: HuskyTarget) -> float:
        return 100.0 * (t.w * t.h) / (IMG_W * IMG_H)

    @staticmethod
    def _arrow_angle(t: HuskyTarget) -> float:
        """Arrow tail->head angle measured from straight up, + = right."""
        if not t.is_arrow:
            return 0.0
        dx = t.x - t.tail_x
        dy = t.tail_y - t.y  # image y grows downward
        return math.degrees(math.atan2(dx, dy))

    def _sort_targets(self, targets: List[HuskyTarget]) -> List[HuskyTarget]:
        if self.id_filter >= 0:
            targets = [t for t in targets if t.id == self.id_filter]
        if self.sort_mode == "closest":
            return sorted(
                targets,
                key=lambda t: (t.x - IMG_W / 2) ** 2 + (t.y - IMG_H / 2) ** 2,
            )
        return sorted(targets, key=lambda t: t.w * t.h, reverse=True)

    # ---- serial thread ----------------------------------------------------

    def _run(self) -> None:
        while self._running:
            try:
                if self._ser is None:
                    self._connect()
                self._maybe_switch_algorithm()
                self._poll()
                time.sleep(0.005)
            except OSError:
                self._close()
                time.sleep(1.0)
            except Exception:  # never let the thread die silently
                self._close()
                time.sleep(1.0)
        self._close()

    def _connect(self) -> None:
        self._ser = self._open_transport()
        self._active_algorithm = -1
        # Knock until the camera answers (bounded wait)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self._send(_CMD_REQUEST_KNOCK)
            frame = self._read_frame(0.2)
            if frame and frame[0] == _CMD_RETURN_OK:
                return
        raise ConnectionError("HuskyLens did not answer knock")

    def _open_transport(self):
        if self.interface == "i2c":
            return _I2CTransport(self.i2c_port, self.i2c_address)
        if self.interface == "uart":
            import serial  # only required for UART

            return serial.Serial(self.port, self.baudrate, timeout=0.05)
        raise ValueError(f"interface must be 'i2c' or 'uart', got {self.interface!r}")

    def _close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def _maybe_switch_algorithm(self) -> None:
        with self._lock:
            wanted = self._requested_algorithm
        if wanted == self._active_algorithm:
            return
        self._send(_CMD_REQUEST_ALGORITHM, struct.pack("<H", wanted))
        frame = self._read_frame(2.0)  # algorithm switch can be slow
        if frame and frame[0] == _CMD_RETURN_OK:
            self._active_algorithm = wanted
            with self._lock:
                self._snapshot = _Snapshot(algorithm=wanted)

    def _poll(self) -> None:
        self._ser.reset_input_buffer()
        t0 = time.perf_counter()
        self._send(_CMD_REQUEST_ALL)

        frame = self._read_frame(0.2)
        if frame is None or frame[0] != _CMD_RETURN_INFO or len(frame[1]) < 2:
            return  # keep previous snapshot; it will go stale on its own

        count = struct.unpack("<H", frame[1][:2])[0]
        targets: List[HuskyTarget] = []
        for _ in range(count):
            f = self._read_frame(0.1)
            if f is None:
                return  # incomplete frame -> drop it
            cmd, data = f
            if len(data) < 10:
                continue
            a, b, c, d, i = struct.unpack("<5H", data[:10])
            if cmd == _CMD_RETURN_BLOCK:
                targets.append(HuskyTarget(x=a, y=b, w=c, h=d, id=i))
            elif cmd == _CMD_RETURN_ARROW:
                # a,b = tail   c,d = head
                targets.append(
                    HuskyTarget(
                        x=c,
                        y=d,
                        w=0,
                        h=0,
                        id=i,
                        is_arrow=True,
                        tail_x=a,
                        tail_y=b,
                    )
                )

        latency_ms = (time.perf_counter() - t0) * 1000.0
        with self._lock:
            self._snapshot = _Snapshot(
                targets=targets,
                latency_ms=latency_ms,
                timestamp=time.monotonic(),
                algorithm=self._active_algorithm,
            )

    # ---- protocol framing -------------------------------------------------

    def _send(self, cmd: int, data: bytes = b"") -> None:
        frame = _HEADER + bytes([len(data), cmd]) + data
        frame += bytes([sum(frame) & 0xFF])
        self._ser.write(frame)

    def _read_frame(self, timeout: float):
        """Return (cmd, data) for the next valid frame, or None on timeout."""
        deadline = time.monotonic() + timeout
        window = b""
        while time.monotonic() < deadline:
            b = self._ser.read(1)
            if not b:
                continue
            window = (window + b)[-3:]
            if window != _HEADER:
                continue

            hdr = self._ser.read(2)  # length, command
            if len(hdr) < 2:
                window = b""
                continue
            length, cmd = hdr
            data = self._ser.read(length) if length else b""
            chk = self._ser.read(1)
            if len(data) < length or len(chk) < 1:
                window = b""
                continue
            if (sum(_HEADER + hdr + data) & 0xFF) != chk[0]:
                window = b""
                continue  # bad checksum, resync
            return cmd, data
        return None


# ---------------------------------------------------------------------------
# Calibration note: the real HuskyLens field of view differs by unit/lens, so
# measure it. Place a target at a known distance, record its pixel x, and
# solve:  hfov = 2 * atan((IMG_W/2) / focal_px)  with
#         focal_px = (pixel_offset) / tan(known_angle).
# Then set `hfov_deg` accordingly.
# ---------------------------------------------------------------------------

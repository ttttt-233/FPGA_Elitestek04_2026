"""Single-owner UART backend shared by CLI, Qt and future automation/MCP.

No Qt dependency. Only this module imports pyserial. One outstanding sequence;
commands are never automatically retried (a timed-out event may have executed).
"""
from collections import deque
from pathlib import Path
from threading import Event, RLock
import time

try:
    from .protocol import (Algorithm, BAUD, Command, ControlGroup, MediaAction,
                           PacketParser, Status, bcd_ms, decode_control, decode_media,
                           DEVICE_FLAG_NAMES, encode_command)
except ImportError:
    from protocol import (Algorithm, BAUD, Command, ControlGroup, MediaAction,
                          PacketParser, Status, bcd_ms, decode_control, decode_media,
                          DEVICE_FLAG_NAMES, encode_command)


class FPGAError(Exception):
    pass


class CommandError(FPGAError):
    def __init__(self, command, status):
        self.command, self.status = int(command), int(status)
        name = Status(status).name if status in Status._value2member_map_ else str(status)
        super().__init__(f"command 0x{int(command):02X}: {name}")


class ResponseTimeout(FPGAError):
    pass


def list_ports() -> list[str]:
    from serial.tools.list_ports import comports
    return [p.device for p in comports()]


class FPGAClient:
    def __init__(self, port: str, baud: int = BAUD, timeout: float = 1.0, transport_factory=None):
        self.port, self.baud, self.timeout = port, baud, timeout
        self.transport_factory = transport_factory
        self.serial = None
        self.parser = PacketParser()
        self.latest_frame = None
        self._last_rx = 0.0
        self.targets = {}
        self.telemetry_status = {}
        self.version = {}
        self.raw_listeners = []
        self.listeners = []
        self.photo_progress = []
        self._replies = deque(maxlen=32)
        self._sequence = 0
        self._lock = RLock()
        self._cancel = Event()
        self._uncertain = False
        self._raw_stream_receiving = False

    @property
    def connected(self):
        return self.serial is not None

    def connect(self, handshake=True):
        with self._lock:
            if self.connected:
                return self
            self._cancel.clear()
            self._uncertain = False
            self.parser = PacketParser()
            self._replies.clear()
            self.targets.clear()
            self.latest_frame = None
            self._last_rx = time.monotonic()
            self.telemetry_status = {}
            self.version = {}
            factory = self.transport_factory
            if factory is None:
                import serial
                factory = serial.Serial
            self.serial = factory(port=self.port, baudrate=self.baud, timeout=0.01,
                                  write_timeout=self.timeout, rtscts=False, dsrdtr=False)
            try:
                # Windows pyserial defaults to a 4 KiB driver RX queue: only
                # 20 ms at 2 Mbaud. RAW must survive OS scheduling/GC pauses.
                set_buffers = getattr(self.serial, 'set_buffer_size', None)
                if set_buffers is not None:
                    set_buffers(rx_size=1024*1024, tx_size=64*1024)
                if handshake:
                    time.sleep(0.004)  # Let an abandoned FPGA command byte-timeout expire.
                    self.get_version()
                    if self.version["protocol"] != 3:
                        raise FPGAError("control requires protocol v3; capture/watch can read v1/v2 without handshake")
            except Exception:
                self.disconnect()
                raise
            return self

    def cancel(self):
        """Thread-safe cancellation; performs no serial I/O from a UI thread."""
        self._cancel.set()

    def disconnect(self):
        self.cancel()
        with self._lock:
            if self.serial is not None:
                self.serial.close()
                self.serial = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *_):
        self.disconnect()

    def poll(self, wait=False) -> list[dict]:
        with self._lock:
            if self._cancel.is_set():
                raise FPGAError("connection cancelled")
            if self.serial is None:
                raise FPGAError("not connected")
            available = self.serial.in_waiting
            if not available and not wait:
                data = b""
            else:
                data = self.serial.read(min(4096, max(1, available)))
            now = time.monotonic()
            idle = not data and now - self._last_rx >= 0.02
            if data:
                self._last_rx = now
            for callback in tuple(self.raw_listeners):
                callback(data)
            records = self.parser.feed(data, idle=idle)
            for record in records:
                if record["type"] == "frame":
                    self.latest_frame = record
                    self.targets = {t["slot"]: t for t in record["targets"]}
                    self.telemetry_status = record
                elif record["type"] == "target":
                    self.targets[record["slot"]] = record
                elif record["type"] == "status":
                    self.telemetry_status = record
                if (record["type"] in ("ack", "response") or "sequence" in record) and not (
                        record['type'] == 'raw_block' and self._raw_stream_receiving):
                    self._replies.append(record)
                for callback in tuple(self.listeners):
                    callback(record)
            return records

    def transact(self, command: int, value: int = 0) -> dict:
        with self._lock:
            if self._uncertain:
                raise FPGAError("previous command timed out; reconnect before issuing more commands")
            if self.serial is None:
                raise FPGAError("not connected")
            self.poll()
            self._replies.clear()
            sequence = self._sequence
            self._sequence = (sequence + 1) & 255
            packet = encode_command(command, value, sequence)
            try:
                written = self.serial.write(packet)
            except OSError as exc:
                self._uncertain = True
                raise FPGAError(f"serial write failed; outcome unknown: {exc}") from exc
            if written != len(packet):
                self._uncertain = True
                raise FPGAError("short serial write; reconnect before retrying")
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                self.poll(wait=True)
                while self._replies:
                    reply = self._replies.popleft()
                    if reply.get("sequence") != sequence:
                        continue
                    if reply["type"] == "target":
                        if int(command) != Command.GET_TARGET or reply["slot"] != value:
                            continue
                    elif reply["command"] != int(command):
                        continue
                    if int(command) == Command.RAW_BLOCK and (reply['type'] != 'raw_block' or reply['index'] != value):
                        if reply['type'] == 'ack' and reply['status'] != Status.OK:
                            raise CommandError(command, reply['status'])
                        continue
                    if reply["type"] == "ack" and reply["status"] != Status.OK:
                        raise CommandError(command, reply["status"])
                    if int(command) in (0, 1, 2, 0x40) and reply["type"] != "response":
                        continue
                    if int(command) == Command.GET_TARGET and reply["type"] != "target":
                        continue
                    return reply
            self._uncertain = True
            raise ResponseTimeout(f"no matching response for command 0x{int(command):02X}, sequence {sequence}; outcome unknown")

    def _read(self, command, page):
        return self.transact(command, page)["value"]

    def save_playback_photo(self, stem):
        """Lease the selected SNAP, download CRC16 blocks, save exactly 3 files.

        All serial I/O stays on the owner thread. Missing/corrupt blocks are
        explicitly re-read under the same lease; missing RAW is never guessed.
        """
        try:
            from .raw_photo import parse_header, save_photo
        except ImportError:
            from raw_photo import parse_header, save_photo
        # OUTPUT/K2 reloads the FPGA without closing the USB serial handle.
        # Never derive an export address from connect-time geometry.
        self.get_version()
        if self.version.get('build_id', 0) < 0x0304:
            raise FPGAError('SNAP RAW export requires firmware build 0304 or newer')
        status = self.get_status()
        if status['control']['group'] != ControlGroup.PLAYBACK or not status['media']['selected_valid'] or status['media']['video']:
            raise FPGAError('Select a SNAP photograph in PLAYBACK before saving')
        self.transact(Command.RAW_CLOSE)  # Recover an abandoned earlier host lease.
        ready_deadline = time.monotonic() + 1.5
        while True:
            try:
                self.transact(Command.RAW_OPEN)
                break
            except CommandError as exc:
                # BUSY explicitly means no lease was acquired. Let the existing
                # HDMI frame boundary finish switching to the chosen SNAP.
                if exc.status != Status.BUSY or time.monotonic() >= ready_deadline: raise
                if self._cancel.wait(0.01): raise FPGAError('photo export cancelled')
        try:
            pixels = self.version['width'] * self.version['height']
            def block(index):
                busy_deadline=time.monotonic()+2.0
                timeouts=0
                while True:
                    try:
                        return self.transact(Command.RAW_BLOCK, index)['data']
                    except ResponseTimeout:
                        timeouts+=1
                        if timeouts>=3:raise
                        # Only this immutable, leased block READ is idempotent.
                        # Never clear uncertainty/retry a control event or lease.
                        self._uncertain=False
                    except CommandError as exc:
                        if exc.status==Status.BUSY and time.monotonic()<busy_deadline:
                            if self._cancel.wait(.01):raise FPGAError('photo export cancelled')
                            continue
                        raise FPGAError(f'{exc}; block {index}, profile '
                                        f"{self.version['width']}x{self.version['height']}, "
                                        f"build {self.version['build_id']:04X}") from exc
            first_header = 0xfff8 if self.version['build_id'] >= 0x0305 else pixels//16
            header = b''.join(block(first_header+i) for i in range(8))
            metadata = parse_header(header)
            if (metadata['width'], metadata['height']) != (self.version['width'], self.version['height']):
                raise FPGAError('photo/profile geometry mismatch')
            data = bytearray(pixels)
            seen = bytearray(pixels//16)
            received=0
            last_raw=time.monotonic()
            sequence = self._sequence
            stream_error = []
            def collect(record):
                nonlocal received,last_raw
                if record.get('sequence') != sequence or stream_error: return
                if record['type'] == 'ack' and record['command'] == Command.RAW_BLOCK and record['status']:
                    stream_error.append(CommandError(Command.RAW_BLOCK, record['status']))
                elif record['type'] == 'raw_block':
                    index=record['index']
                    if index>=len(seen):
                        self._uncertain = True
                        stream_error.append(FPGAError('RAW stream block outside photo geometry; reconnect'))
                        return
                    start=index*16;last_raw=time.monotonic()
                    if seen[index]:
                        if data[start:start+16]!=record['data']:
                            self._uncertain=True
                            stream_error.append(FPGAError('leased RAW block changed; reconnect'))
                        return
                    data[start:start+16]=record['data'];seen[index]=1;received+=16
                    if index%128==0 or received==pixels:
                        for callback in tuple(self.photo_progress): callback(received,pixels)
            self.listeners.append(collect)
            self._raw_stream_receiving = True
            try:
                errors = self.parser.errors
                self.transact(Command.RAW_STREAM)
                deadline = time.monotonic() + max(30.0, pixels*15/self.baud + 10.0)
                while not seen[-1] and not stream_error:
                    self.poll(wait=True)
                    # CRC-invalid packets never enter collect(). Retain holes,
                    # drain the stream, then explicitly re-read checked blocks.
                    if received and time.monotonic()-last_raw>.25:break
                    if time.monotonic() >= deadline:
                        self._uncertain = True
                        raise ResponseTimeout('RAW stream timed out; reconnect')
                if stream_error: raise stream_error[0]
            finally:
                self.listeners.remove(collect)
                self._raw_stream_receiving = False
            missing=[i for i,valid in enumerate(seen) if not valid]
            for index in missing:
                data[index*16:index*16+16]=block(index)
                seen[index]=1;received+=16
                for callback in tuple(self.photo_progress):callback(received,pixels)
            if received!=pixels or not all(seen):raise FPGAError('RAW download incomplete')
            self.last_photo_transfer=dict(bytes=pixels,repaired_blocks=len(missing),
                parser_errors=self.parser.errors-errors)
        finally:
            if not self._cancel.is_set() and not self._uncertain:
                self.transact(Command.RAW_CLOSE)
        if self._cancel.is_set(): raise FPGAError('photo export cancelled')
        return save_photo(bytes(data), header, stem)

    def get_version(self):
        self.version = dict(zip(("protocol", "build_id", "width", "height", "fps"),
                               (self._read(Command.GET_VERSION, page) for page in range(5))))
        return dict(self.version)

    def get_control_state(self):
        words = [self._read(Command.GET_CONTROL_STATE, page) for page in range(2)]
        return decode_control(*words)

    def get_status(self):
        words = [self._read(Command.GET_STATUS, page) for page in range(6)]
        result = dict(fps_x10=words[0], fps=words[0] / 10, failed_frames=words[1], target_count=words[2] & 255,
                      device={name: bool(words[2] & (1 << (8 + bit))) for bit, name in enumerate(DEVICE_FLAG_NAMES)},
                      uart_rx_errors=words[3] & 255, uart_checksum_errors=words[3] >> 8,
                      uart_command_count=words[4] & 255, uart_tx_overruns=words[4] >> 8,
                      media=decode_media(words[5]), control=self.get_control_state())
        if self.version.get("build_id", 0) >= 0x0302:
            counts = self._read(Command.GET_CONTROL_STATE, 3)
            result["media"].update(photo_count=counts & 31, index=(counts >> 5) & 31,
                                   count=(counts >> 10) & 31)
        if self.version.get('build_id',0) >= 0x0306:
            result['raw_diagnostics']=self._read(Command.GET_CONTROL_STATE,2)
        try:
            actual = self._read(Command.GET_STATUS, 6)
            result.update(actual_algorithm=(actual >> 15) & 1, actual_high=actual & 4095,
                          actual_low=self._read(Command.GET_STATUS, 7), actual_filter=(actual >> 14) & 1)
        except CommandError as exc:
            if exc.status != Status.BUSY:
                raise
            result.update(actual_algorithm=None, actual_high=None, actual_low=None)
        return result

    def get_target(self, slot):
        # Explicit BUSY permits retrying a read; never retry an event or timeout.
        for attempt in range(3):
            try:
                return self.transact(Command.GET_TARGET, slot)
            except CommandError as exc:
                if exc.status != Status.BUSY or attempt == 2:
                    raise
                if self._cancel.wait(0.001):
                    raise FPGAError("connection cancelled")

    def get_targets(self):
        """Six independent coherent records, not an atomic six-target frame."""
        return [self.get_target(slot) for slot in range(6)]

    def set_threshold(self, value): return self.transact(Command.SET_THRESHOLD, value)
    def set_auto(self, enabled): return self.transact(Command.SET_AUTO, int(enabled))
    def set_algorithm(self, algorithm): return self.transact(Command.SET_ALGORITHM, int(algorithm))
    def set_shape_enable(self, enabled): return self.transact(Command.SET_SHAPE, int(enabled))
    def set_filter(self, value): return self.transact(Command.SET_FILTER, int(value))
    def set_mode(self, mode): return self.transact(Command.SET_MODE, int(mode))
    def set_menu(self, group, item=0):
        if not 0 <= int(group) <= 5 or not 0 <= item <= 3:
            raise ValueError("group=0..5, item=0..3")
        return self.transact(Command.SET_MENU, (item << 3) | int(group))
    def set_osd_enable(self, enabled): return self.transact(Command.SET_OSD, int(enabled))
    def set_quick_hide(self, enabled): return self.transact(Command.SET_QUICK_HIDE, int(enabled))
    def set_osd_mask(self, mask): return self.transact(Command.SET_OSD_MASK, mask)
    def set_profiler(self, enabled): return self.transact(Command.SET_PROFILER, int(enabled))
    def media(self, action): return self.transact(Command.MEDIA, int(action))
    def snapshot(self): return self.media(MediaAction.SNAPSHOT)
    def video_start(self): return self.media(MediaAction.VIDEO_START)
    def video_stop(self): return self.media(MediaAction.VIDEO_STOP)
    def pre_capture(self): return self.media(MediaAction.PRE)
    def media_prev(self): return self.media(MediaAction.PREVIOUS)
    def media_next(self): return self.media(MediaAction.NEXT)
    def play(self): return self.media(MediaAction.PLAY)
    def pause(self): return self.media(MediaAction.PAUSE)
    def profile_request(self, profile=None):
        if profile is not None:
            return self.transact(Command.SET_PROFILE, profile)
        result = dict(profile_word=self._read(Command.GET_PROFILE, 0))
        try:
            result.update(zip(("scan_ms", "out_ms", "gap_ms"),
                              (bcd_ms(self._read(Command.GET_PROFILE, page)) for page in range(1, 4))))
        except CommandError as exc:
            if exc.status not in (Status.BUSY, Status.UNSUPPORTED):
                raise
            result["timing_available"] = False
        else:
            result["timing_available"] = True
        return result

    @staticmethod
    def replay(path):
        parser = PacketParser()
        with Path(path).open("rb") as stream:
            while data := stream.read(4096):
                yield from parser.feed(data)
            yield from parser.feed(b"", idle=True)

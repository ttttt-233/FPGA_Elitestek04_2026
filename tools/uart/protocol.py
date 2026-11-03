"""Binary UART ABI. Pure Python: usable without Qt, pyserial or hardware."""
from enum import IntEnum
from functools import reduce
from operator import xor

SYNC = 0xA5
PROTOCOL_VERSION = 3
BAUD = 2000000
BUILD_ID = 0x0307
DIGIT_OTHER = 10


def digit_label(value: int) -> str:
    return str(value) if 0 <= value <= 9 else "OTHER" if value == DIGIT_OTHER else "UNKNOWN"


class Command(IntEnum):
    GET_VERSION = 0x00
    GET_STATUS = 0x01
    GET_CONTROL_STATE = 0x02
    GET_TARGET = 0x03
    SET_THRESHOLD = 0x10
    SET_AUTO = 0x11
    SET_ALGORITHM = 0x12
    SET_SHAPE = 0x13
    SET_FILTER = 0x14
    SET_MODE = 0x15
    SET_MENU = 0x16
    MEDIA = 0x20
    SET_OSD = 0x30
    SET_QUICK_HIDE = 0x31
    SET_OSD_MASK = 0x32
    SET_PROFILER = 0x33
    GET_PROFILE = 0x40
    SET_PROFILE = 0x41
    RAW_OPEN = 0x50
    RAW_CLOSE = 0x51
    RAW_BLOCK = 0x52
    RAW_STREAM = 0x53


class Status(IntEnum):
    OK = 0
    BAD_CHECKSUM = 1
    UNKNOWN_CMD = 2
    BAD_ARGUMENT = 3
    BUSY = 4
    UNSUPPORTED = 5
    TRANSFER_ERROR = 6


class TargetKind(IntEnum):
    NONE = 0
    RECT = 1
    CIRC = 2
    TRIANGLE = 3
    DIGIT = 4


class Algorithm(IntEnum):
    SOBEL = 0
    CANNY = 1


class ControlGroup(IntEnum):
    CAPTURE = 0
    PLAYBACK = 1
    DISPLAY = 2
    THRESHOLD = 3
    ALGORITHM = 4
    OSD = 5


class MediaAction(IntEnum):
    VIDEO_START = 0
    VIDEO_STOP = 1
    PRE = 2
    SNAPSHOT = 3
    PREVIOUS = 4
    NEXT = 5
    PLAY = 6
    PAUSE = 7


MODE_NAMES = ("Color", "Filtered gray", "Edge", "Color / edge", "Color + edge", "Raw gray / edge", "Gradient overlay")
OSD_MASK_NAMES = ("Status/context", "Guide", "Profiler/debug HUD")
STATUS_PAGE_NAMES = ("result_fps_x10", "failed_frames", "device_and_count", "uart_errors", "uart_commands", "media", "applied_algorithm_high", "applied_low")
DEVICE_FLAG_NAMES = ("ddr_pass", "ddr_done", "camera_configured", "camera_config_error", "camera_pixels_seen", "media_tick_overrun", "media_transfer_error", "media_source_overrun")


def checksum(data: bytes) -> int:
    return reduce(xor, data, 0)


def encode_command(command: int, value: int = 0, sequence: int = 0) -> bytes:
    if not 0 <= int(command) <= 255 or not 0 <= value <= 65535 or not 0 <= sequence <= 255:
        raise ValueError("command/sequence must fit u8; value must fit u16")
    data = bytes((SYNC, 1, sequence, int(command))) + value.to_bytes(2, "little")
    return data + bytes((checksum(data),))


def packet_length(kind: int) -> int:
    if kind == 0xA1:
        return 24
    if kind == 0xA0:
        return 100
    if kind == 0x90:
        return 6
    if kind == 0x91:
        return 7
    if kind == 0x81:
        return 9
    if kind & 0xF8 == 0x28:
        return 19
    if kind & 0xF8 == 0x20:
        return 18
    if kind & 0xF8 == 0x10:
        return 17
    return 0


def decode_packet(packet: bytes) -> dict:
    if len(packet) < 2 or packet[0] != SYNC or len(packet) != packet_length(packet[1]) or checksum(packet):
        raise ValueError("invalid UART packet")
    kind = packet[1]
    if kind == 0xA1:
        import binascii
        if binascii.crc_hqx(packet[2:21], 0xffff) != int.from_bytes(packet[21:23], "little"):
            raise ValueError("RAW block CRC mismatch")
        return dict(type="raw_block", sequence=packet[2], command=int(Command.RAW_BLOCK),
                    index=int.from_bytes(packet[3:5], "little"), data=packet[5:21])
    if kind == 0xA0:
        # Target records share the v2 layout. Decode all six before publishing.
        targets = []
        for slot in range(6):
            raw = bytes((SYNC, 0x20 | slot)) + packet[5 + slot * 15:20 + slot * 15]
            targets.append(decode_packet(raw + bytes((checksum(raw),))))
        return dict(type="frame", version=3, frame_id=int.from_bytes(packet[2:4], "little"),
                    target_count=packet[4], targets=targets,
                    fps_x10=int.from_bytes(packet[95:97], "little"),
                    error_count=int.from_bytes(packet[97:99], "little"))
    if kind == 0x90:
        return dict(type="ack", sequence=packet[2], command=packet[3], status=packet[4],
                    status_name=Status(packet[4]).name if packet[4] in Status._value2member_map_ else "UNKNOWN_STATUS")
    if kind == 0x91:
        return dict(type="response", sequence=packet[2], command=packet[3], value=int.from_bytes(packet[4:6], "little"))
    if kind == 0x81:
        return dict(type="status", version=packet[7], fps_x10=int.from_bytes(packet[2:4], "little"),
                    error_count=int.from_bytes(packet[4:6], "little"), target_count=packet[6])
    query = kind & 0xF8 == 0x28
    v2 = kind & 0xF8 == 0x20 or query
    offset = 1 if query else 0
    flags = packet[2 + offset]
    base = (7 if v2 else 6) + offset
    box = int.from_bytes(packet[base:base + 6], "little")
    shape = flags & 7
    result = dict(type="target", version=3 if query else 2 if v2 else 1, slot=kind & 7,
                  alive=bool(flags & 0x80), kind=shape,
                  kind_name=TargetKind(shape).name if shape in TargetKind._value2member_map_ else "UNKNOWN",
                  quality=(flags >> 3) & 3, id=packet[3 + offset], digit=packet[4 + offset], age=packet[5 + offset],
                  lost_age=packet[6 + offset] if v2 else (flags >> 5) & 3,
                  bbox=dict(left=(box >> 36) & 4095, top=(box >> 24) & 4095, right=(box >> 12) & 4095, bottom=box & 4095),
                  vx=int.from_bytes(packet[base + 6:base + 8], "little", signed=True),
                  vy=int.from_bytes(packet[base + 8:base + 10], "little", signed=True))
    if query:
        result["sequence"] = packet[2]
    return result


class PacketParser:
    """Incremental framing, bounded storage and checksum-based resynchronization."""
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes, *, idle=False) -> list[dict]:
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            if self.buffer[0] != SYNC:
                del self.buffer[0]
                continue
            length = packet_length(self.buffer[1])
            if not length:
                del self.buffer[0]
                self.errors += 1
                continue
            if len(self.buffer) < length:
                # A truncated packet may precede a complete, short ACK. Recover
                # only at a recognized header whose entire frame checks out.
                # A fragmented full frame may contain a valid short packet in
                # its target payload. Only scan inside it after a real idle gap.
                if length in (24, 100) and not idle:
                    break
                found = False
                for pos in range(1, len(self.buffer) - 1):
                    n = packet_length(self.buffer[pos + 1]) if self.buffer[pos] == SYNC else 0
                    if n and len(self.buffer) >= pos + n and checksum(self.buffer[pos:pos + n]) == 0:
                        del self.buffer[:pos]
                        self.errors += 1
                        found = True
                        break
                if found:
                    continue
                break
            packet = bytes(self.buffer[:length])
            if checksum(packet):
                del self.buffer[0]
                self.errors += 1
                continue
            try:
                record = decode_packet(packet)
            except ValueError:
                del self.buffer[:length]
                self.errors += 1
                continue
            result.append(record)
            del self.buffer[:length]
        return result


def decode_packets(data: bytes) -> list[dict]:
    return PacketParser().feed(data)


def decode_control(word: int, threshold: int) -> dict:
    return dict(group=(word >> 13) & 7, item=(word >> 11) & 3, mode=(word >> 8) & 7,
                algorithm=word & 1, auto=bool(word & 2), shape=bool(word & 4), osd=bool(word & 8),
                quick_hide=bool(word & 16), osd_mask=(word >> 5) & 7, threshold=threshold)


def decode_media(word: int) -> dict:
    return dict(selected_valid=bool(word & 0x8000), playing=bool(word & 0x4000),
                video=bool(word & 0x2000), recording=bool(word & 0x1000),
                count=(word >> 8) & 15, index=(word >> 4) & 15, photo_count=word & 15)


def bcd_ms(word: int) -> int:
    return (word >> 8 & 15) * 100 + (word >> 4 & 15) * 10 + (word & 15)

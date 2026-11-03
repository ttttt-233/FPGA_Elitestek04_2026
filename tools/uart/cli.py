"""Scriptable control: python tools/uart/cli.py --port COM11 status."""
import argparse
import json
import sys
import time
try:
    from .client import FPGAClient, FPGAError
    from .protocol import Algorithm, ControlGroup, MODE_NAMES, BAUD
except ImportError:
    from client import FPGAClient, FPGAError
    from protocol import Algorithm, ControlGroup, MODE_NAMES, BAUD


def integer(value):
    return int(value, 0)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--timeout", type=float, default=1.0)
    sub = p.add_subparsers(dest="action", required=True)
    for name in ("status", "version", "targets", "snapshot", "pre", "media-prev", "media-next", "play", "pause"):
        sub.add_parser(name)
    watch = sub.add_parser("watch")
    watch.add_argument("--seconds", type=float, default=0, help="0: until Ctrl+C; compatible with old telemetry")
    for name in ("auto", "shape", "osd", "quick-hide", "profiler"):
        s = sub.add_parser(name);s.add_argument("value", choices=("on", "off"))
    s = sub.add_parser("threshold");s.add_argument("value", type=integer)
    s = sub.add_parser("algorithm");s.add_argument("value", choices=("sobel", "canny"))
    s = sub.add_parser("mode");s.add_argument("value", type=int, choices=range(7), help=str(dict(enumerate(MODE_NAMES))))
    s = sub.add_parser("menu");s.add_argument("group", choices=[g.name.lower() for g in ControlGroup]);s.add_argument("--item", type=int, choices=range(4), default=0)
    s = sub.add_parser("mask");s.add_argument("value", type=integer)
    s = sub.add_parser("video");s.add_argument("value", choices=("start", "stop"))
    s = sub.add_parser("profile");s.add_argument("--set", type=integer, dest="profile")
    s = sub.add_parser('save-photo');s.add_argument('path', help='DNG filename; two JPEG names are derived')
    a = p.parse_args(argv)
    client = FPGAClient(a.port, a.baud, a.timeout)
    try:
        client.connect(handshake=a.action != "watch")
        if a.action == "watch":
            end = time.monotonic() + a.seconds if a.seconds > 0 else float("inf")
            while time.monotonic() < end:
                for record in client.poll(wait=True):
                    print(json.dumps(record), flush=True)
            return 0
        commands = {
            "status": client.get_status, "version": client.get_version, "targets": client.get_targets,
            "snapshot": client.snapshot, "pre": client.pre_capture,
            "media-prev": client.media_prev, "media-next": client.media_next,
            "play": client.play, "pause": client.pause,
        }
        setters = {"auto": client.set_auto, "shape": client.set_shape_enable, "osd": client.set_osd_enable,
                   "quick-hide": client.set_quick_hide, "profiler": client.set_profiler}
        if a.action in commands:
            result = commands[a.action]()
        elif a.action in setters:
            result = setters[a.action](a.value == "on")
        elif a.action == 'save-photo': result = client.save_playback_photo(a.path)
        elif a.action == "threshold": result = client.set_threshold(a.value)
        elif a.action == "algorithm": result = client.set_algorithm(Algorithm[a.value.upper()])
        elif a.action == "mode": result = client.set_mode(a.value)
        elif a.action == "menu": result = client.set_menu(ControlGroup[a.group.upper()], a.item)
        elif a.action == "mask": result = client.set_osd_mask(a.value)
        elif a.action == "video": result = client.video_start() if a.value == "start" else client.video_stop()
        else: result = client.profile_request(a.profile)
        print(json.dumps(result, indent=2))
        return 0
    except KeyboardInterrupt:
        return 0
    except (FPGAError, OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        client.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

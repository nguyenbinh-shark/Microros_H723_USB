"""Optional command-line diagnostics; normal use is the desktop shortcut."""
import argparse
import json

from rpbot_console import DEFAULTS, STATE, check_connection, remote, validate_settings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--install-autostart", action="store_true")
    parser.add_argument("--save-map", metavar="NAME")
    parser.add_argument("--check-reset", action="store_true", help="Check reset service without clearing the map")
    args = parser.parse_args()
    STATE.mkdir(parents=True, exist_ok=True)
    settings = dict(DEFAULTS)
    config = STATE / "settings.json"
    if config.is_file():
        settings.update(json.loads(config.read_text(encoding="utf-8")))
    validate_settings(settings)
    if args.install_autostart:
        print(remote(settings, "install-autostart"))
    report = check_connection(settings)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    (STATE / "last-diagnostics.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if args.save_map:
        if not report["session"] or not report["data"]["mapping_ready"]:
            raise RuntimeError("No healthy mapping session; map not saved")
        print(remote(settings, "save", "--name", args.save_map,
                     "--domain", str(report["session"]["domain"])))
    if args.check_reset:
        if not report["session"]:
            raise RuntimeError("No confirmed real mapping session")
        print(remote(settings, "reset-info", "--domain", str(report["session"]["domain"]),
                     "--pid", str(report["session"]["pid"])))
    if not report["data"] or not report["data"]["mapping_ready"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

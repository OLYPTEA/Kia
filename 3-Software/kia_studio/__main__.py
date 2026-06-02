"""GUI entry point. `python -m kia_studio`.

M1 ships the protocol + domain core only; the Qt window lands in M2.
"""
import sys


def main() -> int:
    try:
        from .ui.main_window import run        # noqa: F401  (added in M2)
    except ImportError:
        print("Kia Studio GUI not built yet (arrives in M2).", file=sys.stderr)
        print("M1 delivers the offline core — run the tests with:  pytest", file=sys.stderr)
        return 1
    return run()


if __name__ == "__main__":
    raise SystemExit(main())

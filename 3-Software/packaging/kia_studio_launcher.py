"""Frozen-app entry point — launches the Kia Studio GUI (used by the PyInstaller spec)."""
import sys

from kia_studio.ui.main_window import run

if __name__ == "__main__":
    sys.exit(run())

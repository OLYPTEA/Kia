"""Build the Kia Studio Windows app via PyInstaller (onedir by default).

    python packaging/build.py            # build with the spec
Output: software/dist/KiaStudio/KiaStudio.exe  (run the .exe).
Edit ONEFILE in packaging/KiaStudio.spec for a single-file build.
"""
import os
import subprocess
import sys

SOFTWARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    os.chdir(SOFTWARE)
    cmd = [sys.executable, "-m", "PyInstaller", "packaging/KiaStudio.spec",
           "--noconfirm", "--clean"]
    print(">", " ".join(cmd))
    raise SystemExit(subprocess.call(cmd))


if __name__ == "__main__":
    main()

"""Compile the Windows installer with Inno Setup (ISCC).

Prereq: run packaging/build.py first (creates dist/KiaStudio/).
Install Inno Setup if missing:  winget install JRSoftware.InnoSetup
Then: python packaging/build_installer.py  ->  packaging/installer_out/KiaStudio-Setup-*.exe
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _find_iscc():
    exe = shutil.which("iscc") or shutil.which("ISCC")
    if exe:
        return exe
    bases = [os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
             os.environ.get("ProgramFiles", r"C:\Program Files"),
             os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs")]  # winget user install
    for base in bases:
        for name in ("Inno Setup 6", "Inno Setup 5"):
            cand = os.path.join(base, name, "ISCC.exe")
            if os.path.exists(cand):
                return cand
    return None


def main():
    if not os.path.isdir(os.path.join(os.path.dirname(HERE), "dist", "KiaStudio")):
        sys.exit("dist/KiaStudio not found — run `python packaging/build.py` first.")
    iscc = _find_iscc()
    if not iscc:
        sys.exit("Inno Setup (ISCC) not found. Install: winget install JRSoftware.InnoSetup")
    raise SystemExit(subprocess.call([iscc, os.path.join(HERE, "installer.iss")]))


if __name__ == "__main__":
    main()

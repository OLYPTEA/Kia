# PyInstaller spec for Kia Studio — onedir Windows build (robust for Qt + moderngl/OpenGL).
# Build:  pyinstaller packaging/KiaStudio.spec --noconfirm     (run from software/)
# Onefile: set ONEFILE = True below (slower startup, single .exe).
import os
from PyInstaller.utils.hooks import collect_data_files, collect_submodules, collect_dynamic_libs

ONEFILE = False
APP_NAME = "KiaStudio"
SPEC_DIR = os.path.abspath(SPECPATH)          # software/packaging
ROOT = os.path.dirname(SPEC_DIR)              # software/
ICON = os.path.join(SPEC_DIR, "kia.ico")
icon = ICON if os.path.exists(ICON) else None

# bundle package data: STL meshes + calibration/geometry JSON under kia_studio/resources/**
datas = collect_data_files("kia_studio", includes=["resources/**/*"])

# native/dynamic deps that PyInstaller can miss
hiddenimports = (
    collect_submodules("moderngl")
    + collect_submodules("glcontext")
    + ["OpenGL", "OpenGL.platform.win32", "OpenGL.arrays.ctypesarrays",
       "OpenGL.arrays.numpymodule", "PySide6.QtOpenGLWidgets", "pyqtgraph.opengl"]
)
binaries = collect_dynamic_libs("glcontext")

a = Analysis(
    [os.path.join(SPEC_DIR, "kia_studio_launcher.py")],
    pathex=[ROOT],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    excludes=["tkinter", "matplotlib", "pytest", "PySide6.Qt3DCore",
              "PySide6.Qt3DRender", "PySide6.Qt3DExtras", "PySide6.QtWebEngineCore"],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe_kwargs = dict(name=APP_NAME, console=False, icon=icon)
if ONEFILE:
    exe = EXE(pyz, a.scripts, a.binaries, a.datas, [], **exe_kwargs)
else:
    exe = EXE(pyz, a.scripts, [], exclude_binaries=True, **exe_kwargs)
    coll = COLLECT(exe, a.binaries, a.datas, name=APP_NAME)

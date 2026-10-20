"""Paths to the PCB toolchain (KiCad 9, Java 25, Freerouting 2.4.1): per-OS defaults (macOS, Windows, Linux), each overridable by an
environment variable: KICAD_CLI, KICAD_PYTHON, KICAD9_SYMBOL_DIR, KICAD9_FOOTPRINT_DIR, KICAD9_3DMODEL_DIR, JAVA (or JAVA_HOME),
FREEROUTING_JAR."""
import glob, os, platform, shutil
from pathlib import Path
PCB_DIR = Path(__file__).parent
SYSTEM = platform.system()
env = os.environ.get
if SYSTEM == 'Darwin':
    APP = Path('/Applications/KiCad/KiCad.app')
    _cli, _python, _share = APP / 'Contents/MacOS/kicad-cli', APP / 'Contents/Frameworks/Python.framework/Versions/Current/bin/python3', APP / 'Contents/SharedSupport'
elif SYSTEM == 'Windows':   # all-users install, else per-user install
    _roots = [Path(env('ProgramFiles', r'C:\Program Files')) / 'KiCad/9.0', Path(env('LOCALAPPDATA', '~')) / 'Programs/KiCad/9.0']
    _root = next((r for r in _roots if r.is_dir()), _roots[0])
    _cli, _python, _share = _root / 'bin/kicad-cli.exe', _root / 'bin/python.exe', _root / 'share/kicad'
else:   # Linux: the distribution's KiCad 9 packages put pcbnew into the system Python
    _cli, _python, _share = Path(shutil.which('kicad-cli') or '/usr/bin/kicad-cli'), Path('/usr/bin/python3'), Path('/usr/share/kicad')
KICAD_CLI = env('KICAD_CLI', str(_cli))
KICAD_PYTHON = env('KICAD_PYTHON', str(_python))
SYMBOL_DIR = Path(env('KICAD9_SYMBOL_DIR', _share / 'symbols'))
FOOTPRINT_DIR = Path(env('KICAD9_FOOTPRINT_DIR', _share / 'footprints'))
MODEL_DIR = Path(env('KICAD9_3DMODEL_DIR', _share / '3dmodels'))


def _java():
    """ruling R14: Freerouting 2.4.1 is built for Java 25 (class file 69). macOS: Homebrew's openjdk@25. Elsewhere the first of
    JAVA_HOME, PATH, then the newest Temurin JDK 25 under ~/tools or Program Files."""
    if SYSTEM == 'Darwin': return '/opt/homebrew/opt/openjdk@25/bin/java'
    found = [str(Path(env('JAVA_HOME')) / 'bin/java')] if env('JAVA_HOME') else []
    found += [shutil.which('java')]
    found += sorted(glob.glob(str(Path.home() / 'tools/jdk-25*/bin/java.exe')) + glob.glob(r'C:\Program Files\Eclipse Adoptium\jdk-25*\bin\java.exe'), reverse=True)
    return next((j for j in found if j and shutil.which(j)), 'java')
JAVA = env('JAVA') or _java()
FREEROUTING = env('FREEROUTING_JAR', str(PCB_DIR / 'tools' / 'freerouting.jar'))

"""Paths to the PCB toolchain (KiCad 9, Java 25, Freerouting 2.4.1): per-OS defaults (macOS, Windows, Linux), each overridable by an
environment variable: KICAD_CLI, KICAD_PYTHON, KICAD9_SYMBOL_DIR, KICAD9_FOOTPRINT_DIR, KICAD9_3DMODEL_DIR, JAVA (else JAVA_HOME,
see _java), FREEROUTING_JAR."""
import glob, os, platform, re, shutil
from pathlib import Path
PCB_DIR = Path(__file__).parent
SYSTEM = platform.system()
env = os.environ.get
if SYSTEM == 'Darwin':
    APP = Path('/Applications/KiCad/KiCad.app')
    _cli, _python, _share = APP / 'Contents/MacOS/kicad-cli', APP / 'Contents/Frameworks/Python.framework/Versions/Current/bin/python3', APP / 'Contents/SharedSupport'
elif SYSTEM == 'Windows':   # all-users install, else per-user install
    _roots = [Path(env('ProgramFiles', r'C:\Program Files')) / 'KiCad/9.0', Path(env('LOCALAPPDATA') or Path.home() / 'AppData/Local') / 'Programs/KiCad/9.0']
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
    """ruling R14: Freerouting 2.4.1 is built for Java 25 (class file 69). The first file of JAVA_HOME's java, the newest pinned JDK 25
    (macOS: Homebrew's openjdk@25, Apple silicon or Intel; Windows: Temurin under ~/tools or Program Files; Linux: /usr/lib/jvm), then
    PATH's java. Path.is_file, not shutil.which: under KiCad's Python 3.11 on Windows which() rejects a full path without .exe."""
    exe = 'java.exe' if SYSTEM == 'Windows' else 'java'
    def newest(*pats):   # version order, not string order: jdk-25.0.10 before jdk-25.0.4; JDK 25 only (LIO-236: /usr/lib/jvm/*25* also
        # matches java-21-openjdk-21.0.25), the +build suffix left out (jdk-25.0.4.1+1)
        key = lambda j: [int(n) for n in re.findall(r'\d+', Path(j).parent.parent.name.split('+')[0])]
        return sorted((j for pat in pats for j in glob.glob(pat) if key(j)[:1] == [25]), key=key, reverse=True)
    found = [str(Path(env('JAVA_HOME')) / 'bin' / exe)] if env('JAVA_HOME') else []
    if SYSTEM == 'Darwin': found += newest('/opt/homebrew/opt/openjdk@25/bin/java', '/usr/local/opt/openjdk@25/bin/java')
    elif SYSTEM == 'Windows':
        found += newest(str(Path.home() / 'tools/jdk-25*/bin/java.exe'), str(Path(env('ProgramFiles', r'C:\Program Files')) / 'Eclipse Adoptium/jdk-25*/bin/java.exe'))
    else: found += newest('/usr/lib/jvm/*25*/bin/java')
    return next((j for j in found if Path(j).is_file()), shutil.which('java') or 'java')
JAVA = env('JAVA') or _java()
FREEROUTING = env('FREEROUTING_JAR', str(PCB_DIR / 'tools' / 'freerouting.jar'))
FREEROUTING_SHA256 = '251101c3eeac22d7e7dfcf6796603279e5d1000283eb82d8f093780f7afc6aa9'   # freerouting-2.4.1.jar (test_tools checks it)

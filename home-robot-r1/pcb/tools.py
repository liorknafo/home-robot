"""Paths to the PCB toolchain (KiCad 9, Java 25, Freerouting 2.4.1)."""
from pathlib import Path
APP = Path('/Applications/KiCad/KiCad.app')
KICAD_CLI = str(APP / 'Contents/MacOS/kicad-cli')
KICAD_PYTHON = str(APP / 'Contents/Frameworks/Python.framework/Versions/Current/bin/python3')
SYMBOL_DIR = APP / 'Contents/SharedSupport/symbols'
FOOTPRINT_DIR = APP / 'Contents/SharedSupport/footprints'
JAVA = '/opt/homebrew/opt/openjdk@25/bin/java'      # ruling R14: Freerouting 2.4.1 is built for Java 25 (class file 69)
FREEROUTING = str(Path(__file__).parent / 'tools' / 'freerouting.jar')
PCB_DIR = Path(__file__).parent

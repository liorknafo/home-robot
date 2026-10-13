import subprocess, unittest
from pathlib import Path
from pcb import tools
from pcb.tests import left_out

class ToolTests(unittest.TestCase):
    def test_kicad_cli_is_version_9(self):
        out = subprocess.run([tools.KICAD_CLI, 'version'], capture_output=True, text=True).stdout.strip()
        self.assertTrue(out.startswith('9.'), out)
    def test_kicad_python_imports_pcbnew(self):
        r = subprocess.run([tools.KICAD_PYTHON, '-c', 'import pcbnew;print(pcbnew.Version())'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr[-400:]); self.assertTrue(r.stdout.strip().startswith('9.'), r.stdout)
    def test_libraries_present(self):
        self.assertTrue((tools.SYMBOL_DIR / 'Device.kicad_sym').is_file())
        self.assertTrue((tools.FOOTPRINT_DIR / 'Connector_JST.pretty').is_dir())
    def test_java_25_and_freerouting(self):
        """Freerouting 2.4.1 is built for Java 25 (class file 69): check the JVM version AND actually start the jar headless, so a
        JVM / jar mismatch (UnsupportedClassVersionError) can never slip through again (ruling R14)."""
        r = subprocess.run([tools.JAVA, '-version'], capture_output=True, text=True)
        self.assertIn('version "25', r.stderr)
        if left_out.missing('pcb/tools/freerouting.jar'): self.skipTest(left_out.reason('pcb/tools/freerouting.jar'))   # the JVM check above still ran
        self.assertGreater(Path(tools.FREEROUTING).stat().st_size, 1_000_000)
        r = subprocess.run([tools.JAVA, '-Djava.awt.headless=true', '-jar', tools.FREEROUTING, '--help'],
                           capture_output=True, text=True, timeout=120)
        out = r.stdout + r.stderr
        self.assertNotIn('UnsupportedClassVersionError', out)
        self.assertEqual(r.returncode, 0, out[-600:])
        self.assertIn('Freerouting v2.', out)
        self.assertIn('-de <design.dsn>', out)

if __name__ == '__main__':
    unittest.main()

import json, subprocess, tempfile, unittest
from pathlib import Path
from pcb import board_spec as B, tools
from pcb.make_schematic import read_netlist, export_netlist, main as make_schematic_main
D = Path(B.__file__).parent
SCH, PRO = D / 'r1_power_io.kicad_sch', D / 'r1_power_io.kicad_pro'

def unconnected_pins(nets):
    """KiCad names a net 'unconnected-(<ref>-<pin name>-Pad<n>)' for each single pin with a no-connect flag."""
    return {p for n, pins in nets.items() if n.startswith('unconnected-(') for p in pins}

class SchematicTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        export_netlist(SCH, Path(cls.tmp.name) / 'r1_power_io.net')
        cls.nets = read_netlist(Path(cls.tmp.name) / 'r1_power_io.net')

    def test_erc_has_no_errors(self):
        out = Path(self.tmp.name) / 'erc.json'
        subprocess.run([tools.KICAD_CLI, 'sch', 'erc', '--format', 'json', '--severity-error', '-o', str(out), str(SCH)],
                       check=True, capture_output=True)
        rep = json.loads(out.read_text())
        errs = [v for s in rep['sheets'] for v in s['violations']]
        self.assertEqual(errs, [], json.dumps(errs[:5], indent=1))

    def test_connectivity_erc_rules_are_never_downgraded(self):
        sev = json.loads(PRO.read_text())['erc']['rule_severities']
        for rule in ('pin_not_connected', 'label_dangling', 'wire_dangling', 'pin_not_driven', 'power_pin_not_driven'):
            self.assertEqual(sev.get(rule, 'error'), 'error', rule)

    def test_netlist_equals_board_spec(self):
        got = {n: pins for n, pins in self.nets.items() if not n.startswith('unconnected-(')}
        want = {n: set(p) for n, p in B.NETS.items()}
        self.assertEqual(got, want)

    def test_no_connect_pins_equal_board_spec_nc(self):
        self.assertEqual(unconnected_pins(self.nets), set(B.NC))

    def test_every_part_is_in_the_netlist(self):
        """Every part with pins is on some net; the only pinless parts are the clamp holes H5 / H6 (ruling PB3: Ø3.0 NPTH, no net)."""
        pinless = {r for r, p in B.PARTS.items() if not B.symbol_pin_numbers(p['sym'])}
        self.assertEqual(pinless, {'H5', 'H6'})
        self.assertEqual({r for pins in self.nets.values() for r, p in pins}, set(B.PARTS) - pinless)

    def test_project_netclasses_match_board_spec(self):
        ns = json.loads(PRO.read_text())['net_settings']
        classes = {c['name']: c for c in ns['classes']}
        self.assertEqual(set(classes), {'Default', *B.NETCLASSES})
        self.assertEqual((classes['Default']['track_width'], classes['Default']['clearance']), (0.15, 0.15))   # PB17
        for name, spec in B.NETCLASSES.items():
            self.assertEqual((classes[name]['track_width'], classes[name]['clearance']), (spec['width'], spec['clearance']), name)
            got = sorted(p['pattern'] for p in ns['netclass_patterns'] if p['netclass'] == name)
            self.assertEqual(got, sorted(spec['nets']), name)
        self.assertEqual(len(ns['netclass_patterns']), sum(len(s['nets']) for s in B.NETCLASSES.values()))

    def test_generated_files_are_current(self):
        with tempfile.TemporaryDirectory() as t:
            make_schematic_main(Path(t))
            for f in (SCH.name, PRO.name, 'sym-lib-table', 'r1_power_io.kicad_dru'):
                self.assertEqual((Path(t) / f).read_text(), (D / f).read_text(), f'{f} is stale: run pcb/make_schematic.py')

if __name__ == '__main__':
    unittest.main()

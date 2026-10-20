import json, subprocess, tempfile, unittest
from pathlib import Path
from pcb import board_spec as B, tools
from pcb.make_schematic import read_netlist, export_netlist, project, sym_lib_table, fp_lib_table, main as make_schematic_main
from pcb.tests import left_out
D = Path(B.__file__).parent
SCH, PRO = D / 'r1_power_io.kicad_sch', D / 'r1_power_io.kicad_pro'

def unconnected_pins(nets):
    """KiCad names a net 'unconnected-(<ref>-<pin name>-Pad<n>)' for each single pin with a no-connect flag."""
    return {p for n, pins in nets.items() if n.startswith('unconnected-(') for p in pins}

class SchematicTests(unittest.TestCase):
    """The checks that need no KiCad."""
    def test_connectivity_erc_rules_are_never_downgraded(self):
        sev = json.loads(PRO.read_text(encoding='utf-8'))['erc']['rule_severities']
        for rule in ('pin_not_connected', 'label_dangling', 'wire_dangling', 'pin_not_driven', 'power_pin_not_driven'):
            self.assertEqual(sev.get(rule, 'error'), 'error', rule)

    def test_project_netclasses_match_board_spec(self):
        ns = json.loads(PRO.read_text(encoding='utf-8'))['net_settings']
        classes = {c['name']: c for c in ns['classes']}
        self.assertEqual(set(classes), {'Default', *B.NETCLASSES})
        self.assertEqual((classes['Default']['track_width'], classes['Default']['clearance']), (0.15, 0.15))   # PB17
        for name, spec in B.NETCLASSES.items():
            self.assertEqual((classes[name]['track_width'], classes[name]['clearance']), (spec['width'], spec['clearance']), name)
            got = sorted(p['pattern'] for p in ns['netclass_patterns'] if p['netclass'] == name)
            self.assertEqual(got, sorted(spec['nets']), name)
        self.assertEqual(len(ns['netclass_patterns']), sum(len(s['nets']) for s in B.NETCLASSES.values()))

    def test_project_rules_and_lib_tables_are_current(self):
        for f, text in ((PRO.name, json.dumps(project(), indent=2) + '\n'), ('sym-lib-table', sym_lib_table()), ('fp-lib-table', fp_lib_table()), ('r1_power_io.kicad_dru', B.DRC_RULES)):
            self.assertEqual((D / f).read_text(encoding='utf-8'), text, f'{f} is stale: run pcb/make_schematic.py')


@left_out.needs_kicad()
class KiCadSchematicTests(unittest.TestCase):
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
        rep = json.loads(out.read_text(encoding='utf-8'))
        errs = [v for s in rep['sheets'] for v in s['violations']]
        self.assertEqual(errs, [], json.dumps(errs[:5], indent=1))

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

    def test_committed_netlist_is_current(self):
        """r1_power_io.net (build_pcb reads its no-connect nets) has the fresh export's nets; its text differs (absolute path, date)."""
        self.assertEqual(read_netlist(D / 'r1_power_io.net'), self.nets, 'r1_power_io.net is stale: run pcb/make_schematic.py')

    def test_schematic_is_current(self):
        """The schematic embeds KiCad's library symbols (SchematicTests checks the other generated files)."""
        with tempfile.TemporaryDirectory() as t:
            make_schematic_main(Path(t))
            self.assertEqual((Path(t) / SCH.name).read_text(encoding='utf-8'), SCH.read_text(encoding='utf-8'), f'{SCH.name} is stale: run pcb/make_schematic.py')

if __name__ == '__main__':
    unittest.main()

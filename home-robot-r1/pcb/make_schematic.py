"""Net-label schematic from board_spec: every symbol on a grid, a global label on every netted pin, a no-connect flag on every NC pin.

Schematic parity (bay B7): global labels, so the netlist's net names are board_spec's ('VMOT', not the local labels' '/VMOT'); the test
pads and holes are in_bom no, as their footprints (board_spec.in_bom); build_pcb gives every footprint its library nickname and every
no-connect pin the netlist's 'unconnected-(...)' net. kicad-cli pcb drc --schematic-parity then reports nothing (ktest_pcb).

Writes r1_power_io.kicad_sch (KiCad 9 format), r1_power_io.kicad_pro (net classes from board_spec.NETCLASSES + ERC severities),
r1_power_io.kicad_dru (board_spec.DRC_RULES) and
r1_power_io.net (KiCad's own netlist export of the schematic). Deterministic: UUIDs are derived from the part/pin, so a rerun is byte-identical."""
import copy, json, subprocess, sys, uuid
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, tools
from pcb.sexpr import Q, parse, dump, find
D = Path(__file__).parent
PROJECT = 'r1_power_io'
SCH_VERSION = '20250114'          # KiCad 9.0 schematic file format (matches the stock templates shipped with 9.0.9)
_NS = uuid.UUID('5b1c9e0e-6a58-4a39-9d7b-0c2b2f1a7a11')
def U(*key): return Q(str(uuid.uuid5(_NS, '/'.join(map(str, key)))))

# ERC severity changes (everything else keeps KiCad's default). None at present: the schematic passes with the stock rules.
ERC_SEVERITY = {}
# Board DRC severity changes (everything else keeps KiCad's default). B7: lib_footprint_mismatch is off - build_pcb loads every footprint
# from its library at build time (so it cannot drift from it) and then changes it on purpose (module outlines on F.Fab, J1's +/- marks,
# R13's inherited zone connections, U3P.2's spoke angle), which that check reports as a mismatch on exactly 10 footprints: F1, F2, J1, U1A,
# U1B, U2, U3L, U3P, U4L, U4P. B7 fix 1: per-item exclusions for those 10 (board.design_settings.drc_exclusions in this file, keys
# 'lib_footprint_mismatch|x|y|footprint uuid|aux uuid', the footprint UUIDs are seeded) were tried and are not usable: kicad-cli 9.0.9 does not
# apply them (the same 10 warnings, and a track_width exclusion in a scratch copy was ignored too), and the CLI is what the tests and any
# batch DRC run. So the check stays off project-wide, and ktest_pcb.test_lib_footprint_mismatch_is_exactly_the_ten_modified_footprints runs
# DRC with it on (a temp copy) and fails if the flagged set is not those 10, so a new mismatch cannot hide behind the ignore.
DRC_SEVERITY = {'lib_footprint_mismatch': 'ignore'}

def lib_symbol(lib_id):
    """The library symbol as embedded in a schematic's lib_symbols (name = lib_id); a derived ('extends') symbol is flattened."""
    lib, name = lib_id.split(':')
    syms = B._symbols(lib)
    s = copy.deepcopy(syms[name])
    ext = find(s, 'extends')
    if ext:   # derived symbol: parent's drawing and pins, child's properties
        parent = copy.deepcopy(syms[ext[0][1]])
        body = [c for c in parent[2:] if not (isinstance(c, list) and c[0] == 'property')]
        for c in body:
            if isinstance(c, list) and c[0] == 'symbol': c[1] = Q(c[1].replace(ext[0][1], name, 1))
        s = ['symbol', Q(name)] + [c for c in s[2:] if not (isinstance(c, list) and c[0] == 'extends')] + body
    s[1] = Q(lib_id)
    return s
def pins_of(sym):   # pin number -> (x, y, angle) of the electrical end in symbol coordinates (y up); angle points from the end into the body
    return {str(find(p, 'number')[0][1]): (float(find(p, 'at')[0][1]), float(find(p, 'at')[0][2]), int(float(find(p, 'at')[0][3])))
            for sub in find(sym, 'symbol') for p in find(sub, 'pin')}
def num(v):
    return f'{v:.4f}'.rstrip('0').rstrip('.')
def pretty(x, d=0):
    """KiCad-style layout: leading atoms on the head line, each child list on its own tab-indented line."""
    if not isinstance(x, list): return dump(x)
    i = next((k for k, c in enumerate(x) if isinstance(c, list)), len(x))
    head = '(' + ' '.join(dump(c) for c in x[:i])
    if i == len(x): return head + ')'
    pad = '\t' * (d + 1)
    return head + ''.join('\n' + pad + pretty(c, d + 1) for c in x[i:]) + '\n' + '\t' * d + ')'

def read_netlist(path):
    """KiCad S-expression netlist -> {net name: {(ref, pin)}}. A local label in the root sheet exports as '/NAME'; the leading '/' is stripped."""
    tree = parse(Path(path).read_text())[0]; out = {}
    for net in find(find(tree, 'nets')[0], 'net'):
        name = str(find(net, 'name')[0][1])
        out[name[1:] if name.startswith('/') else name] = {(str(find(n, 'ref')[0][1]), str(find(n, 'pin')[0][1])) for n in find(net, 'node')}
    return out
def export_netlist(sch, out):
    subprocess.run([tools.KICAD_CLI, 'sch', 'export', 'netlist', '--format', 'kicadsexpr', '-o', str(out), str(sch)], check=True, capture_output=True)

def instance(ref, part, sym, X, Y, root):
    lib_id = part['sym']
    props = {str(p[1]): p for p in find(sym, 'property')}
    def prop(k, v):   # lib property placement (offset from the symbol origin, y flipped to the page) and font
        p = props[k]; at = find(p, 'at')[0]
        return ['property', Q(k), Q(v), ['at', num(X + float(at[1])), num(Y - float(at[2])), at[3]], copy.deepcopy(find(p, 'effects')[0])]
    in_bom = find(sym, 'in_bom')[0][1] if B.in_bom(ref) else 'no'        # B7: a bare test pad is no BOM part (its footprint says so)
    inst = ['symbol', ['lib_id', Q(lib_id)], ['at', num(X), num(Y), '0'], ['unit', '1'],
            ['exclude_from_sim', find(sym, 'exclude_from_sim')[0][1]], ['in_bom', in_bom],
            ['on_board', find(sym, 'on_board')[0][1]], ['dnp', 'yes' if part.get('dnp') else 'no'], ['uuid', U(ref)],
            prop('Reference', ref), prop('Value', part['value']), prop('Footprint', part['fp']),
            prop('Datasheet', str(props['Datasheet'][2])), prop('Description', str(props['Description'][2]))]
    inst += [['pin', Q(n), ['uuid', U(ref, n)]] for n in pins_of(sym)]
    inst += [['instances', ['project', Q(PROJECT), ['path', Q('/' + root), ['reference', Q(ref)], ['unit', '1']]]]]
    return inst

def label(net, x, y, pin_angle):
    """A global label (B7, schematic parity): its net is named `net`, as on the board, where a local label's would be '/net'."""
    a = (pin_angle + 180) % 360            # text runs away from the symbol body
    return ['global_label', Q(net), ['shape', 'passive'], ['at', num(x), num(y), str(a)], ['fields_autoplaced', 'yes'],
            ['effects', ['font', ['size', '1.27', '1.27']], ['justify', 'left' if a in (0, 90) else 'right']]]

def schematic():
    root = U('root'); pin_net = {p: n for n, pins in B.NETS.items() for p in pins}
    libsyms, items, used = [], [], set()
    for k, (ref, part) in enumerate(sorted(B.PARTS.items())):
        sym = lib_symbol(part['sym'])
        if part['sym'] not in used: libsyms.append(sym); used.add(part['sym'])
        X, Y = 25.4 + 50.8 * (k % 8), 25.4 + 63.5 * (k // 8)           # 1.27-grid aligned
        items.append(instance(ref, part, sym, X, Y, str(root)))
        for n, (px, py, ang) in pins_of(sym).items():
            x, y = X + px, Y - py
            if (ref, n) in pin_net:
                items.append(label(pin_net[(ref, n)], x, y, ang) + [['uuid', U(ref, n, 'label')]])
            else:
                items.append(['no_connect', ['at', num(x), num(y)], ['uuid', U(ref, n, 'nc')]])
    return ['kicad_sch', ['version', SCH_VERSION], ['generator', Q('eeschema')], ['generator_version', Q('9.0')], ['uuid', root],
            ['paper', Q('A0')], ['lib_symbols', *libsyms], *items, ['sheet_instances', ['path', Q('/'), ['page', Q('1')]]],
            ['embedded_fonts', 'no']]

def project():
    def cls(name, width, clearance, priority=0):
        via = B.CLASS_VIA[name]                      # the one via table (board_spec), read by route.py and ktest_route too
        return {'bus_width': 12, 'clearance': clearance, 'diff_pair_gap': 0.25, 'diff_pair_via_gap': 0.25, 'diff_pair_width': 0.2, 'line_style': 0,
                'microvia_diameter': 0.3, 'microvia_drill': 0.1, 'name': name, 'pcb_color': 'rgba(0, 0, 0, 0.000)', 'priority': priority,
                'schematic_color': 'rgba(0, 0, 0, 0.000)', 'track_width': width, 'via_diameter': via[0], 'via_drill': via[1], 'wire_width': 6}
    classes = [cls('Default', B.DEFAULT_CLASS['width'], B.DEFAULT_CLASS['clearance'], priority=2147483647)]   # PB17
    for i, (name, c) in enumerate(B.NETCLASSES.items()):
        classes.append(cls(name, c['width'], c['clearance'], priority=i))
    return {'board': {'design_settings': {'rule_severities': DRC_SEVERITY}},
            'erc': {'erc_exclusions': [], 'meta': {'version': 0}, 'rule_severities': ERC_SEVERITY},
            'meta': {'filename': f'{PROJECT}.kicad_pro', 'version': 3},
            'net_settings': {'classes': classes, 'meta': {'version': 4}, 'net_colors': None, 'netclass_assignments': None,
                             'netclass_patterns': [{'netclass': name, 'pattern': n} for name, c in B.NETCLASSES.items() for n in c['nets']]}}

def sym_lib_table():
    """Project symbol-library table (the CLI has no global one): lets ERC resolve every embedded symbol and compare it with the library."""
    libs = sorted({p['sym'].split(':')[0] for p in B.PARTS.values()})
    return '(sym_lib_table\n' + ''.join(
        f'  (lib (name "{l}")(type "KiCad")(uri "${{KICAD9_SYMBOL_DIR}}/{l}.kicad_sym")(options "")(descr ""))\n' for l in libs) + ')\n'

def main(out_dir=D):
    out = Path(out_dir)
    (out / f'{PROJECT}.kicad_sch').write_text(pretty(schematic()) + '\n')
    (out / f'{PROJECT}.kicad_pro').write_text(json.dumps(project(), indent=2) + '\n')
    (out / 'sym-lib-table').write_text(sym_lib_table())
    (out / f'{PROJECT}.kicad_dru').write_text(B.DRC_RULES)
    export_netlist(out / f'{PROJECT}.kicad_sch', out / f'{PROJECT}.net')
if __name__ == '__main__':
    main()

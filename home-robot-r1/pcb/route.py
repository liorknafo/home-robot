"""Route the placed board (bay B5: 4 layers): scripted pre-routes + Freerouting 2.4.1 (headless) + zone fill -> pcb/r1_power_io.kicad_pcb.

Run with KiCad's Python:  $KPY pcb/route.py               (the defaults route the committed board: --grids 1 (set-up trunk-guards),
                                                          --orders asc, the critical step = CRITICAL_NETS (the Motor class + the
                                                          DRIVER_BUNDLE), stages A-F; one variant, one JVM)
                          $KPY pcb/route.py --grids all   (every set-up in SETUPS; rewrites pcb/r1_power_io.kicad_pcb only on a clean
                                                          result, never touches r1_power_io.kicad_pro)
One router process (one Freerouting JVM, heap JVM_HEAP) at a time by default (--parallel): the routing machine is short of memory and
swap / disk (L5b). It starts from a fresh build_pcb placement every time (B12a: build_pcb.py run into the work dir; all tracks and vias
deleted, zones unfilled), so re-running it on the routed board gives the same board: every Freerouting run is single-threaded and its
input DSN canonical, which makes each variant deterministic.
Work files (DSN, SES, Freerouting logs, per-variant boards) go to a temp dir (--keep DIR keeps them).

Layers (copper.STACKUP): F.Cu, In2.Cu and B.Cu are Freerouting's signal layers; In1.Cu is the solid GND plane, layer type power on the
board (build_pcb), which the Specctra export writes as (type power) - Freerouting never routes a power layer (routable = false) but its
GND plane still joins every GND via and through-hole pad (router_dsn asserts the type). Vias are through vias (all four layers).

Pipeline:
 1. strip the board; add the pre-routes (board_spec.PREROUTES / PREROUTE_VIAS: the LP12 GND vias of the top-side SMD GND pads and the
    B4 stitching vias) LOCKED - KiCad exports locked copper as (type fix), Freerouting keeps it, ImportSpecctraSES re-adds it after
    clearing the board. The SMD GND pads leave the router's GND net (drop_smd_gnd_pins) so it routes no GND stub to them;
 2. per variant (a router set-up in SETUPS x a DSN wiring order), in its own process (worker): stage A (each tap net alone, its own
    zones' keepouts left out; then any --critical nets), B (everything else, whole keepouts, the stage-A copper locked), then C, D, E
    (completion runs that lift the B.Cu GND guards step by step), and F (PB18: the links still open drawn by a grid A* round all the
    copper - grid_complete), each =
    ExportSpecctraDSN -> candidate_dsn (keepouts, canonical wiring) -> Freerouting -> ImportSpecctraSES; evaluate after each (zone
    fill, connectivity, kicad-cli DRC, R4 trunks, GND pads on the In1 plane, R15 on In1, In1 slots, R17, zone keepouts, pad spokes,
    trunk capacity);
 3. keep the best variant (clean first, then the fewest vias, then the least track), drop dangling vias and stubs, fill the zones, save
    with SaveBoard(..., aSkipSettings=True) so the project's net classes stay as they are.

Router keepouts (candidate_dsn). Freerouting reads KiCad's zones as planes (conduction areas that are NOT obstacles to other nets), so:
 - every POWER_ZONES polygon on each of its layers and every F.Cu GND patch is a keepout on that layer (grown ZONE_KEEPOUT_GROW), so no
   other net's track or via enters it. A zone net's pad that no zone of its net covers (a tap: pull-ups on 3V3, LED / test-pad feeds on
   VBAT_SW / VMOT / 5V) is routed first, its net alone (stage A, one Freerouting run per tap net), with that net's own zone keepouts
   left out so the tap reaches its plane anywhere; no other net is in that run, and later stages see every keepout whole;
 - the M3 washers / clamp keep-outs (R17b) on all four layers;
 - via keepouts round every In1 void of the placement that is VOID_GUARD_MIN or longer (a via antipad merged into a socket row's void
   would stretch it past R15's 10 mm);
 - B.Cu guards round the GND pads (gnd_guards): the B.Cu GND fill is extra copper (In1 carries the returns), but the GND trunk pads'
   B.Cu spokes count for their current (PB12) and a pad left with one spoke is a starved thermal."""
import argparse, hashlib, json, math, re, shutil, subprocess, sys, tempfile, time
from pathlib import Path
import pcbnew
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, copper as C, gridroute, tools, zone_check

D = Path(__file__).parent
PCB, PRO, DRU = D / 'r1_power_io.kicad_pcb', D / 'r1_power_io.kicad_pro', D / 'r1_power_io.kicad_dru'   # DRU: custom DRC rules
OUTLINE = json.loads((D / 'board_outline.json').read_text(encoding='utf-8'))
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
LAYER = zone_check.LAYER                                           # STACKUP name -> pcbnew layer id
PLANE = C.GND_PLANE_LAYER                                          # In1.Cu: the GND plane, not routed
ROUTED = tuple(L for L in C.STACKUP if L != PLANE)                 # F.Cu, In2.Cu, B.Cu: Freerouting's signal layers


def V(u, v): return pcbnew.VECTOR2I(MM(u), MM(v))


def strip(board):
    """Remove every track and via and unfill the zones. BOARD.Delete, not Remove: after Remove the SWIG wrappers of the removed items
    are collected without a destructor and later calls on the board return untyped objects (KiCad 9.0.9)."""
    for item in list(board.GetTracks()): board.Delete(item)
    for z in board.Zones(): z.UnFill()


def add_track(board, net, layer, width, a, b, locked=True):
    t = pcbnew.PCB_TRACK(board); t.SetLayer(LAYER[layer]); t.SetWidth(MM(width)); t.SetStart(V(*a)); t.SetEnd(V(*b))
    t.SetNet(board.FindNet(net)); t.SetLocked(locked); board.Add(t)
    return t


def add_via(board, net, at, locked=True):
    """A through via (F.Cu to B.Cu: all four layers) of the net's via size (board_spec.net_via: the class's, GND pinned at 0.8 / 0.4)."""
    dia, drill = B.net_via(net)
    v = pcbnew.PCB_VIA(board); v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetPosition(V(*at)); v.SetWidth(MM(dia)); v.SetDrill(MM(drill)); v.SetNet(board.FindNet(net)); v.SetLocked(locked); board.Add(v)
    return v


def add_preroutes(board):
    for r in B.PREROUTES:
        for a, b in zip(r['pts'], r['pts'][1:]): add_track(board, r['net'], r['layer'], r['width'], a, b)
    for net, at in B.PREROUTE_VIAS: add_via(board, net, at)


def preroute_keys():
    segs = {(r['net'], LAYER[r['layer']], frozenset({tuple(a), tuple(b)})) for r in B.PREROUTES for a, b in zip(r['pts'], r['pts'][1:])}
    vias = {(net, tuple(at)) for net, at in B.PREROUTE_VIAS}
    return segs, vias


def missing_preroutes(board):
    """Pre-route segments / vias not on the board with their own net (KiCad merges the nets of copper that touches when it saves and
    loads a board, so a pre-route that touched another net's copper would come back renamed)."""
    segs, vias = preroute_keys()
    r3 = lambda p: (round(TOMM(p.x), 3), round(TOMM(p.y), 3))
    have = {(t.GetNetname(), t.GetLayer(), frozenset({r3(t.GetStart()), r3(t.GetEnd())})) for t in board.GetTracks() if t.GetClass() != 'PCB_VIA'}
    have_v = {(t.GetNetname(), r3(t.GetPosition())) for t in board.GetTracks() if t.GetClass() == 'PCB_VIA'}
    return sorted(map(str, (segs - have) | (vias - have_v)))


def relock(board):
    """Lock exactly the pre-routes; everything the router added stays unlocked."""
    segs, vias = preroute_keys()
    r3 = lambda p: (round(TOMM(p.x), 3), round(TOMM(p.y), 3))
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_VIA': t.SetLocked((t.GetNetname(), r3(t.GetPosition())) in vias)
        else: t.SetLocked((t.GetNetname(), t.GetLayer(), frozenset({r3(t.GetStart()), r3(t.GetEnd())})) in segs)


# --- geometry helpers --------------------------------------------------------------------------------------------------------------------
def polyset(pts):
    ps = pcbnew.SHAPE_POLY_SET(); ps.NewOutline()
    for u, v in pts: ps.Append(MM(u), MM(v))
    return ps


def rings(ps):
    """[(outline [(u, v)], [hole [(u, v)], ...])] of a polygon set, mm."""
    pts = lambda c: [(TOMM(c.CPoint(i).x), TOMM(c.CPoint(i).y)) for i in range(c.PointCount())]
    return [(pts(ps.Outline(k)), [pts(ps.Hole(k, h)) for h in range(ps.HoleCount(k))]) for k in range(ps.OutlineCount())]


def inflated_ps(pts, d):
    """Polygon (u, v) list grown by d mm (mitred corners), as a SHAPE_POLY_SET."""
    ps = polyset(pts)
    ps.Inflate(MM(d), pcbnew.CORNER_STRATEGY_CHAMFER_ACUTE_CORNERS, MM(0.01))
    return ps


def circle_pts(u, v, r, n=32):
    """Polygon round a circle of radius r (its edges outside the circle)."""
    R = r / math.cos(math.pi / n)
    return [(u + R * math.cos(2 * math.pi * (k + .5) / n), v + R * math.sin(2 * math.pi * (k + .5) / n)) for k in range(n)]


def nearest_on_ring(pts, u, v):
    """The point of the closed polygon boundary `pts` nearest (u, v)."""
    best = None
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        L2 = (x1 - x0) ** 2 + (y1 - y0) ** 2
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((u - x0) * (x1 - x0) + (v - y0) * (y1 - y0)) / L2))
        q = (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        d = math.hypot(q[0] - u, q[1] - v)
        if best is None or d < best[0]: best = (d, q)
    return best[1]


# --- DSN edits -------------------------------------------------------------------------------------------------------------------------
def um(u, v): return f'{round(u * 1000)} {-round(v * 1000)}'          # DSN: unit um, y axis up (board v = -y)


def dsn_polygon(layer, pts):
    pts = list(pts) + [pts[0]]
    return f'(polygon {layer} 0 ' + ' '.join(um(u, v) for u, v in pts) + ')'


def dsn_area(kind, name, layer, outline, holes=()):
    """A structure keepout of `kind` (keepout: every new wire and via; via_keepout: vias only) with its holes as (window ...)s."""
    return (f'    ({kind} "{name}" {dsn_polygon(layer, outline)}' + ''.join(f' (window {dsn_polygon(layer, h)})' for h in holes)
            + ')\n')


def insert_in_structure(text, block):
    """Insert `block` at the end of the DSN (structure ...) scope."""
    i = text.index('\n  (placement')
    j = text.rindex('\n  )', 0, i)
    return text[:j + 1] + block + text[j + 1:]


def layer_types(text):
    """{layer: type} of the DSN's (structure (layer ...)) entries."""
    return dict(re.findall(r'\(layer (\S+)\s+\(type (\w+)\)', text))


WIRE = re.compile(r'\(wire \(path (\S+) (\S+)((?:\s+-?[\d.]+\s+-?[\d.]+){3,})\)(.*?\))\n')


def split_wires(text):
    """KiCad exports chained segments as one multi-point path; Freerouting connects a wire only at its two END points (a pad under an
    interior corner counts as unconnected), so every path becomes 2-point wires."""
    def one(m):
        layer, width, rest = m.group(1), m.group(2), m.group(4)
        xy = m.group(3).split()
        pts = list(zip(xy[0::2], xy[1::2]))
        return ''.join(f'(wire (path {layer} {width}  {a[0]} {a[1]}  {b[0]} {b[1]}){rest}\n    ' for a, b in zip(pts, pts[1:])).rstrip(' ')
    return WIRE.sub(one, text)


WIRE_ORDERS = {             # how canonical() orders the DSN wiring; Freerouting's result depends on it, so it is one of the variants
    'asc': lambda lines: sorted(lines),
    'desc': lambda lines: sorted(lines, reverse=True),
    'net': lambda lines: sorted(lines, key=lambda l: (re.findall(r'\(net (\S+)\)', l), l)),
}
WIRE_ORDER = 'asc'          # set per worker


def canonical(text):
    """Same board, same DSN: KiCad writes the wiring in the board's internal track order (it changes with how the board was loaded and
    saved) and the file path as the design name, and Freerouting's result depends on both. Fix the name and sort the wiring lines
    (WIRE_ORDER)."""
    text = re.sub(r'\A\(pcb [^\n]*', '(pcb r1_power_io', text)
    i = text.index('\n  (wiring\n') + len('\n  (wiring\n'); j = text.index('\n  )', i - 1)
    lines = [l for l in text[i:j].split('\n') if l.strip()]
    return text[:i] + '\n'.join(WIRE_ORDERS[WIRE_ORDER](lines)) + text[j:]


VIA_PAD_KEEP = 0.5        # mm: via copper to any other net's pad copper (R17a); Freerouting keeps the class clearance from a via
VIA_GROW = 2 * (VIA_PAD_KEEP - B.DEFAULT_CLASS['clearance'])   # mm added to every via padstack's diameter in the router's DSN (PB17: 0.7)


def grow_vias(text):
    """The router's DSN with every via padstack VIA_GROW mm wider: Freerouting then keeps the real via VIA_PAD_KEEP from other nets'
    pads (it has no separate via-to-pin clearance for KiCad's net classes); normalize_vias puts the class size back after the import."""
    def one(m):
        return re.sub(r'\(circle (\S+) (\d+(?:\.\d+)?)', lambda c: f'(circle {c.group(1)} {round(float(c.group(2)) + VIA_GROW * 1000)}',
                      m.group(0))
    return re.sub(r'\(padstack "Via\[[^"]*"\n(?:      \(shape[^\n]*\n)+', one, text)


def normalize_vias(board):
    """Every via back to its net class's via size (board_spec.CLASS_VIA): ImportSpecctraSES takes the diameter from the session's
    padstack."""
    for v in [t for t in board.GetTracks() if t.GetClass() == 'PCB_VIA']:
        dia, drill = B.net_via(v.GetNetname())
        v.SetWidth(MM(dia)); v.SetDrill(MM(drill))


def drop_smd_gnd_pins(text):
    """The DSN with the top-side SMD GND pads (board_spec.SMD_GND_VIAS) taken out of the GND net's pin list: their scripted via joins
    them to the In1 plane by overlapping the pad, but Freerouting counts a via as connected only with its centre on the pad and would
    route a GND stub to each one (ruling (c): no GND tracks). The pads stay in the DSN as obstacles."""
    i = text.index('(net GND\n'); j = text.index(')', text.index('(pins', i))
    pins = [t for t in text[text.index('(pins', i) + 5:j].split() if t.split('-')[0] not in B.SMD_GND_VIAS]
    return text[:text.index('(pins', i)] + '(pins ' + ' '.join(pins) + text[j:]


OUTER_CLASSES = gridroute.OUTER_CLASSES   # R4: Motor / Power widths are sized for 1 oz outer copper (In2 is 0.5 oz internal: a 1.0 mm
                                          # Motor track there carries 0.7 A, not 2.1 A): Freerouting and stage F route them on F.Cu / B.Cu only


def outer_only(text):
    """The DSN with a (use_layer F.Cu B.Cu) in the circuit of every OUTER_CLASSES class (Freerouting: NetClass.setActiveRoutingLayer)."""
    for cls in OUTER_CLASSES:
        text, n = re.subn(r'(\(class ' + re.escape(cls) + r'\s[^(]*\(circuit)', r'\1\n        (use_layer F.Cu B.Cu)', text)
        assert n == 1, f'class {cls} not found once in the DSN'
    return text


def router_dsn(text, areas):
    """areas: DSN keepout lines (dsn_area) added at the end of the structure. Checks the layer set-up first: In1.Cu must be a power layer
    (the board's layer type, build_pcb) so Freerouting leaves it alone, the routed layers signal layers."""
    types = layer_types(text)
    want = {L: ('power' if L == PLANE else 'signal') for L in C.STACKUP}
    assert types == want, f'DSN layer types {types}, want {want} (build_pcb marks In1.Cu as a power layer)'
    return canonical(split_wires(grow_vias(drop_smd_gnd_pins(outer_only(insert_in_structure(text, ''.join(areas)))))))


def freeroute(dsn, ses, passes, log):
    """Freerouting 2.4.1 headless (flags from its --help and settings classes: GUI off, analytics and file log off, one thread so the
    result is reproducible). PB18: every routed track keeps its class width - the fanout stage (SMD pin escapes) is off, as it necked
    traces at small SMD pins (0.112 mm ENC_LA_5V at R1, 0.6 mm VMOT at TP2 in every PB17 run; KiCad's DRC does not flag them, ktest_route
    does), and so are automatic neck-down and the necked retry (neck_width_um 0). Returns the run time in seconds."""
    cmd = [tools.JAVA, '-Djava.awt.headless=true', '-XX:+UseSerialGC', f'-Xmx{JVM_HEAP}', '-jar', tools.FREEROUTING, '--gui.enabled=false',
           '--usage_and_diagnostic_data.disable_analytics=true', '--logging.file.enabled=false',
           '--router.fanout.enabled=false', '--router.automatic_neckdown=false', '--router.neck_width_um=0',
           '-de', str(dsn), '-do', str(ses), '-mp', str(passes), '-mt', '1']
    t0 = time.time()
    with open(log, 'w', encoding='utf-8') as f:
        try: r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=RUN_TIMEOUT)
        except subprocess.TimeoutExpired: raise RuntimeError(f'Freerouting ran over {RUN_TIMEOUT} s, see {log}')
    if r.returncode != 0 or not Path(ses).is_file(): raise RuntimeError(f'Freerouting failed, see {log}')
    return time.time() - t0


JVM_HEAP = '1g'           # Freerouting's heap cap (its peak is ~150 MB): the routing machine is short of memory and swap (L5b)
RUN_TIMEOUT = 900         # s per Freerouting run: a normal run takes under 5 min; now and then one hangs in a pass, and that variant is
                          # then dropped (its worker fails) instead of stalling route.py


# --- router set-up ---------------------------------------------------------------------------------------------------------------------
ZONE_KEEPOUT_GROW = 0.45 - B.DEFAULT_CLASS['clearance']   # mm: zone polygons grown for the router (+ the class clearance >= the zones' 0.4)
EDGE_KEEPOUT_GROW = 0.55 - B.DEFAULT_CLASS['clearance']   # mm: cutouts as keepouts on every layer (+ the class clearance: >= 0.5 mm edge clearance)
VOID_GUARD_MIN = 6.0      # mm: In1 voids of the placement this long or longer get a via keepout (zone_check.in1_voids)
VOID_GUARD_GROW = 0.3     # mm the via keepout reaches past the void (a via's antipad then keeps the plane's 0.2 mm web to it)
GND_GUARD = 1.0           # mm beyond each GND pad's radius kept free of B.Cu tracks (clipped at other nets' pads), see gnd_guards
TRUNK_GUARD = 3.0         # same for the GND trunk pads (POWER_TRUNK['GND']): their B.Cu spokes carry current (PB12)
THERMAL_GAP, SPOKE_KEEP = 0.5, 1.2   # mm: relief gap (build_pcb) and how far past it each spoke's band stays free of B.Cu tracks
SPOKE_KEEP_LIGHT = 0.3               # the same in the last-resort completion run (E)
# Router set-ups (the CLI's --grids indices, the name kept from the strip's B.Cu crossing-cell grids): the B.Cu GND guards stage B starts
# with. gnd_guard: guard circle past every GND pad's radius (0: only the GND trunk pads get one, TRUNK_GUARD); spoke_keep: spoke bands.
SETUPS = [dict(name='full-guards', gnd_guard=GND_GUARD, spoke_keep=SPOKE_KEEP),
          dict(name='trunk-guards', gnd_guard=0.0, spoke_keep=SPOKE_KEEP),
          dict(name='light-guards', gnd_guard=0.0, spoke_keep=SPOKE_KEEP_LIGHT)]
SETUP = SETUPS[1]         # set per worker


def zone_polys():
    """[(net, layer, polygon, trunk)] of the copper the router keeps other nets out of: every POWER_ZONES entry on each of its layers
    (trunk: a trunk entry, not a helper pour), and the F.Cu GND patches (board_spec.GND_PATCHES, trunk False)."""
    return ([(z['net'], L, z['poly'], z.get('trunk', True)) for z in B.POWER_ZONES for L in z['layers']]
            + [('GND', 'F.Cu', p, False) for p in B.GND_PATCHES])


def taps(board):
    """[(net, (u, v), 'REF.pin')] - the zone nets' pads (GND aside: the In1 plane joins those) that no zone polygon of their net covers on
    a layer the pad is on: each needs a track or via to its net's zone copper."""
    zones = {}
    for net, L, pts, _ in zone_polys(): zones.setdefault(net, []).append((L, polyset(pts)))
    out = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            net = p.GetNetname()
            if net not in zones or net == 'GND': continue
            if not any(p.IsOnLayer(LAYER[L]) and z.Contains(p.GetPosition()) for L, z in zones[net]):
                out.append((net, (TOMM(p.GetPosition().x), TOMM(p.GetPosition().y)), f'{fp.GetReference()}.{p.GetNumber()}'))
    return sorted(out)


def zone_keepouts(board, own=None):
    """DSN keepout lines for zone_polys() on their layers, each grown ZONE_KEEPOUT_GROW - except the zones of net `own`, which get none:
    the stage that routes that net alone (worker, stage A) lets its taps reach its zones anywhere; no other net is routed there, and
    every later stage sees every keepout whole with the stage-A copper locked."""
    out = []
    for k, (net, L, pts, _) in enumerate(zone_polys()):
        if net == own: continue
        ko = inflated_ps(pts, ZONE_KEEPOUT_GROW)
        for j, (outline, holes) in enumerate(rings(ko)):
            out.append(dsn_area('keepout', f'zone{k}_{net}_{L}_{j}', L, outline, holes))
    return out


def fixed_keepouts(board):
    """DSN keepout lines that never change: the M3 washers / clamp keep-outs (R17b: only GND within zone_check.hole_keep) and the
    outline's cutouts, on all four layers."""
    out = []
    for h in zone_check.MOUNT_HOLES:
        c = board.FindFootprintByReference(h).GetPosition()
        out += [dsn_area('keepout', f'hole_{h}_{L}', L, circle_pts(TOMM(c.x), TOMM(c.y), zone_check.hole_keep(h))) for L in C.STACKUP]
    for i, c in enumerate(OUTLINE['cutouts_uv']):
        for L in C.STACKUP:
            out += [dsn_area('keepout', f'cutout{i}_{L}_{j}', L, o, hs) for j, (o, hs) in enumerate(rings(inflated_ps(c, EDGE_KEEPOUT_GROW)))]
    return out


VOID_KEEP = []            # set per worker: void_keepouts(baseline)


def void_keepouts(base):
    """DSN via keepouts (every layer) round each In1 void of the filled placement baseline that is VOID_GUARD_MIN mm or longer, grown
    VOID_GUARD_GROW: a via there would merge its antipad into the void (the socket rows' merged antipads are already 9.3 mm)."""
    out = []
    for k, (length, _, pts) in enumerate(v for v in zone_check.in1_voids(base, rings=True) if v[0] >= VOID_GUARD_MIN):
        for j, (o, hs) in enumerate(rings(inflated_ps(pts, VOID_GUARD_GROW))):
            out += [dsn_area('via_keepout', f'void{k}_{j}_{L}', L, o) for L in C.STACKUP]
    return out


def gnd_guards(board, trunk_guard=TRUNK_GUARD, spoke_keep=SPOKE_KEEP, gnd_guard=GND_GUARD, lifted=()):
    """B.Cu keepouts that keep the B.Cu GND fill round every GND pad: a circle (pad radius + gnd_guard, or + trunk_guard for the GND trunk
    pads) and, so that each pad keeps >= 2 relief spokes even in a 2.54 mm pin row, a 2 mm wide band along each of the 4 spoke
    directions (pad radius + relief gap + spoke_keep long). Nothing may overlap another net's pad (it stops 0.3 mm short, the band in a
    blocked direction is dropped): Freerouting cannot connect to a pad that a keepout overlaps, even on the other layer. Guards of pads
    inside a `lifted` window (u0, v0, u1, v1) are left out, the GND trunk pads' excepted.
    Returns ([(diameter, u, v)] circles, [(u0, v0, u1, v1)] bands)."""
    trunk = set(B.POWER_TRUNK['GND'])
    pads = [(TOMM(p.GetPosition().x), TOMM(p.GetPosition().y), TOMM(max(p.GetSize().x, p.GetSize().y)) / 2, p.GetNetname(),
             (fp.GetReference(), p.GetNumber())) for fp in board.GetFootprints() for p in fp.Pads()]
    others = [(u, v, r) for u, v, r, net, _ in pads if net != 'GND']

    def clear(u0, v0, u1, v1):
        return all(((max(u0 - u, 0, u - u1)) ** 2 + (max(v0 - v, 0, v - v1)) ** 2) ** 0.5 > r + 0.3 for u, v, r in others)
    circles, bands = [], []
    for u, v, r, net, key in pads:
        if net != 'GND': continue
        if key not in trunk and any(w[0] <= u <= w[2] and w[1] <= v <= w[3] for w in lifted): continue
        want = r + (trunk_guard if key in trunk else gnd_guard)
        room = min((((u - u2) ** 2 + (v - v2) ** 2) ** 0.5 - r2 - 0.3 for u2, v2, r2 in others), default=want)
        if min(want, room) > r + 0.2: circles.append((2 * min(want, room), u, v))
        for du, dv in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            for length in (r + THERMAL_GAP + spoke_keep, r + THERMAL_GAP + 0.4):
                a, b = (u, v), (u + du * length, v + dv * length)
                rect = (min(a[0], b[0]) - (1.0 if du == 0 else 0), min(a[1], b[1]) - (1.0 if dv == 0 else 0),
                        max(a[0], b[0]) + (1.0 if du == 0 else 0), max(a[1], b[1]) + (1.0 if dv == 0 else 0))
                if clear(*rect): bands.append(rect); break
    return circles, bands


def candidate_dsn(raw, board, lifted=(), trunk_only=False, light=False, own=None):
    """The router's DSN: the zone, hole, cutout and In1-void keepouts and the B.Cu GND guards of the worker's SETUP. lifted: windows
    (u0, v0, u1, v1) where the ordinary GND pads' guards are left out (stage C); trunk_only: guard circles only round the GND trunk pads
    (stage D); light: also the spoke bands only SPOKE_KEEP_LIGHT long (stage E: every GND pad still keeps its relief spokes); own: the
    net routed alone in a stage-A step, whose own zones get no keepout (zone_keepouts)."""
    circles, bands = gnd_guards(board, TRUNK_GUARD, SPOKE_KEEP_LIGHT if light else SETUP['spoke_keep'],
                                0.0 if (trunk_only or light) else SETUP['gnd_guard'], lifted)
    areas = zone_keepouts(board, own) + fixed_keepouts(board) + VOID_KEEP
    areas += [dsn_area('keepout', f'gnd_spoke{i}', 'B.Cu', B.rect_pts(r)) for i, r in enumerate(bands)]
    areas += [f'    (keepout "gnd_guard{i}" (circle B.Cu {round(d * 1000)} {um(u, v)}))\n' for i, (d, u, v) in enumerate(circles)]
    return router_dsn(raw, areas)


def unconnected_pairs(board, nets=None, with_net=False):
    """One (point, point) pair per missing link, computed here rather than taken from the DRC report (KiCad picks among equivalent
    ratsnest edges differently from run to run, and stage C must be reproducible): each net's copper - pads, tracks, vias, zone fill
    pieces - grouped into clusters by contact on a shared layer; the clusters joined by a minimum spanning tree over their closest pairs
    of anchor points (pad centres, track ends, via centres). with_net: (net, point, point) triples (stage F routes each link on its own
    net)."""
    layers = [LAYER[L] for L in C.STACKUP]
    def shape(item, L):
        ps = pcbnew.SHAPE_POLY_SET(); item.TransformShapeToPolygon(ps, L, MM(0.005), MM(0.005), pcbnew.ERROR_OUTSIDE)
        return ps
    items = {}
    for fp in board.GetFootprints():
        for q in fp.Pads():
            if q.GetNetname():
                items.setdefault(q.GetNetname(), []).append(({L: shape(q, L) for L in layers if q.IsOnLayer(L)}, [q.GetPosition()]))
    for t in board.GetTracks():
        pts = [t.GetPosition()] if t.GetClass() == 'PCB_VIA' else [t.GetStart(), t.GetEnd()]
        items.setdefault(t.GetNetname(), []).append(({L: shape(t, L) for L in layers if t.IsOnLayer(L)}, pts))
    for z in board.Zones():
        if z.GetIsRuleArea() or not z.IsFilled(): continue
        for pc in zone_check.pieces(zone_check.solid_fill(z, z.GetLayer()), min_area=0.0):
            o = pc.Outline(0)
            items.setdefault(z.GetNetname(), []).append(({z.GetLayer(): pc}, [o.CPoint(i) for i in range(0, o.PointCount(), 8)]))
    pairs = []
    for net in sorted(items):
        if nets is not None and net not in nets: continue
        its = items[net]
        group = list(range(len(its)))
        def root(i):
            while group[i] != i: i = group[i]
            return i
        boxes = [{L: ps.BBox() for L, ps in it[0].items()} for it in its]
        for i in range(len(its)):
            for j in range(i + 1, len(its)):
                if root(i) == root(j): continue
                for L in set(its[i][0]) & set(its[j][0]):
                    if not boxes[i][L].Intersects(boxes[j][L]): continue
                    x = its[i][0][L].CloneDropTriangulation(); x.BooleanIntersection(its[j][0][L])
                    if x.OutlineCount(): group[root(i)] = root(j); break
        clusters = {}
        for i, it in enumerate(its): clusters.setdefault(root(i), []).extend((TOMM(p.x), TOMM(p.y)) for p in it[1])
        cl = sorted(sorted(c) for c in clusters.values())
        if len(cl) < 2: continue
        joined = [cl[0]]; rest = cl[1:]
        while rest:                                    # Prim: the closest pair between the joined clusters and the rest
            d, a, b, k = min(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2, a, b, k)
                             for c in joined for a in c for k, r in enumerate(rest) for b in r)
            pairs.append((net, a, b) if with_net else (a, b)); joined.append(rest.pop(k))
    return pairs


def drop_dangling_stubs(board):
    """Delete the router's track segments with a free end (nothing of their net there: no other track end or body, via, pad or zone
    fill), repeatedly - the stubs left where a dangling via was removed. The zones must be filled. Returns how many it deleted."""
    dropped = 0
    while True:
        tracks = [t for t in board.GetTracks() if t.GetClass() != 'PCB_VIA']
        vias = [t for t in board.GetTracks() if t.GetClass() == 'PCB_VIA']
        pads = [q for fp in board.GetFootprints() for q in fp.Pads()]
        zones = [z for z in board.Zones() if not z.GetIsRuleArea() and z.IsFilled()]
        def joined(t, p):
            n, L = t.GetNetname(), t.GetLayer()
            return (any(o is not t and o.GetNetname() == n and o.GetLayer() == L and o.HitTest(p) for o in tracks)
                    or any(v.GetNetname() == n and v.HitTest(p) for v in vias)
                    or any(q.GetNetname() == n and q.IsOnLayer(L) and q.HitTest(p) for q in pads)
                    or any(z.GetNetname() == n and z.GetLayer() == L and z.GetFilledPolysList(L).Contains(p) for z in zones))
        stubs = [t for t in tracks if not t.IsLocked() and not (joined(t, t.GetStart()) and joined(t, t.GetEnd()))]
        if not stubs: return dropped
        for t in stubs: board.Delete(t)
        dropped += len(stubs)


def drop_dangling_vias(board):
    """Delete the router's vias that join copper on one layer only (KiCad's via_dangling): track ends at the via, a through-hole pad of
    its net under it (all layers) and the filled zones of its net that contain it (a tap's via into a plane) each count their layer. The
    zones must be filled. Returns how many it deleted."""
    key = lambda p: (p.x, p.y)
    ends = {}
    for t in board.GetTracks():
        if t.GetClass() != 'PCB_VIA':
            for p in (t.GetStart(), t.GetEnd()): ends.setdefault((t.GetNetname(), key(p)), set()).add(t.GetLayer())
    zones = [z for z in board.Zones() if not z.GetIsRuleArea() and z.IsFilled()]
    dropped = 0
    for v in [t for t in board.GetTracks() if t.GetClass() == 'PCB_VIA' and not t.IsLocked()]:
        layers = set(ends.get((v.GetNetname(), key(v.GetPosition())), ()))
        for fp in board.GetFootprints():
            for q in fp.Pads():
                if q.GetNetname() == v.GetNetname() and q.HitTest(v.GetPosition()): layers |= set(LAYER[L] for L in C.STACKUP if q.IsOnLayer(LAYER[L]))
        for z in zones:
            if z.GetNetname() == v.GetNetname() and z.GetFilledPolysList(z.GetLayer()).Contains(v.GetPosition()): layers.add(z.GetLayer())
        if len(layers) < 2:
            board.Delete(v); dropped += 1
    return dropped


_BASELINE = {}


def baseline(work):
    """The placement baseline for the return-path and In1-void checks: placed.kicad_pcb (placement + locked pre-routes), zones filled."""
    if work not in _BASELINE:
        b = pcbnew.LoadBoard(str(work / 'placed.kicad_pcb'))
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        _BASELINE[work] = b
    return _BASELINE[work]


IN1_SLOT_MAX = 10.0       # mm: R15, no In1 void longer (zone_check.in1_voids, measured along its bends)


def scripted_via(t):
    """The item is one of the scripted pre-route vias (board_spec.PREROUTE_VIAS: LP12 GND vias and stitching vias)."""
    return t.GetClass() == 'PCB_VIA' and (t.GetNetname(), (round(TOMM(t.GetPosition().x), 3), round(TOMM(t.GetPosition().y), 3))) in {
        (n, (round(u, 3), round(v, 3))) for n, (u, v) in B.PREROUTE_VIAS}


def foreign_zone_copper(board):
    """[(item class, net, zone net, layer)] - tracks / vias of another net with copper inside a zone_polys() polygon on its layer. The
    scripted pre-route vias are left out inside helper pours only (the B4 3V3 pour under the Pico flows round the LP12 GND vias of the
    pull-ups; ktest_pcb keeps every pre-route via off the trunk polygons)."""
    tracks = list(board.GetTracks())
    bad = []
    for net, L, pts, trunk in zone_polys():
        lid, zone = LAYER[L], polyset(pts)
        bb = zone.BBox()
        for t in tracks:
            if t.GetNetname() == net or not t.IsOnLayer(lid) or not t.GetBoundingBox().Intersects(bb): continue
            if not trunk and scripted_via(t): continue
            x = pcbnew.SHAPE_POLY_SET(); t.TransformShapeToPolygon(x, lid, 0, MM(0.005), pcbnew.ERROR_OUTSIDE)
            x.BooleanIntersection(zone)
            if x.OutlineCount(): bad.append((t.GetClass(), t.GetNetname(), net, L))
    return bad


def narrow_tracks(board):
    """[(net, layer or 'via', size mm)] - tracks narrower than their net class's width and vias smaller than its via (board_spec.CLASS_VIA,
    GND at GND_VIA), the board_spec.NARROW_TRACKS segments aside (PB18: scripted probe feeds with no load). ktest_route checks the same."""
    width = {n: c['width'] for n, c in B.NETCLASSES.items()}
    ok = B.narrow_track_keys()
    r3 = lambda p: (round(TOMM(p.x), 3), round(TOMM(p.y), 3))
    bad = []
    for t in board.GetTracks():
        net, cls = t.GetNetname(), t.GetNet().GetNetClassName()
        if t.GetClass() == 'PCB_VIA':
            dia, drill = B.GND_VIA if net == 'GND' else B.CLASS_VIA.get(cls, B.CLASS_VIA['Default'])
            if TOMM(t.GetWidth(pcbnew.F_Cu)) < dia - 1e-6 or TOMM(t.GetDrillValue()) < drill - 1e-6:
                bad.append((net, 'via', round(TOMM(t.GetWidth(pcbnew.F_Cu)), 3)))
        elif TOMM(t.GetWidth()) < width.get(cls, B.DEFAULT_CLASS['width']) - 1e-6:
            key = (net, t.GetLayerName(), round(TOMM(t.GetWidth()), 3), frozenset({r3(t.GetStart()), r3(t.GetEnd())}))
            if key not in ok: bad.append((net, t.GetLayerName(), round(TOMM(t.GetWidth()), 3)))
    return sorted(set(bad))


def quality(board, base):
    """R15 / R17 / zone checks on a filled board: (violation count, report dict) - In1 return paths over their limits (zone_check), In1
    voids over IN1_SLOT_MAX, vias closer than VIA_PAD_KEEP to another net's pad, non-GND copper at the M3 holes, GND tracks, other nets'
    copper in the zone polygons, tracks on In1, Power-class router tracks and Motor / Power router tracks on an inner layer (R4), tracks /
    vias under their class size (PB18; the listed NARROW_TRACKS aside), trunk pins whose spokes fall short of their current (PB12; B6: copper
    reached only through vias counts up to the via barrels; board_spec.PAD_PENDING pins are reported apart), trunk nets short of their
    current (PB11)."""
    paths = zone_check.return_paths(board, base, layer=PLANE)
    over = [(f'{a[0]}.{a[1]}-{b[0]}.{b[1]}', round(r, 3)) for (a, b), _, _, r, lim in paths if not r <= lim]
    all_voids = zone_check.in1_voids(board)
    voids = [v[:2] for v in all_voids if v[0] > IN1_SLOT_MAX]
    gaps = zone_check.via_pad_gaps(board)
    close = [g for g in gaps if g[0] < VIA_PAD_KEEP - 1e-6]
    holes = zone_check.hole_intrusions(board)
    gnd_tracks = len(zone_check.gnd_copper_problems(board))      # GND tracks, and GND vias other than the SMD pads' plane vias / stitching
    foreign = foreign_zone_copper(board)
    power = set(B.NETCLASSES['Power']['nets'])
    outer = {n for c in OUTER_CLASSES for n in B.NETCLASSES[c]['nets']}
    inner = {LAYER[L] for L in C.STACKUP if L not in ('F.Cu', 'B.Cu')}
    bad_tracks = sorted({(t.GetNetname(), t.GetLayerName()) for t in board.GetTracks() if t.GetClass() != 'PCB_VIA'
                         and (t.GetLayer() == LAYER[PLANE] or (t.GetNetname() in power and not t.IsLocked())
                              or (t.GetNetname() in outer and t.GetLayer() in inner and not t.IsLocked()))})   # PB18 / R4
    narrow = narrow_tracks(board)
    spokes, pending = [], []
    for net, pins in B.POWER_TRUNK.items():
        for ref, pin in pins:
            cap, _ = zone_check.pin_spoke_capacity(board, ref, pin)          # B6: via barrels counted
            if cap < B.pad_current(ref, pin) - 1e-9:
                (pending if (ref, pin) in B.PAD_PENDING else spokes).append((f'{ref}.{pin}', round(cap, 2), round(B.pad_current(ref, pin), 2)))
    capacity = []
    for net in (n for n in B.POWER_TRUNK if n != 'GND'):
        a = zone_check.trunk_capacity(board, net)['capacity_a']
        if a < B.CURRENT_A[net]: capacity.append((net, a))
    n = (len(over) + len(voids) + len(close) + len(holes) + gnd_tracks + len(foreign) + len(bad_tracks) + len(narrow) + len(spokes)
         + len(capacity))
    return n, dict(returns_over=over, worst_return=max((round(r, 3) for *_, r, _ in paths), default=0), in1_voids_over=voids,
                   longest_in1_void=all_voids[0][:2], min_via_pad=gaps[0][0] if gaps else None, vias_near_pads=close,
                   hole_intrusions=holes, gnd_tracks=gnd_tracks, foreign_zone_copper=foreign, bad_tracks=bad_tracks, narrow_tracks=narrow,
                   spokes_short=spokes, spokes_pending=pending,
                   capacity_short=capacity)


def evaluate(board, workdir, base):
    """Fill the zones and measure a routed board: (score tuple, report dict). Lower score is better; score[:4] == (0, 0, 0, 0) is a
    clean board (connected, no DRC errors, R4 trunks, every GND pad on the In1 plane, the quality checks); score[4] = vias, score[5] =
    track mm."""
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()
    unconnected = board.GetConnectivity().GetUnconnectedCount(False)
    tracks = [t for t in board.GetTracks() if t.GetClass() in ('PCB_TRACK', 'PCB_ARC')]
    vias = [t for t in board.GetTracks() if t.GetClass() == 'PCB_VIA']
    per = {L: round(sum(TOMM(t.GetLength()) for t in tracks if t.GetLayer() == LAYER[L])) for L in C.STACKUP}
    total = sum(per.values())
    out = workdir / 'eval.kicad_pcb'
    pcbnew.SaveBoard(str(out), board, True)
    shutil.copy(PRO, workdir / 'eval.kicad_pro'); shutil.copy(DRU, workdir / 'eval.kicad_dru')
    subprocess.run([tools.KICAD_CLI, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', str(workdir / 'drc.json'), str(out)],
                   check=True, capture_output=True)
    drc = json.loads((workdir / 'drc.json').read_text(encoding='utf-8'))
    drc_errors = len(drc['violations'])
    drc_types = {}
    for v in drc['violations']: drc_types[v['type']] = drc_types.get(v['type'], 0) + 1
    pairs = unconnected_pairs(board) if drc['unconnected_items'] else []
    trunk = zone_check.trunk_problems(board)
    gnd_off = zone_check.gnd_pads_off_main_piece(board, PLANE)
    _, gnd_area = zone_check.gnd_main_piece(board, PLANE)
    q, qrep = quality(board, base)
    router_vias = sum(1 for v in vias if not v.IsLocked())
    rep = dict(unconnected=unconnected, drc_errors=drc_errors, drc_types=drc_types, trunk_problems=trunk, gnd_pads_off_main=gnd_off,
               in1_main_mm2=round(gnd_area), track_mm=per, total_mm=total, vias=len(vias), router_vias=router_vias, **qrep)
    rep['_pairs'] = pairs
    return (unconnected + drc_errors, len(trunk), len(gnd_off), q, router_vias, total), rep


# Routed first (stage A, alone, before the tap nets), on a free board: the Motor class (1.0 mm wide, 1.0 mm clearance, R16 - the
# hardest to fit once the signals are down) and, since PB17, the driver bundle - the Pico -> DRV_L / DRV_R logic links that cross the
# strip between U1B and the drivers (DRIVER_BUNDLE). --critical NET,NET,... replaces the list, --critical none empties it.
DRIVER_BUNDLE = ['R_IN1', 'R_IN2', 'R_CS', 'DRV_FAULT', 'L_CS', 'BUMP_R', 'COIL_P']
CRITICAL_NETS = list(B.NETCLASSES['Motor']['nets']) + DRIVER_BUNDLE
STAGE2_MARGIN = 6.0       # mm round each missing connection where the completion run (C) lifts the ordinary GND pads' B.Cu guards


def pair_windows(pairs, margin=STAGE2_MARGIN):
    """(u0, v0, u1, v1) round each (point, point) pair, grown by margin mm: where stage C lifts the B.Cu guards."""
    return [(min(p[0], q[0]) - margin, min(p[1], q[1]) - margin, max(p[0], q[0]) + margin, max(p[1], q[1]) + margin) for p, q in pairs]


def only_nets(dsn, keep):
    """The DSN with only the nets in `keep` left to route. Every other net is split into one-item nets - each pin (NAME__k), each wire or
    via of the wiring (NAME__wK) and each plane (NAME__pK) - which Freerouting has nothing to join: the pins and the copper stay
    obstacles, and every new net joins NAME's (class ...) so it keeps NAME's clearance (dropping the net instead would leave netless
    pins at the default 0.25 mm - the Motor class's 1.0 mm, R16, would be lost; leaving its wires under the old name makes Freerouting
    re-create the net and route between their pieces)."""
    split, out, i = {}, [], 0
    for m in re.finditer(r'\n    \(net (\S+)\n      \(pins([^)]*)\)\n    \)', dsn):
        out.append(dsn[i:m.start()])
        name = m.group(1).strip('"')
        if name in keep: out.append(m.group(0))
        else:
            pins = m.group(2).split()
            split[name] = [f'{name}__{k}' for k in range(len(pins))]
            out.append(''.join(f'\n    (net {n}\n      (pins {q})\n    )' for n, q in zip(split[name], pins)))
        i = m.end()
    text = ''.join(out) + dsn[i:]
    count = [0]

    def rename(m, kind):
        name = m.group(2).strip('"')
        if name not in split: return m.group(0)
        count[0] += 1
        new = f'{name}__{kind}{count[0]}'
        split[name].append(new)
        return m.group(1) + new + m.group(3)
    w = text.index('\n  (wiring')
    text = text[:w] + re.sub(r'(\(net )("[^"]*"|[^\s()]+)(\))', lambda m: rename(m, 'w'), text[w:])
    text = re.sub(r'(\(plane )("[^"]*"|[^\s()]+)( \()', lambda m: rename(m, 'p'), text)

    def cls(m):
        extra = [n for nm in m.group(2).split() for n in split.get(nm.strip('"'), [])]
        return m.group(1) + m.group(2) + ''.join(' ' + n for n in extra)
    return re.sub(r'(\(class \S+)((?:\s+[^\s()]+)+)', cls, text)


SPLIT_NET = re.compile(r'\(net "?([^"\s()]+?)__(?:\d+|[wp]\d+)"?(?=[\s)])')   # a one-item net of only_nets in a session


def unrouted(log):
    """Connections Freerouting left unrouted (from its log)."""
    found = re.findall(r'final score: [\d.]+ \((\d+) unrouted', Path(log).read_text(encoding='utf-8'))
    return int(found[-1]) if found else 999


def route_step(name, d, start, dsn_of, passes):
    """Export `start`, write the DSN dsn_of(raw text, board), run Freerouting, import the session into a fresh copy of `start`.
    Returns (board, unrouted count)."""
    b = pcbnew.LoadBoard(str(start))
    assert pcbnew.ExportSpecctraDSN(b, str(d / f'{name}_raw.dsn')), 'DSN export failed'
    (d / f'{name}.dsn').write_bytes(dsn_of((d / f'{name}_raw.dsn').read_text(encoding='utf-8'), b).encode())
    freeroute(d / f'{name}.dsn', d / f'{name}.ses', passes, d / f'{name}.log')
    ses = d / f'{name}.ses'                        # only_nets' one-item nets back to their own names (else KiCad imports them netless)
    ses.write_bytes(SPLIT_NET.sub(lambda m: f'(net "{m.group(1)}"', ses.read_text(encoding='utf-8')).encode())
    b = pcbnew.LoadBoard(str(start))
    assert pcbnew.ImportSpecctraSES(b, str(d / f'{name}.ses')), f'{name}: SES import failed'
    normalize_vias(b)
    for t in [t for t in b.GetTracks() if t.GetNetname() == 'GND' and not t.IsLocked()]: b.Delete(t)   # ruling (c), see drop_smd_gnd_pins
    return b, unrouted(d / f'{name}.log')


def save_locked(b, path, lock=True):
    """Save a stage's board as the next stage's start. lock: Freerouting must keep this copper (else only the pre-routes and the
    copper locked before stay fixed, and the router may rip up and re-route the rest)."""
    if lock:
        for t in b.GetTracks(): t.SetLocked(True)
    pcbnew.SaveBoard(str(path), b, True); shutil.copy(PRO, path.with_suffix('.kicad_pro'))
    return path


def lock_nets(b, nets):
    """Lock every track and via of `nets` (the stage-A copper: later stages keep it - with every zone keepout whole they could not
    re-route a tap to its plane)."""
    for t in b.GetTracks():
        if t.GetNetname() in nets: t.SetLocked(True)


def tap_width(net):
    """Track width (mm) of the net's class (board_spec.NETCLASSES; 0 for the default class)."""
    return next((c['width'] for c in B.NETCLASSES.values() if net in c['nets']), 0.0)


# --- stage F (PB18): grid completion of the links Freerouting left open -------------------------------------------------------------
# gridroute.search (A*, pure) over F.Cu / In2.Cu / B.Cu on a GRID_RES grid - F.Cu / B.Cu only for the OUTER_CLASSES (R4), and those
# nets on one layer end to end, no via (one via is too thin for a Motor / Power current) - with layer changes by the net's via
# for every other class, round every other net's copper (pads, tracks, vias) at the larger class clearance, the other nets'
# zone polygons (GRID_ZONE), the M3 / clamp keep-outs, the board edge and, on B.Cu, the GND pads' spoke guards of the last-resort stage E
# (gnd_guards, light); a via also keeps R17a's VIA_PAD_KEEP from other nets' pads, stays out of every pad (its own net's too: no via in
# a pad, above all not in a reflowed SMD pad) and out of the In1-void via keep-outs (void_keepouts). Each link found is added as router
# copper; evaluate() then judges the board as for every other stage (fills, DRC, R15, In1 slots, spokes and capacities decide).
GRID_RES = 0.1            # mm
GRID_MARGIN = 0.06        # mm added to every keep-out for the grid's discretisation
GRID_EDGE = 0.5           # mm: copper to the board edge
GRID_ZONE = 0.45          # mm: copper to another net's zone polygon (>= the zones' 0.4 fill clearance)
GRID_LAYERS = gridroute.LAYERS


def net_clearance(net):
    return next((c['clearance'] for c in B.NETCLASSES.values() if net in c['nets']), B.DEFAULT_CLASS['clearance'])


def net_width(net):
    return next((c['width'] for c in B.NETCLASSES.values() if net in c['nets']), B.DEFAULT_CLASS['width'])


def raster(ps, grid, w, h):
    """Even-odd scanline fill of a SHAPE_POLY_SET (outlines with holes) into grid (bytearray w * h, GRID_RES cells)."""
    for i in range(ps.OutlineCount()):
        chains = [ps.Outline(i)] + [ps.Hole(i, k) for k in range(ps.HoleCount(i))]
        edges, y0, y1 = [], 1e9, -1e9
        for ch in chains:
            n = ch.PointCount()
            pts = [(TOMM(ch.CPoint(k).x), TOMM(ch.CPoint(k).y)) for k in range(n)]
            for k in range(n):
                a, c = pts[k], pts[(k + 1) % n]
                if a[1] != c[1]: edges.append((a, c))
                y0, y1 = min(y0, a[1]), max(y1, a[1])
        for r in range(max(0, int(math.floor(y0 / GRID_RES))), min(h - 1, int(math.ceil(y1 / GRID_RES))) + 1):
            y = r * GRID_RES
            xs = sorted(ax + (y - ay) * (cx - ax) / (cy - ay) for (ax, ay), (cx, cy) in edges if (ay <= y < cy) or (cy <= y < ay))
            for k in range(0, len(xs) - 1, 2):
                c0, c1 = max(0, int(math.ceil(xs[k] / GRID_RES))), min(w - 1, int(math.floor(xs[k + 1] / GRID_RES)))
                if c1 >= c0: grid[r * w + c0: r * w + c1 + 1] = b'\x01' * (c1 - c0 + 1)


def grid_obstacles(board, net, half, pad_extra, w, h, guards):
    """{layer: bytearray}: the cells where copper of `net` with the given half size (track half width or via radius) would break a
    clearance (pad_extra: more round other nets' pads - R17a for vias); guards: the B.Cu GND guard circles / bands (gnd_guards)."""
    me = net_clearance(net)
    area = pcbnew.SHAPE_POLY_SET(); board.GetBoardPolygonOutlines(area)
    area.Deflate(MM(GRID_EDGE + half + GRID_MARGIN), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.02))
    inside = bytearray(w * h); raster(area, inside, w, h)
    outside = bytes(1 - c for c in inside)
    holes = [(board.FindFootprintByReference(r).GetPosition(), zone_check.hole_keep(r)) for r in zone_check.MOUNT_HOLES]
    grids = {}
    for L in GRID_LAYERS:
        lid, g = LAYER[L], bytearray(outside)
        for fp in board.GetFootprints():
            for p in fp.Pads():
                if p.GetNetname() == net or not p.IsOnLayer(lid): continue
                c = max(me, net_clearance(p.GetNetname())) + half + GRID_MARGIN + pad_extra
                ps = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps, lid, MM(c), MM(0.02), pcbnew.ERROR_OUTSIDE); raster(ps, g, w, h)
        for t in board.GetTracks():
            if t.GetNetname() == net or not t.IsOnLayer(lid): continue
            c = max(me, net_clearance(t.GetNetname())) + half + GRID_MARGIN
            ps = pcbnew.SHAPE_POLY_SET(); t.TransformShapeToPolygon(ps, lid, MM(c), MM(0.02), pcbnew.ERROR_OUTSIDE); raster(ps, g, w, h)
        for zn, zl, pts, _ in zone_polys():
            if zl != L or zn == net: continue
            ps = polyset(pts); ps.Inflate(MM(GRID_ZONE + half + GRID_MARGIN), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.02))
            raster(ps, g, w, h)
        if net != 'GND':
            for pos, keep in holes:
                raster(polyset(circle_pts(TOMM(pos.x), TOMM(pos.y), keep + half + GRID_MARGIN, 48)), g, w, h)
            if L == 'B.Cu':
                circles, bands = guards
                for d, u, v in circles: raster(polyset(circle_pts(u, v, d / 2 + half + GRID_MARGIN, 48)), g, w, h)
                for r in bands: raster(inflated_ps(B.rect_pts(r), half + GRID_MARGIN), g, w, h)
        grids[L] = g
    return grids


def copper_layers_at(board, net, pt):
    """The signal layers on which net's copper (pad, track, via, zone fill) covers pt."""
    v = pcbnew.VECTOR2I(MM(pt[0]), MM(pt[1]))
    out = set()
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == net and p.HitTest(v): out |= {L for L in GRID_LAYERS if p.IsOnLayer(LAYER[L])}
    for t in board.GetTracks():
        if t.GetNetname() != net: continue
        if t.GetClass() == 'PCB_VIA':
            if t.HitTest(v): out |= set(GRID_LAYERS)
        elif t.HitTest(v, MM(0.01)): out.add(t.GetLayerName())
    for z in board.Zones():
        if z.GetNetname() == net:
            out |= {L for L in GRID_LAYERS if z.IsOnLayer(LAYER[L]) and z.HitTestFilledArea(LAYER[L], v)}
    return out


def grid_route(board, base, net, a, c):
    """gridroute.search from point a to point c of `net`: [(layer, ix, iy)] or None."""
    w, h = int(math.ceil(B.BOARD_W / GRID_RES)) + 2, int(math.ceil(B.BOARD_H / GRID_RES)) + 2
    guards = gnd_guards(board, TRUNK_GUARD, SPOKE_KEEP_LIGHT, 0.0) if net != 'GND' else ([], [])
    trk = grid_obstacles(board, net, net_width(net) / 2, 0.0, w, h, guards)
    vr = B.net_via(net)[0] / 2
    vg = grid_obstacles(board, net, vr, max(0.0, VIA_PAD_KEEP - net_clearance(net)), w, h, guards)
    novia = bytearray(w * h)
    for fp in board.GetFootprints():                 # no via in any pad, its own net's included (SMD pads are reflowed)
        for p in fp.Pads():
            for L in GRID_LAYERS + ('In1.Cu',):
                if p.IsOnLayer(LAYER[L]):
                    ps = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps, LAYER[L], MM(vr + GRID_MARGIN), MM(0.02), pcbnew.ERROR_OUTSIDE)
                    raster(ps, novia, w, h)
    for length, _, pts in zone_check.in1_voids(base, rings=True):      # the In1-void via keep-outs (void_keepouts)
        if length >= VOID_GUARD_MIN: raster(inflated_ps(pts, VOID_GUARD_GROW + vr + GRID_MARGIN), novia, w, h)
    via_ok = bytes(0 if (x or y or z or n) else 1 for x, y, z, n in zip(vg['F.Cu'], vg['In2.Cu'], vg['B.Cu'], novia))
    la, lc = copper_layers_at(board, net, a), copper_layers_at(board, net, c)
    ca = (int(round(a[0] / GRID_RES)), int(round(a[1] / GRID_RES))); cc = (int(round(c[0] / GRID_RES)), int(round(c[1] / GRID_RES)))
    return gridroute.search(trk, w, h, via_ok, [(L,) + ca for L in la], [(L,) + cc for L in lc], gridroute.layers_for(net),
                            gridroute.vias_allowed(net))


def add_grid_path(board, net, path, a, c):
    """The grid path as tracks (collinear cells merged, the ends on a and c exactly) and vias at its layer changes."""
    runs, vias = gridroute.runs(path, GRID_RES, a, c)
    for L, pts in runs:
        for p, q in zip(pts, pts[1:]):
            if p != q: add_track(board, net, L, net_width(net), p, q, locked=False)
    for v in vias: add_via(board, net, v, locked=False)


def grid_complete(board, base):
    """Stage F: complete each open link (unconnected_pairs) on the grid, in turn, each round the copper of the ones before it. base: the
    filled placement (its In1 voids give the via keep-outs). Returns how many it drew."""
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    done = 0
    for net, a, c in unconnected_pairs(board, with_net=True):
        # B12a: the link's own net. Picking 'a net with copper at both ends' from an (unordered) set of names could pick GND - its B.Cu fill
        # lies under both ends of many signal links - and draw the link as GND tracks
        if not (copper_layers_at(board, net, a) and copper_layers_at(board, net, c)): continue
        path = grid_route(board, base, net, a, c)
        if path: add_grid_path(board, net, path, a, c); done += 1
    return done


def worker(g, order, work, passes):
    """One router set-up (SETUPS[g]) and wiring order, end to end (a separate process):
     A  the critical nets (CRITICAL_NETS: the Motor class plus DRIVER_BUNDLE) alone; then each tap net (taps(): zone nets with pads off
        their zones) alone, one run each (widest class first), its own zones' keepouts left out (own); a step that leaves connections
        open runs again with the ordinary GND guards lifted round each of them (STAGE2_MARGIN); every step's copper is locked for the next;
     B  everything else round the locked stage-A copper, with the set-up's B.Cu GND guards and every zone keepout whole;
     C  if B left connections unrouted: the ordinary GND pads' guards lifted round each missing connection;
     D  if still short: guard circles only round the GND trunk pads, everywhere (spoke bands stay);
     E  if still short: trunk circles and the light spoke bands (candidate_dsn(light=True));
     F  if still short: grid_complete draws the open links one by one on a GRID_RES grid round everything laid (no Freerouting).
    The zone, hole and In1-void keepouts stay in every stage. In C, D and E only the pre-routes and the stage-A copper stay fixed:
    Freerouting may rip up and re-route what B-D laid. Writes setup<g>-<order>/result.kicad_pcb and result.json (score and report)."""
    global WIRE_ORDER, SETUP, VOID_KEEP
    WIRE_ORDER, SETUP = order, SETUPS[g]
    d = work / f'setup{g}-{order}'; d.mkdir(exist_ok=True)
    for ext in ('kicad_pcb', 'kicad_pro'):         # the worker's own copy: parallel workers loading one project race on its lock
        shutil.copy(work / f'placed.{ext}', d / f'placed.{ext}')
    work = d
    VOID_KEEP = void_keepouts(baseline(work))
    # PB14: the widest tap class first (the PowerTap 0.8 mm nets: 5V, VBAT_SW, VMOT), the thin 3V3 taps last - a 0.8 mm tap cannot
    # pass between header pins, so it gets the board before the 3V3 links that can
    tap_nets = sorted({net for net, _, _ in taps(baseline(work))}, key=lambda n: (-tap_width(n), n))
    first = sorted(set(CRITICAL_NETS) | set(tap_nets))
    start = work / 'placed.kicad_pcb'
    for k, net in enumerate((['+'.join(CRITICAL_NETS)] if CRITICAL_NETS else []) + tap_nets):
        nets = set(net.split('+'))
        # the pre-routed Motor / PowerTap nets stay whole in every step; a pre-routed signal net (PB17: 3V3, DRV_SLEEP) is split like
        # any other outside its own step - its locked copper an obstacle, as earlier steps' copper is (3V3 keeps its place: last)
        keep = nets | {r['net'] for r in B.PREROUTES if tap_width(r['net'])} | {'GND'}
        own = net if net in tap_nets else None
        b, n = route_step(f'A{k}', d, start, lambda raw, b: candidate_dsn(only_nets(raw, keep), b, own=own), passes)
        if n:                          # the guards lifted round each connection this step left open
            again = save_locked(b, d / f'A{k}.kicad_pcb')
            pcbnew.ZONE_FILLER(b).Fill(b.Zones())
            wins = pair_windows(unconnected_pairs(b, nets))
            b, n = route_step(f'A{k}x', d, again, lambda raw, b: candidate_dsn(only_nets(raw, keep), b, wins, own=own), passes)
        print(f'stage A{k} {net}: {n} left open', flush=True)
        start = save_locked(b, d / f'A{k}done.kicad_pcb')
    start = save_locked(b, d / 'AB.kicad_pcb')
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    print('stage A: tap nets still open', sorted({net for net in first if unconnected_pairs(b, {net})}), flush=True)
    b, n = route_step('B', d, start, lambda raw, b: candidate_dsn(raw, b), passes)
    score, rep = evaluate(b, d, baseline(work))
    rep['stage'] = 'B'
    print('stage B', score, {k: v for k, v in rep.items() if k != '_pairs'}, flush=True)
    if score[0]:
        windows = pair_windows(rep['_pairs'])
        lock_nets(b, first)
        start = save_locked(b, d / 'BC.kicad_pcb', lock=False)
        bc, _ = route_step('C', d, start, lambda raw, b: candidate_dsn(raw, b, windows), passes)
        sc, rc = evaluate(bc, d, baseline(work))
        rc['stage'] = 'C'
        print('stage C', sc, {k: v for k, v in rc.items() if k != '_pairs'}, flush=True)
        if sc < score: b, score, rep = bc, sc, rc
    if score[0]:                       # D: still short of connections: guard circles only round the GND trunk pads
        lock_nets(b, first)
        start = save_locked(b, d / 'CD.kicad_pcb', lock=False)
        bd, _ = route_step('D', d, start, lambda raw, b: candidate_dsn(raw, b, trunk_only=True), passes)
        sd, rd = evaluate(bd, d, baseline(work))
        rd['stage'] = 'D'
        print('stage D', sd, {k: v for k, v in rd.items() if k != '_pairs'}, flush=True)
        if sd < score: b, score, rep = bd, sd, rd
    if score[0]:                       # E: last resort - the light guards; the pre-routes and the stage-A copper stay fixed
        relock(b)
        lock_nets(b, first)
        start = save_locked(b, d / 'DE.kicad_pcb', lock=False)
        be, _ = route_step('E', d, start, lambda raw, b: candidate_dsn(raw, b, light=True), passes)
        se, re_ = evaluate(be, d, baseline(work))
        re_['stage'] = 'E'
        print('stage E', se, {k: v for k, v in re_.items() if k != '_pairs'}, flush=True)
        if se < score: b, score, rep = be, se, re_
    if score[0]:                       # F (PB18): the links still open, completed on a grid round all the copper laid so far
        relock(b)
        bf = pcbnew.LoadBoard(str(save_locked(b, d / 'EF.kicad_pcb', lock=False)))
        done = grid_complete(bf, baseline(work))
        sf, rf = evaluate(bf, d, baseline(work))
        rf['stage'] = 'F'
        print(f'stage F ({done} links drawn)', sf, {k: v for k, v in rf.items() if k != '_pairs'}, flush=True)
        if sf < score: b, score, rep = bf, sf, rf
    relock(b)
    pcbnew.SaveBoard(str(d / 'result.kicad_pcb'), b, True)
    shutil.copy(PRO, d / 'result.kicad_pro')        # LoadBoard takes the net classes (zone clearances) from it
    rep.pop('_pairs', None)
    (d / 'result.json').write_bytes(json.dumps(dict(score=list(score), rep=rep)).encode())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--passes', type=int, default=DEFAULT_PASSES, help=f'Freerouting max passes per run (default {DEFAULT_PASSES})')
    ap.add_argument('--grids', default=str(DEFAULT_GRID), help=f"comma-separated indices into SETUPS (router set-ups), or 'all' "
                    f"(default: {DEFAULT_GRID}, the set-up that routes the committed board)")
    ap.add_argument('--force', action='store_true', help='write the best board even when it is not clean')
    ap.add_argument('--keep', help='keep the work files (DSN, SES, logs) in this directory')
    ap.add_argument('--out', default=str(PCB), help='routed board to write (default: pcb/r1_power_io.kicad_pcb)')
    ap.add_argument('--orders', default=DEFAULT_ORDER, help=f'DSN wiring orders to try, of {list(WIRE_ORDERS)} (default: {DEFAULT_ORDER}, '
                    'the order that routes the committed board)')
    ap.add_argument('--parallel', type=int, default=MAX_PARALLEL, help=f'router processes at a time (default {MAX_PARALLEL})')
    ap.add_argument('--critical', default='', help="comma-separated nets routed first, alone (default: CRITICAL_NETS, the Motor class + "
                    "DRIVER_BUNDLE; 'none': no such step)")
    ap.add_argument('--worker', type=int, help=argparse.SUPPRESS)      # internal: one set-up / order, in its own process
    ap.add_argument('--order', help=argparse.SUPPRESS)
    ap.add_argument('--work', help=argparse.SUPPRESS)
    args = ap.parse_args()
    pcbnew.KIID.SeedGenerator(1)       # new copper's UUIDs (pre-routes, imported sessions) and so the saved item order: reproducible
    global CRITICAL_NETS
    if args.critical: CRITICAL_NETS = [] if args.critical == 'none' else args.critical.split(',')
    if args.worker is not None:
        worker(args.worker, args.order, Path(args.work), args.passes); return 0
    grids = list(range(len(SETUPS))) if args.grids == 'all' else [int(i) for i in args.grids.split(',')]
    cands = [(g, o) for o in args.orders.split(',') for g in grids]
    tmp = None if args.keep else tempfile.TemporaryDirectory()
    work = Path(args.keep or tmp.name); work.mkdir(parents=True, exist_ok=True)

    placed = work / 'placed.kicad_pcb'                          # the placement + locked pre-routes, the start of every run
    # B12a: the start is a fresh build_pcb placement (its own process, as `$KPY pcb/build_pcb.py` runs), not the committed board: loading a
    # routed board uses up the UUID generator differently, so the pre-routes got other UUIDs, another save order and another DSN (a re-run
    # on the routed board diverged at stage C and left a link open). The placement is board_spec's either way.
    subprocess.run([sys.executable, str(D / 'build_pcb.py'), str(placed)], check=True, capture_output=True)
    shutil.copy(PRO, work / 'placed.kicad_pro')
    board = pcbnew.LoadBoard(str(placed))
    strip(board); add_preroutes(board)
    pcbnew.SaveBoard(str(placed), board, True)
    lost = missing_preroutes(pcbnew.LoadBoard(str(placed)))
    assert not lost, f'pre-routes changed on save (touching another net?): {lost}'

    t0 = time.time()
    waiting, running, results = list(cands), [], []
    while waiting or running:
        while waiting and len(running) < args.parallel:
            g, o = waiting.pop(0)
            log = open(work / f'worker{g}-{o}.log', 'w', encoding='utf-8')
            running.append((g, o, subprocess.Popen([sys.executable, __file__, '--worker', str(g), '--order', o, '--work', str(work),
                                                    '--passes', str(args.passes), '--critical', args.critical], stdout=log, stderr=subprocess.STDOUT)))
        time.sleep(2)
        for g, o, p in [r for r in running if r[2].poll() is not None]:
            running.remove((g, o, p))
            f = work / f'setup{g}-{o}' / 'result.json'
            if p.returncode != 0 or not f.is_file():
                print(f'set-up {g} {o}: worker failed (see {work / f"worker{g}-{o}.log"})'); continue
            r = json.loads(f.read_text(encoding='utf-8'))
            print(f'set-up {g} {SETUPS[g]["name"]} {o}: score {r["score"]} {r["rep"]}', flush=True)
            results.append((tuple(r['score']), g, o, r['rep']))
    router_s = time.time() - t0
    if not results: print('no variant finished'); return 1
    score, g, o, rep = min(results, key=lambda r: (r[0], r[1], r[2]))          # clean first, then the fewest vias, then the least track
    best = pcbnew.LoadBoard(str(work / f'setup{g}-{o}' / 'result.kicad_pcb'))
    pcbnew.ZONE_FILLER(best).Fill(best.Zones())
    dropped = drop_dangling_vias(best)
    if dropped: pcbnew.ZONE_FILLER(best).Fill(best.Zones())
    dropped = (dropped, drop_dangling_stubs(best))
    stage = rep['stage']
    score, rep = evaluate(best, work, baseline(work))              # the board as it will be written (evaluate fills the zones)
    rep.pop('_pairs', None)
    rep['stage'] = stage
    rep['dangling_vias_stubs_dropped'] = dropped
    print(f'Freerouting: {len(cands)} set-up / order variants, {args.parallel} at a time, {args.passes} passes max per run, '
          f'{router_s:.0f} s wall')
    print(f'best: set-up {g} {SETUPS[g]["name"]} wiring {o}: score {score} {rep}')
    clean = tuple(score[:4]) == (0, 0, 0, 0)
    if not clean and not args.force:
        print(f'not clean: {args.out} left as it was (--force writes it anyway)')
        return 1
    props = pcbnew.MAP_STRING_STRING(best.GetProperties())
    # the effective configuration and the tools it ran on (ktest_route prints and checks it): the DSN order follows KiCad's footprint
    # library, the routing the exact Freerouting jar and JVM (LIO-228)
    jar = hashlib.sha256(Path(tools.FREEROUTING).read_bytes()).hexdigest()
    java = re.search(r'version "([^"]+)"', subprocess.run([tools.JAVA, '-version'], capture_output=True, text=True).stderr)[1]
    props['ROUTED_WITH'] = (f'route.py set-up {g} {SETUPS[g]["name"]} wiring {o} stage {rep["stage"]} passes {args.passes} '
                            f'critical {",".join(CRITICAL_NETS) or "none"} kicad {pcbnew.Version()} freerouting-sha256 {jar} java {java}')
    best.SetProperties(props)
    assert pcbnew.SaveBoard(args.out, best, True), 'SaveBoard failed'   # aSkipSettings: keep the project's net classes
    print('wrote', args.out, 'clean' if clean else '(NOT CLEAN, --force)')
    return 0 if clean else 1


DEFAULT_GRID = 1          # set-up trunk-guards ... (bay B6 / PB20: with J1's pins swapped and the corner re-zoned, full-guards (B5's
                          # set-up 0) left a starved B.Cu thermal at U2.2 after stage F; trunk-guards routes clean)
DEFAULT_ORDER = 'asc'     # ... wiring order asc: with CRITICAL_NETS, the variant that routes the committed board (B5 / PB18, B6 / PB20)
DEFAULT_PASSES = 80       # ... Freerouting max passes per run. ROUTED_WITH records all of these (ktest_route ties the committed board's
                          # record to them: route.py with no arguments reproduces it, B5 fix 1 / B6 fix 1)
MAX_PARALLEL = 1          # router processes at a time (one Freerouting JVM each): memory / swap on the routing machine


if __name__ == '__main__':
    sys.exit(main())

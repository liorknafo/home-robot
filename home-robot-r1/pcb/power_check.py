"""Bay B6 power-path check of the routed 4-layer board: IPC-2221 at <= 10 degC rise for the ordered copper (copper.py).

Run with KiCad's Python:  $KPY pcb/power_check.py [--board PCB] [--out FILE]   (prints the report table; --out also writes it)

The measurements are the B4 / B5 ones (zone_check), wrapped here under the B6 interfaces; nothing is measured twice:
 - trunk_capacity      = zone_check.trunk_capacity: per layer the true narrowest copper joining the net's POWER_TRUNK pins (PB11, contact
                         level: a pad of every pin, a fuse contact being one pin), the layers combined with copper.capacity_a;
 - pad_spoke_capacity  = zone_check.pin_spoke_capacity: what the pin's thermal spokes deliver into its net's trunk copper, as a max-flow
                         in which copper reached only through vias counts up to the via barrels (carry item 1, copper.via_current_a);
 - physical_check      the < 3 % rule of the carry list (items 2 / 3): where a trunk's conservative margin is thin, which pin the neck
                         isolates, so what current really passes it;
 - track_loads         per tap track / via: the summed TAP_LOAD_A of the pads it alone feeds, against its IPC-2221 capacity (item 4);
 - battery_gnd_ring_cut  ruling PB20 (b): GND is a plane, not a trunk - every GND trunk pin's pad metric (above) and, round the
                         battery GND pin, the copper leaving every ring out to the nearest GND load pin: pin_ring_cut, a lower bound
                         (per-layer min cuts from the pad edge, through the spokes, capped at them, B6 fix 1; rated on their combined
                         cross-section, LIO-232);
 - min_track_width, motor_vias (item 5), c3_ripple (item 6), drv8874_dissipation / drv8874_current_for_tj (item 7, PB20 e).
B4's 3.0 mm polygon floor (zone_check.ERODE, ktest_pcb / ktest_route) stays as a corridor rule; the per-net IPC capacity is the sizing rule
that supersedes it (B4 minor, carry list)."""
import argparse, heapq, math, sys
from pathlib import Path
import pcbnew
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, copper as C, zone_check as Z

PCB = Path(__file__).parent / 'r1_power_io.kicad_pcb'
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
TAP_NETS = B.TAP_NETS
SPOKE_MAX = 2.0            # mm: relief spokes no wider (R13 / brief: the pads stay iron-solderable)
LOW_MARGIN = 0.03          # carry items 2 / 3: a trunk under 3 % conservative margin ...
PHYSICAL_MARGIN = 0.20     # ... is accepted only with >= 20 % on the current its neck really carries
NECK_TOL = 0.0             # the physical rule judges necks with no reach tolerance (PB11: the true narrowest)
GND_EXTRA_PADS = ('J1', 'J3', 'U5A', 'U5B', 'U3P', 'U4P')     # brief: their GND pads join the power trunk pads


# --- the brief's interfaces ------------------------------------------------------------------------------------------------------------
def trunk_capacity(board, net, tol=Z.REACH_TOL):
    """{'width_mm': {layer: w}, 'capacity_a': A} of the net's trunk copper (zone_check.trunk_capacity, ruling PB11)."""
    return Z.trunk_capacity(board, net, tol)


def power_trunk_pads(board):
    """[(ref, pin, net)]: every POWER_TRUNK pin, plus the GND pins of J1, J3, U5A, U5B, U3P and U4P (in POWER_TRUNK['GND'] already)."""
    out = [(r, p, net) for net, pins in B.POWER_TRUNK.items() for r, p in pins]
    for ref in GND_EXTRA_PADS:
        for p in board.FindFootprintByReference(ref).Pads():
            if p.GetNetname() == 'GND' and (ref, p.GetNumber(), 'GND') not in out: out.append((ref, p.GetNumber(), 'GND'))
    return out


def pad_spokes(board, ref, pad):
    """{layer: (live spokes, spoke width mm, A)} - the raw spokes per layer (zone_check.pin_spokes)."""
    return Z.pin_spokes(board, ref, pad)


def pad_spoke_capacity(board, ref, pad):
    """A the pin's spokes deliver into its trunk copper, via barrels counted (zone_check.pin_spoke_capacity)."""
    return Z.pin_spoke_capacity(board, ref, pad)[0]


def pocket_vias(board, ref, pad):
    """Drills (mm) of the vias that join the pin's non-trunk copper (patches, pockets, helper pours) to its trunk."""
    seen = {}
    for _, _, held in Z.pin_path_capacity(board, ref, pad)[1]['via_pieces']:
        for u, v, d in held: seen[(u, v)] = d
    return list(seen.values())


def pad_uses_spokes(board, ref, pad):
    """True when every zone joint of the pin - each of its pads, on each layer its net's fill reaches the pad - is a thermal relief:
    the zone's pad connection (or the pad's own override) is THERMAL, the spokes are <= SPOKE_MAX wide and the fill covers well under
    the whole relief ring (a solid joint fills it). False if the pin meets no zone at all."""
    pads = [p for p in board.FindFootprintByReference(ref).Pads() if p.GetNumber() == pad]
    net, found = pads[0].GetNetname(), False
    for z in [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net]:
        gap = TOMM(z.GetThermalReliefGap())
        for name in C.STACKUP:
            lid = Z.LAYER[name]
            if not z.IsOnLayer(lid): continue
            fill = Z.solid_fill(z, lid)
            for p in pads:
                if not p.IsOnLayer(lid): continue
                ring = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ring, lid, MM(gap - 0.05), MM(0.005), pcbnew.ERROR_INSIDE)
                inner = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(inner, lid, MM(0.05), MM(0.005), pcbnew.ERROR_INSIDE)
                ring.BooleanSubtract(inner)
                full = Z.area_mm2(ring)
                ring.BooleanIntersection(fill)
                if not Z._spokes_into(p, lid, fill, gap, TOMM(z.GetThermalReliefSpokeWidth())): continue    # B12a (carry 10): a joint is fill that reaches the pad (B6 fix 1), not fill that only crosses the ring
                found = True
                conn = p.GetLocalZoneConnection()
                if conn == pcbnew.ZONE_CONNECTION_INHERITED: conn = z.GetPadConnection()
                if conn != pcbnew.ZONE_CONNECTION_THERMAL: return False
                if TOMM(z.GetThermalReliefSpokeWidth()) > SPOKE_MAX + 1e-9: return False
                if Z.area_mm2(ring) > 0.9 * full: return False
    return found


def min_track_width(board, net):
    """The narrowest track segment of the net, mm (inf if it has none)."""
    return min((TOMM(t.GetWidth()) for t in board.GetTracks() if t.GetClass() != 'PCB_VIA' and t.GetNetname() == net), default=math.inf)


def motor_vias(board):
    """[(net, (u, v))] of every via on a Motor-class net (carry item 5: the motor pairs run with none)."""
    nets = set(B.NETCLASSES['Motor']['nets'])
    return [(v.GetNetname(), (round(TOMM(v.GetPosition().x), 3), round(TOMM(v.GetPosition().y), 3)))
            for v in board.GetTracks() if v.GetClass() == 'PCB_VIA' and v.GetNetname() in nets]


# --- carry items 2 / 3: the physical current through a thin neck ------------------------------------------------------------------------
def tap_load_total(net):
    """A of every tap load on the net (board_spec.TAP_LOAD_A)."""
    return sum(a for p, a in B.TAP_LOAD_A.items() if p in B.NETS[net])


def physical_check(board, net, tol=NECK_TOL):
    """The trunk at zero reach tolerance and what its neck carries. conservative_margin = capacity / CURRENT_A - 1. If dropping one
    POWER_TRUNK pin from the net raises its capacity, the neck lies on that pin's own access (isolated_pin): it carries that pin's own
    load (pad_current) plus, to be safe, every tap load of the net; rest_capacity_a is the trunk without that pin, which must still carry
    the full CURRENT_A. Otherwise the neck carries the whole net current."""
    pins = B.POWER_TRUNK[net]
    full = Z.trunk_capacity(board, net, tol)
    cap, need = full['capacity_a'], B.CURRENT_A[net]
    isolated, rest = None, full
    if len(pins) > 2:
        for p in pins:
            r = Z.trunk_capacity(board, net, tol, pins=[q for q in pins if q != p])
            if r['capacity_a'] > cap + 1e-6: isolated, rest = p, r
    neck = B.pad_current(*isolated) + tap_load_total(net) if isolated else need
    why = (f"dropping {isolated[0]}.{isolated[1]} widens {full['width_mm']} -> {rest['width_mm']}: the neck feeds "
           f"{isolated[0]}.{isolated[1]} alone ({B.pad_current(*isolated):.3f} A) + the net's taps ({tap_load_total(net) * 1e3:.1f} mA)"
           if isolated else 'the neck carries the whole net current')
    return dict(net=net, need_a=need, width_mm=full['width_mm'], capacity_a=cap, conservative_margin=cap / need - 1,
                isolated_pin=isolated, neck_current_a=neck, physical_margin=cap / neck - 1, rest_width_mm=rest['width_mm'],
                rest_capacity_a=rest['capacity_a'], derivation=why)


# --- carry item 4: the load behind each tap track and via --------------------------------------------------------------------------------
def _shape(item, lid, grow=0.01):
    ps = pcbnew.SHAPE_POLY_SET(); item.TransformShapeToPolygon(ps, lid, MM(grow), MM(0.005), pcbnew.ERROR_OUTSIDE)
    return ps


def _r3(p): return (round(TOMM(p.x), 3), round(TOMM(p.y), 3))


def track_loads(board, net):
    """[dict(kind, layer, size, a, b, key, load_a, cap_a, feeds)] for every track segment and via of the tap net: load_a = the summed
    TAP_LOAD_A of the pads cut off from the net's source when that item is removed (the source: the main pieces of its zones on its
    joining layers; 3V3: TAP_SOURCE). An item on a loop (no pad cut off) is charged every load in its part of the net beyond the source
    (conservative). cap_a = IPC-2221 at 10 degC on its layer's copper (a via: its barrel)."""
    lids = [(L, Z.LAYER[L]) for L in C.STACKUP]
    nodes = []                                                    # (kind, item / key, {lid: polygon})
    joining = Z.joining_layers(board, net) if net in B.POWER_TRUNK else ()
    sources = set()
    for z in [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net]:
        for name, lid in lids:
            if not z.IsOnLayer(lid): continue
            pcs = Z.pieces(Z.solid_fill(z, lid), min_area=0.0)
            main = max(pcs, key=lambda one: one.Area()) if pcs else None
            for pc in pcs:
                if name in joining and pc is main: sources.add(len(nodes))
                nodes.append(('piece', name, {lid: pc}))
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() != net: continue
            if (fp.GetReference(), p.GetNumber()) == B.TAP_SOURCE.get(net): sources.add(len(nodes))
            nodes.append(('pad', (fp.GetReference(), p.GetNumber()), {lid: _shape(p, lid) for _, lid in lids if p.IsOnLayer(lid)}))
    items = []
    for t in board.GetTracks():
        if t.GetNetname() != net: continue
        items.append(len(nodes))
        nodes.append(('via' if t.GetClass() == 'PCB_VIA' else 'track', t, {lid: _shape(t, lid) for _, lid in lids if t.IsOnLayer(lid)}))
    boxes = [{lid: ps.BBox() for lid, ps in n[2].items()} for n in nodes]
    adj = [set() for _ in nodes]
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if nodes[i][0] == 'piece' and nodes[j][0] == 'piece': continue
            for lid in set(nodes[i][2]) & set(nodes[j][2]):
                if not boxes[i][lid].Intersects(boxes[j][lid]): continue
                x = nodes[i][2][lid].CloneDropTriangulation(); x.BooleanIntersection(nodes[j][2][lid])
                if x.OutlineCount(): adj[i].add(j); adj[j].add(i); break
    loads = {i: B.TAP_LOAD_A.get(n[1], 0.0) for i, n in enumerate(nodes) if n[0] == 'pad'}
    def reach(start, banned):
        seen, todo = set(start), list(start)
        while todo:
            for k in adj[todo.pop()]:
                if k not in seen and k not in banned: seen.add(k); todo.append(k)
        return seen
    fed = reach(sources, set())
    out = []
    for i in items:
        kind, t, _ = nodes[i]
        cut = [k for k in loads if k in fed and k not in reach(sources - {i}, {i})]
        if not cut and i in fed:                              # on a loop: everything in its part of the net past the source
            cut = [k for k in reach({i}, sources) if k in loads]
        if kind == 'via':
            size, cap, layer = round(TOMM(t.GetDrillValue()), 3), C.via_current_a(TOMM(t.GetDrillValue())), 'via'
            a = b = _r3(t.GetPosition()); key = None
        else:
            layer, size = t.GetLayerName(), round(TOMM(t.GetWidth()), 3)
            cap = C.ipc2221_current_a(size, C.COPPER_OZ[layer], internal=C.is_internal(layer))
            a, b = _r3(t.GetStart()), _r3(t.GetEnd())
            key = (net, layer, size, frozenset({a, b}))
        out.append(dict(net=net, kind=kind, layer=layer, size=size, a=a, b=b, key=key, load_a=sum(loads[k] for k in cut), cap_a=cap,
                        feeds=sorted(f'{nodes[k][1][0]}.{nodes[k][1][1]}' for k in cut if loads[k] > 0)))
    return out


# --- carry item 6: C3's ripple share ------------------------------------------------------------------------------------------------------
def _farads(ref): return float(B.PARTS[ref]['value'].split('uF')[0]) * 1e-6


def c3_ripple(i_motor=None, f=None, slow_decay=True):
    """DRV_L's PWM input ripple and C3's share. Slow decay (the firmware's PWM, BRINGUP-NOTES: IN1 = 1, IN2 = inverted PWM, so the off-time
    brakes on the low sides): the bridge draws the motor current I for the on-time and nothing for the off-time, so its supply sees an AC
    current of I x sqrt(D (1 - D)) rms, worst at D = 0.5 (I / 2). Fast decay (IN2 low, PWM on IN1: the off-time coasts, LIO-227): the
    motor current flows back into the supply for the off-time, +I then -I, so 2 I x sqrt(D (1 - D)), I at D = 0.5. C3 (at U3P.1) and C1
    (through C3_C1_PATH of VMOT copper) split it by their impedances at the PWM frequency: I3 = I_ac x |Z1| / |Z1 + Z3|. The DC of the
    motor comes down the trunk."""
    i_motor = B.CURRENT_A['L_M1'] if i_motor is None else i_motor
    f = B.DRV_PWM_HZ if f is None else f
    w = 2 * math.pi * f
    i_ac = i_motor * (0.5 if slow_decay else 1.0)
    z3 = complex(B.C3_ESR_OHM, -1 / (w * _farads('C3')))
    z1 = complex(B.C1_ESR_OHM + B.C3_C1_PATH['r_ohm'], w * B.C3_C1_PATH['l_h'] - 1 / (w * _farads('C1')))
    return dict(f_hz=f, slow_decay=slow_decay, i_motor_a=i_motor, i_ac_a=i_ac, c3_a=i_ac * abs(z1) / abs(z1 + z3), c1_a=i_ac * abs(z3) / abs(z1 + z3),
                c3_alone_a=i_ac, rating_a=B.C3_RIPPLE_RATING_A, via_a=C.via_current_a(B.SMD_PAD_VIA[1]))


# --- carry item 7: DRV8874 dissipation (report only) -------------------------------------------------------------------------------------
# TI DRV8874 (SLVSF66) figures quoted from memory - verify against the datasheet before relying on them: R_DS(on) high side + low side
# 200 mOhm typical at T_J 25 degC, rising about 1.5x by 150 degC; thermal shutdown about 165 degC; HTSSOP-16 R_thetaJA about 40 degC/W on a
# JEDEC 4-layer board. The Pololu 4035 carrier (15 x 18 mm, 2 layers) has no published R_thetaJA: 60-80 degC/W is assumed for it.
DRV_RDSON_25C, DRV_RDSON_150C_FACTOR, DRV_TSD_C = 0.200, 1.5, 165.0
DRV_THETA_JA = {'JEDEC 4-layer': 40.0, 'Pololu carrier (assumed)': 70.0}
DRV_EDGE_S = 0.2e-6        # output rise / fall time assumed for the switching loss
DRV_IQ_A = 0.003           # VM supply current while enabled (order of magnitude)
T_AMBIENT_C = 40.0         # air under the Jetson carrier, assumed (BRINGUP measures)


def _drv_tj(i, theta, v=B.VBAT_MAX, f=None, t_amb=T_AMBIENT_C):
    """(W, T_J degC) of one driver at motor current i and R_thetaJA theta: conduction I^2 R_DS(on) (HS + LS, both on during the on-time and
    the slow-decay brake recirculation the firmware must use, c3_ripple) with R_DS(on) rising linearly to x DRV_RDSON_150C_FACTOR at 150 degC, switching
    1/2 V I (t_r + t_f) f, quiescent V I_q; T_J solved by fixed-point iteration."""
    f = B.DRV_PWM_HZ if f is None else f
    alpha = (DRV_RDSON_150C_FACTOR - 1) / 125.0
    p_sw, p_q = 0.5 * v * i * 2 * DRV_EDGE_S * f, v * DRV_IQ_A
    tj = t_amb
    for _ in range(60):
        p = i * i * DRV_RDSON_25C * (1 + alpha * (tj - 25)) + p_sw + p_q
        tj = t_amb + theta * p
    return p, tj


def drv8874_dissipation(i=2.1, v=B.VBAT_MAX, f=None, t_amb=T_AMBIENT_C):
    """Per driver at motor current i: the loss terms at 25 degC and, per R_thetaJA of DRV_THETA_JA, (W, T_J) from _drv_tj."""
    f = B.DRV_PWM_HZ if f is None else f
    out = dict(i_a=i, p_cond_25c_w=i * i * DRV_RDSON_25C, p_sw_w=0.5 * v * i * 2 * DRV_EDGE_S * f, p_q_w=v * DRV_IQ_A, tj={})
    for name, theta in DRV_THETA_JA.items():
        p, tj = _drv_tj(i, theta, v, f, t_amb)
        out['tj'][name] = (round(p, 3), round(tj, 1))
    return out


# --- the GND plane: the copper leaving a pin, as per-layer min cuts on a raster ---------------------------------------------------------
def net_copper(board, net, layer):
    """The net's copper on `layer` as one polygon set: every zone fill of the net there (the GND patches are several zones) plus its pads,
    tracks and vias (zone_check.gnd_copper, for any net and any number of zones)."""
    lid = Z.LAYER[layer]
    ps = pcbnew.SHAPE_POLY_SET()
    for z in board.Zones():
        if not z.GetIsRuleArea() and z.GetNetname() == net and z.IsOnLayer(lid): ps.BooleanAdd(Z.solid_fill(z, lid))
    for it in [p for fp in board.GetFootprints() for p in fp.Pads()] + list(board.GetTracks()):
        if it.GetNetname() == net and it.IsOnLayer(lid): it.TransformShapeToPolygon(ps, lid, 0, MM(0.005), pcbnew.ERROR_INSIDE)
    ps.Simplify()
    return ps


RING_GRID = 0.1            # mm: raster pitch of the ring cuts (B6 fix 1: 0.2 -> 0.1 with whole-cell sampling, copper_raster)


def copper_raster(board, layer, grid=RING_GRID, net='GND', west=2.0, whole=True):
    """(cells, W, H, X0, Y0, grid) - the net's copper on `layer` (net_copper) rastered on a grid (zone_check.raster) with a free border
    round it, so a cut can leave the board: 2 mm, and `west` mm on the -u side (where raster_cut_mm's ray runs: it must reach past any disc
    that pokes over the board edge); None if the net has no copper there. One raster serves every cut on that layer.
    whole (B6 fix 1): a cell is copper only when it lies wholly in copper - the copper is shrunk by half a cell diagonal before the
    cell-centre test - so a cut straight across a neck w wide crosses at most w / grid copper cells and the raster never rounds a neck up
    (plain cell-centre sampling can add a cell per crossing)."""
    fill = net_copper(board, net, layer)
    if whole and fill.OutlineCount(): fill.Deflate(MM(grid / math.sqrt(2)), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.005))
    if not fill.OutlineCount(): return None
    cells, nx, ny, x0, y0 = Z.raster(fill, grid)
    pad, pw = int(2 / grid), int(max(2.0, west) / grid)
    W, H = nx + pad + pw, ny + 2 * pad
    cu = bytearray(W * H)
    for j in range(ny): cu[(j + pad) * W + pw:(j + pad) * W + pw + nx] = cells[j * nx:(j + 1) * nx]
    return cu, W, H, x0 - pw * grid, y0 - pad * grid, grid


def raster_cut_mm(r, centre, radius, away, outer=None):
    """mm of copper on the cheapest closed cut round the disc (centre, radius) mm that leaves the point `away` outside, on a copper_raster
    r: the min cut. A shortest path (crossing copper costs its length, voids and the off-board area are free) from the upper to the lower
    bank of a ray drawn from the disc to the raster's -u edge; a second ray from `away` to the +u edge keeps the cut from enclosing it.
    outer (mm): the cut must also stay within that radius of the centre - a ring cut, which cannot go round a far pad instead (a load pin
    that hangs on its own spokes is cheaper to cut off than the source). 0.0 when no copper leaves the disc (a pocket)."""
    if r is None: return 0.0
    cu, W, H, X0, Y0, grid = r
    cx, cy = centre
    blocked = bytearray(W * H)
    for j in range(H):
        y = Y0 + (j + 0.5) * grid
        dy2, sink_row = (y - cy) ** 2, abs(y - away[1]) < grid
        if dy2 > radius ** 2 and not sink_row and (outer is None or dy2 <= outer ** 2): continue
        for i in range(W):
            x = X0 + (i + 0.5) * grid
            d2 = (x - cx) ** 2 + dy2
            if d2 <= radius ** 2 or (sink_row and x >= away[0]) or (outer is not None and d2 > outer ** 2): blocked[j * W + i] = 1
    jr, ir = int((cy - Y0) / grid), int((cx - X0) / grid)     # the ray runs between rows jr - 1 and jr, columns 0 .. ir
    dist, heap = {}, []
    for i in range(ir + 1):
        k = (jr - 1) * W + i
        if not blocked[k]:
            d = grid if cu[k] else 0.0
            dist[k] = d; heapq.heappush(heap, (d, k))
    s2 = math.sqrt(2)
    while heap:
        d, k = heapq.heappop(heap)
        if d > dist.get(k, math.inf): continue
        i, j = k % W, k // W
        if j == jr and i <= ir: return round(d, 3)
        for di, dj, wgt in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, s2), (1, -1, s2), (-1, 1, s2), (-1, -1, s2)):
            a, b = i + di, j + dj
            if not (0 <= a < W and 0 <= b < H): continue
            n = b * W + a
            if blocked[n]: continue
            if {j, b} == {jr - 1, jr} and min(a, i) <= ir: continue          # never across the ray
            nd = d + (wgt * grid if cu[n] else 0.0)
            if nd < dist.get(n, math.inf): dist[n] = nd; heapq.heappush(heap, (nd, n))
    return math.inf


# GND source pins for the plane report: (pin, the current it brings in or out)
GND_SOURCES = [(B.J1_GND_PIN, B.CURRENT_A['GND']), (('J3', '2'), B.P03_OUT_A), (('U5B', '2'), B.P03_OUT_A), (('U5A', '2'), B.P03_IN_A),
               (('U3P', '2'), 2.1), (('U4P', '2'), 2.1), (('C1', '2'), 2.1)]
GND_CUT_LAYERS = ('F.Cu', 'In1.Cu', 'B.Cu')     # the layers with GND copper (In2 has none)
PAD_EDGE = 0.05            # mm past the pad's half size: the ring cuts' disc holds the pad alone, inside the relief gap, short of every spoke


def _pad(board, ref, pin): return [q for q in board.FindFootprintByReference(ref).Pads() if q.GetNumber() == pin][0]
def _uv(p): return (TOMM(p.GetPosition().x), TOMM(p.GetPosition().y))
def _half(p): return TOMM(max(p.GetSize().x, p.GetSize().y)) / 2


def pin_ring_cut(board, ref, pin, outer, away, grid=RING_GRID, net='GND', layers=GND_CUT_LAYERS):
    """The copper leaving the pin ref.pin out to `outer` mm from its centre, as a lower bound (B6 fix 1). Per layer: the min cut
    (raster_cut_mm, on copper_raster's whole cells) of the net's copper round a disc that holds the pad alone - radius r0 = the pad's
    half size + PAD_EDGE, inside the relief gap, so the cut may run through the pin's relief spokes - confined to the ring out to `outer`
    (so it cannot go round the far pin `away` instead), at IPC-2221 for the layer's copper; capped at the layer's own spokes
    (zone_check.pin_spokes: n x spoke width at their drawn width, so raster rounding can never credit more than the spokes); the layers'
    counted widths rated together on their combined cross-section (copper.capacity_a, LIO-231 / LIO-232: summing each layer's own IPC
    current credited every layer the cooling of a lone track - 8.99 A for the 6.71 A J1.1's copper carried as a stack).

    Why it is a lower bound: any cut of the joined copper (the layers plus every via between them), restricted to one layer, still
    separates that layer's pad copper from the copper beyond `outer` on that layer alone, so it costs at least that layer's own min cut;
    the vias only add paths. The per-layer min cuts together are therefore at most what the joined layers carry. This needs a disc that
    holds the pad alone: a disc that swallows the relief (B6's r0 = pad + gap + 0.25 mm) takes copper the pin reaches only through its
    spokes, or through vias from another layer, as the pin's own - an upper bound of that layer's share, not a lower one (In1 at J1.1:
    3.46 A credited against 2.26 A of spokes). A layer the pin has no spoke on counts 0 here (its via-joined copper counts in the pad
    metric, zone_check.pin_path_capacity).

    One cut covers every ring: with `outer` fixed, a larger disc only removes closed curves, so the min cut never falls as the disc grows
    (the cut profile is monotone in r and r0 binds); the pad-edge disc gives the least copper across any closed curve between the pad and
    `outer` - every radius at once, so there is no sweep over radii.
    Returns dict(pin, centre, r0, r1, per={layer: dict(cut_mm, cut_a, spokes=(n, w), spoke_a, a, w)}, total_a): a is the layer alone, w
    its counted width (the cut, capped at n x w), total_a the stack's rating."""
    p = _pad(board, ref, pin)
    c, r0 = _uv(p), _half(p) + PAD_EDGE
    gap = min(TOMM(z.GetThermalReliefGap()) for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net)
    assert r0 < TOMM(min(p.GetSize().x, p.GetSize().y)) / 2 + gap, f'{ref}.{pin}: a disc of {r0:.2f} mm reaches past the relief gap'
    spokes = Z.pin_spokes(board, ref, pin)
    per = {}
    for L in layers:
        mm = raster_cut_mm(copper_raster(board, L, grid, net, west=outer - c[0] + 3.0), c, r0, away, outer)
        cut_a = C.ipc2221_current_a(mm, C.COPPER_OZ[L], internal=C.is_internal(L)) if 0 < mm < math.inf else 0.0
        n, w, spoke_a = spokes.get(L, (0, 0.0, 0.0))
        per[L] = dict(cut_mm=mm, cut_a=cut_a, spokes=(n, w), spoke_a=spoke_a, a=min(cut_a, spoke_a),
                      w=min(mm if 0 < mm < math.inf else 0.0, n * w))
    return dict(pin=(ref, pin), centre=c, r0=round(r0, 3), r1=round(outer, 3), per=per, total_a=C.capacity_a({L: x['w'] for L, x in per.items()}))


def nearest_gnd_load(board, ref, pin):
    """(pad, r1 mm) - the GND trunk pin nearest ref.pin (the other POWER_TRUNK GND pins) and the ring radius just short of it: its
    distance less its half size, the relief gap and 0.25 mm (PB20: the battery pin's rings run out to the nearest GND load pin)."""
    c = _uv(_pad(board, ref, pin))
    gap = TOMM(Z.zone_of(board, 'GND', C.GND_PLANE_LAYER).GetThermalReliefGap())
    near = min((_pad(board, r, n) for r, n in B.POWER_TRUNK['GND'] if (r, n) != (ref, pin)), key=lambda p: math.dist(_uv(p), c))
    return near, math.dist(_uv(near), c) - _half(near) - gap - 0.25


def gnd_cut_capacity(board, ref, pin):
    """({layer: (cut mm, A)}, the stack's A, nearest GND trunk pin) - the GND copper leaving the pin (report only), measured as the
    battery pin's: pin_ring_cut out to just short of its nearest GND trunk pin (nearest_gnd_load) - per layer the min cut from the pad
    edge, capped at the layer's spokes, the layers rated together: a lower bound. B6 fix 1: the disc used to reach 2 mm past the relief
    to take in the pin's patch vias (copper the pin reaches only through those vias counted as its own, not a lower bound) and the ring
    ran out to 2 mm short of a far pin, where a neck behind dozens of vias set the figure; a patch reached only through vias counts in the
    pad metric (section 3)."""
    near, r1 = nearest_gnd_load(board, ref, pin)
    r = pin_ring_cut(board, ref, pin, max(_half(_pad(board, ref, pin)) + PAD_EDGE + 1.0, r1), _uv(near))
    return ({L: (x['cut_mm'], x['a']) for L, x in r['per'].items()}, r['total_a'],
            f'{near.GetParentFootprint().GetReference()}.{near.GetNumber()}')


def battery_gnd_ring_cut(board, grid=RING_GRID):
    """Ruling PB20 (b), the second GND plane check (beside every GND trunk pin's pad metric): the copper leaving the battery GND pin
    (B.J1_GND_PIN, J1.1 since PB20) across every ring from its pad edge out to just short of the nearest GND load pin (the other
    POWER_TRUNK GND pins; r1 = its distance less its half size, the relief gap and 0.25 mm) - pin_ring_cut's lower bound: per layer the
    min cut from the pad edge (through the relief spokes) to r1, capped at the layer's spokes, the layers rated together on their combined
    cross-section (copper.capacity_a, LIO-232). B6 fix 1: B6 started the
    disc outside the relief (r0 = pad + gap + 0.25 mm), so no layer's cut could pass through its spokes and the figure was not a lower bound.
    Returns pin_ring_cut's dict plus direct (the layers the pin reaches through its own spokes) and away (the nearest load pin)."""
    ref, pin = B.J1_GND_PIN
    near, r1 = nearest_gnd_load(board, ref, pin)
    out = pin_ring_cut(board, ref, pin, r1, _uv(near), grid)
    out['direct'] = tuple(L for L, (n, _, _) in Z.pin_spokes(board, ref, pin).items() if n)
    out['away'] = f'{near.GetParentFootprint().GetReference()}.{near.GetNumber()}'
    return out


def drv8874_current_for_tj(tj_max, theta, v=B.VBAT_MAX, f=None, t_amb=T_AMBIENT_C):
    """The continuous motor current (A) at which drv8874_dissipation's model reaches T_J = tj_max for R_thetaJA theta (bisection)."""
    lo, hi = 0.0, 6.0
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _drv_tj(mid, theta, v, f, t_amb)[1] <= tj_max else (lo, mid)
    return lo


# --- the report ---------------------------------------------------------------------------------------------------------------------------
def report(board):
    L = []
    def row(*cols): L.append('  '.join(str(c) for c in cols))
    pct = lambda x: f'{x * 100:+.1f} %'
    row('# R1 bay board - B6 power-path check (IPC-2221, 10 degC rise; copper', ', '.join(f'{k} {v:g} oz' for k, v in C.COPPER_OZ.items()),
        f'; via barrel plating {C.VIA_PLATING_MM * 1e3:g} um)')
    props = dict(board.GetProperties())
    row('# board:', {str(k): v for k, v in props.items()}.get('ROUTED_WITH', '?'))
    row('')
    row('## 1. Trunks (PB11: true narrowest copper joining every POWER_TRUNK pin, layers in parallel)')
    row(f'{"net":8} {"need A":>6}  {"width per layer, reach tol 0.02 mm":40} {"cap A":>6} {"margin":>8}  |  {"tol 0: width":40} {"cap A":>6} {"margin":>8}')
    for net in (n for n in B.POWER_TRUNK if n != 'GND'):
        a, z = Z.trunk_capacity(board, net), Z.trunk_capacity(board, net, 0.0)
        need = B.CURRENT_A[net]
        row(f'{net:8} {need:6.2f}  {str(a["width_mm"]):40} {a["capacity_a"]:6.2f} {pct(a["capacity_a"] / need - 1):>8}  |  '
            f'{str(z["width_mm"]):40} {z["capacity_a"]:6.2f} {pct(z["capacity_a"] / need - 1):>8}')
    row('')
    row(f'## 2. Thin trunks (conservative margin < {LOW_MARGIN:.0%} at tol 0): accepted only with >= {PHYSICAL_MARGIN:.0%} on the neck\'s real current')
    for net in (n for n in B.POWER_TRUNK if n != 'GND'):
        r = physical_check(board, net)
        if r['conservative_margin'] >= LOW_MARGIN: continue
        row(f'{net}: {r["capacity_a"]:.3f} A vs {r["need_a"]} A ({pct(r["conservative_margin"])}); {r["derivation"]}; neck current '
            f'{r["neck_current_a"]:.3f} A -> physical margin {pct(r["physical_margin"])}; the rest of the trunk {r["rest_width_mm"]} = '
            f'{r["rest_capacity_a"]:.3f} A vs {r["need_a"]} A')
    row('')
    row('## 3. Power trunk pads (PB12: own load; spokes per layer; copper reached through vias counts up to the barrels)')
    row(f'{"pad":7} {"net":8} {"need A":>6}  {"spokes per layer (n x w -> A)":58} {"through vias (spokes A / vias)":34} {"cap A":>6} {"margin":>8} relief')
    for ref, pin, net in power_trunk_pads(board):
        per = pad_spokes(board, ref, pin)
        cap, det = Z.pin_path_capacity(board, ref, pin)
        need = B.pad_current(ref, pin)
        sp = ', '.join(f'{Lr.split(".")[0]} {n}x{w:g}={a:.2f}' for Lr, (n, w, a) in per.items() if n)
        vp = '; '.join(f'{Lr.split(".")[0]} {a:.2f} A / {len(h)}x{(C.via_current_a(h[0][2]) if h else 0):.2f}' for Lr, a, h in det['via_pieces'])
        row(f'{ref + "." + pin:7} {net:8} {need:6.2f}  {sp:58} {vp or "-":34} {cap:6.2f} {pct(cap / need - 1):>8} '
            f'{"yes" if pad_uses_spokes(board, ref, pin) else "NO"}')
    row('')
    row('## 4. Motor tracks (carry item 5)')
    need_w = C.ipc2221_width_mm(B.CURRENT_A['L_M1'], C.COPPER_OZ['F.Cu'])
    for net in B.NETCLASSES['Motor']['nets']:
        lay = sorted({t.GetLayerName() for t in board.GetTracks() if t.GetClass() != 'PCB_VIA' and t.GetNetname() == net})
        w = min_track_width(board, net)
        row(f'{net}: min width {w:.3f} mm on {lay} vs IPC {need_w:.3f} mm for {B.CURRENT_A[net]} A on 1 oz '
            f'({C.ipc2221_current_a(w, 1.0):.2f} A); vias {[v for v in motor_vias(board) if v[0] == net]}')
    row(f'(a layer change would need 3 x {C.via_current_a(B.CLASS_VIA["Motor"][1]):.2f} A Motor vias)')
    row('')
    row('## 5. Tap tracks and vias (carry item 4): load behind each item vs its capacity')
    row(f'{"net":7} {"kind":5} {"layer":7} {"w/drill":>7} {"from":>18} {"to":>18} {"load mA":>8} {"cap A":>6}  feeds')
    for net in TAP_NETS:
        items = track_loads(board, net)
        for it in sorted(items, key=lambda it: (-it['load_a'] / it['cap_a'], it['kind'], it['a'])):
            row(f'{net:7} {it["kind"]:5} {it["layer"]:7} {it["size"]:7.3f} {str(it["a"]):>18} {str(it["b"]):>18} '
                f'{it["load_a"] * 1e3:8.2f} {it["cap_a"]:6.3f}  {",".join(it["feeds"])}')
        worst = max(items, key=lambda it: it['load_a'] / it['cap_a'])
        row(f'  {net}: {len(items)} items, worst load / capacity {worst["load_a"] / worst["cap_a"]:.3f} ({worst["kind"]} {worst["layer"]} '
            f'{worst["size"]} mm, {worst["load_a"] * 1e3:.1f} mA of {worst["cap_a"]:.3f} A)')
    row('')
    row('## 6. GND - a plane, not a trunk (ruling PB20 b): pad metric per pin (section 3) + the battery pin\'s ring cut')
    r = battery_gnd_ring_cut(board)
    row(f'Battery GND pin {r["pin"][0]}.{r["pin"][1]} (J1 pinout {B.J1_PINOUT}), direct layers {list(r["direct"])}: the copper leaving '
        f'the pad across every ring from its edge (disc r0 {r["r0"]} mm, inside the relief gap: the cut may run through the spokes) out '
        f'to r1 {r["r1"]} mm (short of {r["away"]}); per layer the min cut on a {RING_GRID:g} mm whole-cell raster, capped at the '
        f'layer\'s spokes, the layers rated together on their combined cross-section (copper.capacity_a) - a lower bound (pin_ring_cut; '
        f'one cut covers every radius):')
    for lay, x in r['per'].items():
        row(f'  {lay:7}: min cut {x["cut_mm"]:5.2f} mm -> {x["cut_a"]:5.2f} A; spokes {x["spokes"][0]} x {x["spokes"][1]:g} mm -> '
            f'{x["spoke_a"]:5.2f} A; counts {x["w"]:5.2f} mm ({x["a"]:5.2f} A alone)')
    row(f'  stack {r["total_a"]:.2f} A vs {B.CURRENT_A["GND"]} A ({pct(r["total_a"] / B.CURRENT_A["GND"] - 1)}); the layers\' own currents '
        f'summed {sum(x["a"] for x in r["per"].values()):.2f} A')
    g = Z.trunk_capacity(board, 'GND')
    row(f'(the brief\'s single-stream GND trunk metric, superseded by PB20: {g["width_mm"]} -> {g["capacity_a"]:.2f} A)')
    row('  GND copper leaving each source pin, out to its nearest GND trunk pin (pin_ring_cut as above: per-layer min cut from the pad '
        'edge, capped at the spokes, rated together - a lower bound that ignores every via; report only)')
    for (ref, pin), amps in GND_SOURCES:
        per, tot, near = gnd_cut_capacity(board, ref, pin)
        row(f'  {ref}.{pin} (carries {amps:.2f} A, ring to {near}): ' + ', '.join(f'{L.split(".")[0]} {mm:.2f} mm {a:.2f} A'
            for L, (mm, a) in per.items()) + f' -> {tot:.2f} A ({pct(tot / amps - 1)})' + ('  PAD_PENDING' if (ref, pin) in B.PAD_PENDING else '')
            + ('  (vias ignored: a lower bound; the pad metric, section 3, counts them)' if tot < amps else ''))     # B12a (carry 10)
    row('')
    row('## 7. C3 ripple (carry item 6)')
    for f, slow in ((B.DRV_PWM_HZ, True), (B.DRV_PWM_HZ, False), (1e3, True), (100e3, True)):
        r = c3_ripple(f=f, slow_decay=slow)
        row(f'  PWM {f / 1e3:g} kHz {"slow" if slow else "fast (not allowed)"} decay: DRV_L input ripple {r["i_ac_a"]:.3f} A rms (D = 0.5, '
            f'{r["i_motor_a"]} A); C3 {r["c3_a"]:.3f} A, C1 '
            f'{r["c1_a"]:.3f} A; C3 rating {r["rating_a"]} A (its 100 kHz figure); C3 GND via {r["via_a"]:.2f} A; C3 alone (C1 open) {r["c3_alone_a"]:.3f} A')
    row(f'  C3 height {B.PARTS["C3"]["h"]} +/- {B.PART_H_TOL["C3"]} mm vs MODULE_CLEARANCE {B.MODULE_CLEARANCE} mm (for B9)')
    row('')
    row('## 8. DRV8874 dissipation per driver (carry item 7, report only; datasheet figures from memory - verify)')
    for i in (2.1, 1.1):
        d = drv8874_dissipation(i)
        row(f'  {i} A: conduction {d["p_cond_25c_w"]:.3f} W at 25 degC, switching {d["p_sw_w"]:.3f} W, quiescent {d["p_q_w"]:.3f} W; '
            + '; '.join(f'{k}: {p:.2f} W, T_J {t:.0f} degC' for k, (p, t) in d['tj'].items()) + f' (ambient {T_AMBIENT_C:g}, TSD ~{DRV_TSD_C:g})')
    row('  continuous current for T_J <= 110 degC (B10 bring-up limit, firmware via CS 1.1 V/A or VREF): '
        + '; '.join(f'{k}: {drv8874_current_for_tj(110.0, th):.2f} A' for k, th in DRV_THETA_JA.items()))
    return '\n'.join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--board', default=str(PCB))
    ap.add_argument('--out', help='also write the table here')
    args = ap.parse_args()
    text = report(pcbnew.LoadBoard(args.board))
    print(text)
    if args.out: Path(args.out).write_bytes((text + '\n').encode())
    return 0


if __name__ == '__main__':
    sys.exit(main())

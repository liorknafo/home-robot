"""Zone-copper checks shared by the KiCad tests (ktest_pcb on the placed board, ktest_route on the routed board) and the B6 power check.
Run inside KiCad's Python (pcbnew). Board frame (u, v) = KiCad (x, y) in mm. Bay B4: four copper layers (copper.STACKUP); layers are
named by their STACKUP names ('F.Cu', 'In1.Cu', ...) and LAYER maps a name to its pcbnew id.

R4 / R13 trunk check (`trunk_problems`): for each POWER_TRUNK net, on each layer it is a trunk on (`trunk_layers`: the net's
POWER_ZONES trunk entries; GND: GND_TRUNK_LAYERS), take that layer's zone fill, unfracture it (KiCad stores fills fractured: holes joined
to the outline by zero-width cuts; eroding the raw list would split the copper along every cut), erode it by 1.5 mm and drop specks under
0.05 mm2 (offset round-off). A power zone must erode to exactly ONE piece; the GND plane also leaves slivers between 2.54 mm pin rows, so
for GND the largest eroded piece is the trunk. Every pad of every POWER_TRUNK pin must have >= 2 relief spokes (fill crossing the
relief-gap ring that reaches the pad, _spokes_into) on that layer and its centre within pad radius + relief gap + 1.5 + 0.5 mm of that piece. A pin with several pads
(fuse-holder contact: 4 pads) must meet this on every pad. A trunk net with no trunk layer, or a missing zone, is an error.

GND plane integrity (`gnd_pads_off_main_piece`): every GND pad is joined to the largest piece of the (un-eroded) GND fill on the plane
layer (default In1.Cu) through GND copper (fill pieces, pads, tracks, vias) on any layer, so no pad hangs on a GND island.
`gnd_pads_on_islands` is the stricter form: the pad's own copper on that layer must touch that piece.

Return paths (`gnd_path_lengths`, ruling R15): the shortest path between two GND pads through the plane layer's GND copper (plane fill + GND
pads), on a 0.25 mm grid - what a plane slot does to the return current under a signal or power loop. Compared with the same board
holding only its locked pre-routes (the placement baseline). `in1_voids`: the plane's voids (slots) and their lengths (R15: <= 10 mm)."""
import heapq, math
import pcbnew
from pcb import board_spec as B, copper as C

MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
LAYER = {L: getattr(pcbnew, L.replace('.', '_')) for L in C.STACKUP}      # 'F.Cu' -> pcbnew.F_Cu, 'In1.Cu' -> pcbnew.In1_Cu, ...
KEEP_LAYERS = C.STACKUP                # hole_intrusions looks at every copper layer
GND_TRUNK_LAYERS = (C.GND_PLANE_LAYER,)   # the In1 plane carries the GND trunk; the B.Cu GND fill is split into islands by the B.Cu power
                                          # zones (5V, BAT+, F1_OUT), so it is extra copper (stitched by vias in B5), not a trunk
ERODE = 1.5          # mm: half of the 3 mm copper corridor every trunk must keep (ruling R4)
SLACK = 0.5          # mm: allowance between the relief ring and the eroded corridor
MIN_PIECE = 0.05     # mm2: smaller eroded pieces are offset round-off, not copper


def area_mm2(ps):
    return ps.Area() / 1e12 if ps.OutlineCount() else 0.0


def layer_id(layer):
    """pcbnew layer id of a STACKUP name (ids pass through)."""
    return LAYER[layer] if isinstance(layer, str) else layer


def zone_of(board, net, layer):
    """The one copper zone of `net` on `layer` (STACKUP name or id); LookupError if there is none or more than one."""
    zs = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.GetLayer() == layer_id(layer)]
    if len(zs) != 1: raise LookupError(f'{len(zs)} {net} zones on {board.GetLayerName(layer_id(layer))}, want exactly one')
    return zs[0]


def trunk_layers(net):
    """The layers (STACKUP names, in stack order) a POWER_TRUNK net is a trunk on: GND -> GND_TRUNK_LAYERS, any other net -> its
    POWER_ZONES trunk entries (board_spec.zone_layers). Empty for a net without a zone (trunk_report reports it)."""
    return GND_TRUNK_LAYERS if net == 'GND' else B.zone_layers(net)


def solid_fill(zone, layer):
    """The zone's fill on `layer` as real polygons with holes (unfractured copy)."""
    fill = zone.GetFilledPolysList(layer).CloneDropTriangulation()
    fill.Unfracture()
    return fill


def pieces(ps, min_area=MIN_PIECE):
    """Split a polygon set into one SHAPE_POLY_SET per outline (with its holes), dropping specks under min_area mm2."""
    out = []
    for k in range(ps.OutlineCount()):
        one = pcbnew.SHAPE_POLY_SET(); one.AddOutline(ps.Outline(k))
        for h in range(ps.HoleCount(k)): one.AddHole(ps.Hole(k, h))
        if area_mm2(one) >= min_area: out.append(one)
    return out


def eroded_pieces(fill, erode=ERODE):
    er = fill.CloneDropTriangulation(); er.Deflate(MM(erode), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.005))
    return pieces(er)


def dist_mm(ps, pos):
    return 0.0 if ps.Contains(pos) else TOMM(int(math.sqrt(ps.SquaredDistance(pos))))


def trunk_report(board):
    """(net, layer name) -> dict(pieces=n eroded pieces, main_area=mm2, pads=[(ref, num, spokes, dist, limit)], problems=[str]) for every
    POWER_TRUNK net on every layer of trunk_layers(net); a net with no trunk layer gets (net, None) with a 'no trunk zone' problem, and a
    trunk layer without its zone on the board raises LookupError (zone_of)."""
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    rep = {}
    for net, pins in B.POWER_TRUNK.items():
        if not trunk_layers(net):
            rep[(net, None)] = dict(pieces=0, main_area=0.0, pads=[], problems=[f'{net}: no trunk zone on any layer'])
            continue
        for name in trunk_layers(net):
            layer = LAYER[name]
            zone = zone_of(board, net, layer)
            gap = TOMM(zone.GetThermalReliefGap())
            fill = solid_fill(zone, layer)
            pcs = eroded_pieces(fill)
            r = rep[(net, name)] = dict(pieces=len(pcs), main_area=0.0, pads=[], problems=[])
            if not pcs:
                r['problems'].append(f'{net} {name}: no zone copper survives {ERODE} mm erosion'); continue
            if net != 'GND' and len(pcs) != 1:
                r['problems'].append(f'{net} {name}: eroded zone copper is {len(pcs)} pieces')
            main = max(pcs, key=lambda one: one.Area())
            r['main_area'] = area_mm2(main)
            for ref, num, p in [(ref, num, p) for ref, num in pins for p in fps[ref].Pads() if p.GetNumber() == num]:
                rad = TOMM(max(p.GetSize().x, p.GetSize().y)) / 2
                spokes = _spokes_into(p, layer, fill, gap, TOMM(zone.GetThermalReliefSpokeWidth()))   # B12a (carry 10): real spokes (B6 fix 1), not fill that only crosses the ring
                d, limit = dist_mm(main, p.GetPosition()), rad + gap + ERODE + SLACK
                r['pads'].append((ref, num, spokes, round(d, 2), round(limit, 2)))
                if spokes < 2: r['problems'].append(f'{net} {name}: {ref}.{num} has {spokes} relief spokes')
                if d > limit: r['problems'].append(f'{net} {name}: {ref}.{num} is {d:.2f} mm from the main eroded zone copper (limit {limit:.2f})')
    return rep


# --- IPC-2221 capacities (B4 fix 1, rulings PB11 / PB12; reused by B6) ----------------------------------------------------------------
REACH_TOL = 0.02     # mm: polygon round-off allowed when eroded copper "reaches" a pad (no slack: the pad's relief ring itself)


def live_pieces(board, zone, layer):
    """Pieces of the zone's (unfractured) fill on `layer` that carry current onward: the largest piece, and any piece holding a via of the
    zone's net (a stitched island). A piece that is neither is a dead island (it joins nothing but the pad it surrounds)."""
    lid = layer_id(layer)
    pcs = pieces(solid_fill(zone, lid), min_area=0.0)
    if not pcs: return []
    main = max(pcs, key=lambda one: one.Area())
    vias = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == 'PCB_VIA' and v.GetNetname() == zone.GetNetname()]
    return [pc for pc in pcs if pc is main or any(pc.Contains(c) for c in vias)]


def pin_spokes(board, ref, pin):
    """{layer name: (live spokes, spoke width mm, A)} of a pin - every pad numbered `pin` of footprint `ref` - into its net's zones on each
    copper layer (all of them: the F.Cu GND patches are one zone each): a spoke is a piece of live zone copper (live_pieces) crossing the
    pad's relief ring, at least half a spoke wide, that reaches the pad (_spokes_into); A is copper.ipc2221_current_a(spokes x width) at
    the layer's copper weight (internal for In1 / In2)."""
    fp = board.FindFootprintByReference(ref)
    pads = [p for p in fp.Pads() if p.GetNumber() == pin]
    net = pads[0].GetNetname()
    out = {}
    for name in C.STACKUP:
        lid = LAYER[name]
        zs = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.GetLayer() == lid]
        if not zs or not any(p.IsOnLayer(lid) for p in pads): continue
        n, w = 0, min(TOMM(z.GetThermalReliefSpokeWidth()) for z in zs)       # several zones of one net on a layer: the GND patches
        for zone in zs:
            gap, zw = TOMM(zone.GetThermalReliefGap()), TOMM(zone.GetThermalReliefSpokeWidth())
            live = pcbnew.SHAPE_POLY_SET()
            for pc in live_pieces(board, zone, lid): live.BooleanAdd(pc)
            n += sum(_spokes_into(p, lid, live, gap, zw) for p in pads if p.IsOnLayer(lid))
        out[name] = (n, w, C.ipc2221_current_a(n * w, C.COPPER_OZ[name], internal=C.is_internal(name)) if n else 0.0)
    return out


def pin_spoke_capacity(board, ref, pin):
    """(A, pin_spokes) - the current the pin's thermal spokes deliver into its net's trunk copper (ruling PB12), with copper that reaches
    the trunk only through vias counted only up to their barrels (bay B6, carry item 1): pin_path_capacity's max-flow. pin_spokes gives the
    raw per-layer spokes for the report."""
    return pin_path_capacity(board, ref, pin)[0], pin_spokes(board, ref, pin)


def _spokes_into(pad, lid, copper, gap, zw):
    """Thermal spokes of `pad` on layer lid into the polygon set `copper`: fill crossing the relief-gap ring (0.05 mm off the pad to
    0.05 mm short of the gap), at least half a spoke, that reaches the pad - the copper next to that piece comes within 0.01 mm of the
    pad. B6 fix 1: fill that only crosses the ring - a zone outline ending inside the relief, short of the pad, where KiCad draws no
    spoke (U3P.2's F.Cu patch stopped 0.05 mm off its pad) - is no spoke."""
    ring = pcbnew.SHAPE_POLY_SET(); pad.TransformShapeToPolygon(ring, lid, MM(gap - 0.05), MM(0.005), pcbnew.ERROR_INSIDE)
    inner = pcbnew.SHAPE_POLY_SET(); pad.TransformShapeToPolygon(inner, lid, MM(0.05), MM(0.005), pcbnew.ERROR_INSIDE)
    ring.BooleanSubtract(inner); ring.BooleanIntersection(copper)
    edge = pcbnew.SHAPE_POLY_SET(); pad.TransformShapeToPolygon(edge, lid, MM(0.01), MM(0.005), pcbnew.ERROR_OUTSIDE)
    n = 0
    for one in pieces(ring, min_area=0.0):
        if area_mm2(one) < 0.5 * zw * (gap - 0.1): continue
        reach = one.CloneDropTriangulation(); reach.Inflate(MM(0.07), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.005))
        reach.BooleanIntersection(copper); reach.BooleanIntersection(edge)            # the copper bridging the last 0.05 mm to the pad
        if reach.OutlineCount() and area_mm2(reach) > 1e-4: n += 1
    return n


def _max_flow(cap, s, t):
    """Edmonds-Karp on {u: {v: capacity}} (the dict is consumed as the residual graph)."""
    flow = 0.0
    while True:
        prev, queue = {s: None}, [s]
        while queue and t not in prev:
            u = queue.pop(0)
            for v, c in cap.get(u, {}).items():
                if c > 1e-12 and v not in prev: prev[v] = u; queue.append(v)
        if t not in prev: return flow
        path, v = [], t
        while prev[v] is not None: path.append((prev[v], v)); v = prev[v]
        f = min(cap[u][v] for u, v in path)
        for u, v in path:
            cap[u][v] -= f; cap.setdefault(v, {}); cap[v][u] = cap[v].get(u, 0.0) + f
        flow += f


def joining_layers(board, net):
    """Layers (STACKUP names) whose copper is the net's trunk: GND -> GND_TRUNK_LAYERS (the In1 plane); a power net -> its trunk layers
    plus every layer where a helper pour alone joins all its POWER_TRUNK pins (the F1_OUT In2 copy), as trunk_capacity counts."""
    if net == 'GND': return tuple(GND_TRUNK_LAYERS)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    pins = [[p for p in fps[r].Pads() if p.GetNumber() == n] for r, n in B.POWER_TRUNK[net]]
    out = set(trunk_layers(net))
    for name in C.STACKUP:
        zs = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.GetLayer() == LAYER[name]]
        if zs and name not in out and _joined(solid_fill(zs[0], LAYER[name]), pins, TOMM(zs[0].GetThermalReliefGap()), 0.02, REACH_TOL):
            out.add(name)
    return tuple(L for L in C.STACKUP if L in out)


def pin_path_capacity(board, ref, pin):
    """(A, detail) - the most current (IPC-2221, 10 degC) the pin `ref`.`pin` can push into its net's trunk copper through thermal spokes,
    as a max-flow (bay B6, carry item 1). Nodes: the pin (all its pads), every piece of every filled zone of the net on every copper
    layer, the net's vias, its other soldered through-hole pads, the plated M3 mounting holes. Edges: the pin's spokes into a piece (the
    spokes of all its pads into that piece as one width: copper.ipc2221_current_a(n x spoke width) at the layer's copper); another THT
    pad's spokes into a piece (its soldered lead joins its layers); a via joins every piece that holds its centre, and its barrel carries
    copper.via_current_a(drill) in all; a plated mounting hole (MOUNT_HOLES: H1-H4, nothing is soldered in them) joins the pieces its pads'
    spokes reach, its barrels - the M3 hole and the ring of small plated holes round it, one land on every layer - carrying the sum of
    their copper.via_current_a(drill) in all (B6 fix 1: they were counted as soldered leads, with no limit between layers). Sink: the
    largest piece of each of the net's zones on its joining layers (joining_layers: GND -> the In1 plane). So a pocket or patch that
    reaches the trunk only through vias counts only up to their barrels, and a dead-end pour (no via, no other pad) counts nothing. Tracks
    are left out (conservative). detail: {'direct': {layer: A}, 'via_pieces': [(layer, A of spokes, [(u, v, drill)])], 'holes': {ref:
    A of its barrels}}."""
    fp = board.FindFootprintByReference(ref)
    mine = [p for p in fp.Pads() if p.GetNumber() == pin]
    net = mine[0].GetNetname()
    joining = joining_layers(board, net)
    vias = [v for v in board.GetTracks() if v.GetClass() == 'PCB_VIA' and v.GetNetname() == net]
    others = [p for f in board.GetFootprints() if not f.IsDNP() and f.GetReference() not in MOUNT_HOLES for p in f.Pads()
              if p.GetNetname() == net and p.IsOnLayer(pcbnew.F_Cu) and p.IsOnLayer(pcbnew.B_Cu)
              and not (f.GetReference() == ref and p.GetNumber() == pin)]
    mounts = {f.GetReference(): [p for p in f.Pads() if p.GetNetname() == net and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
              for f in board.GetFootprints() if f.GetReference() in MOUNT_HOLES and f.GetReference() != ref}
    mounts = {h: ps for h, ps in mounts.items() if ps}
    cap, S, T = {}, ('pin',), ('trunk',)
    def edge(u, v, a):                                    # undirected: the same capacity both ways
        for x, y in ((u, v), (v, u)): cap.setdefault(x, {}); cap[x][y] = cap[x].get(y, 0.0) + a
    detail = {'direct': {}, 'via_pieces': []}
    k = 0
    for name in C.STACKUP:
        lid = LAYER[name]
        for zone in [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.IsOnLayer(lid)]:
            gap, zw = TOMM(zone.GetThermalReliefGap()), TOMM(zone.GetThermalReliefSpokeWidth())
            pcs = pieces(solid_fill(zone, lid), min_area=0.0)
            if not pcs: continue
            main = max(pcs, key=lambda one: one.Area())
            amps = lambda n: C.ipc2221_current_a(n * zw, C.COPPER_OZ[name], internal=C.is_internal(name))
            for pc in pcs:
                k += 1; node = ('piece', k)
                sink = name in joining and pc is main
                if sink: cap.setdefault(node, {})[T] = math.inf
                bb = pc.BBox(); bb.Inflate(MM(4))
                n = sum(_spokes_into(p, lid, pc, gap, zw) for p in mine if p.IsOnLayer(lid) and bb.Contains(p.GetPosition()))
                held = [v for v in vias if bb.Contains(v.GetPosition()) and pc.Contains(v.GetPosition())]
                if n:
                    edge(S, node, amps(n))
                    if sink: detail['direct'][name] = detail['direct'].get(name, 0.0) + amps(n)
                    else: detail['via_pieces'].append((name, round(amps(n), 3), [(round(TOMM(v.GetPosition().x), 3),
                                                       round(TOMM(v.GetPosition().y), 3), round(TOMM(v.GetDrillValue()), 3)) for v in held]))
                for v in held:
                    cap.setdefault(node, {})[('via', id(v), 'in')] = math.inf
                    cap.setdefault(('via', id(v), 'out'), {})[node] = math.inf
                for p in others:
                    if p.IsOnLayer(lid) and bb.Contains(p.GetPosition()):
                        m = _spokes_into(p, lid, pc, gap, zw)
                        if m: edge(('pad', id(p)), node, amps(m))
                for h, hp in mounts.items():                  # into / out of the hole's barrels through its spokes on this layer
                    m = sum(_spokes_into(p, lid, pc, gap, zw) for p in hp if p.IsOnLayer(lid) and bb.Contains(p.GetPosition()))
                    if m:                                         # (each piece is a node of its own: one edge each way)
                        cap.setdefault(node, {})[('hole', h, 'in')] = amps(m)
                        cap.setdefault(('hole', h, 'out'), {})[node] = amps(m)
    for v in vias:
        cap.setdefault(('via', id(v), 'in'), {})[('via', id(v), 'out')] = C.via_current_a(TOMM(v.GetDrillValue()))
    detail['holes'] = {}
    for h, hp in mounts.items():
        if ('hole', h, 'out') not in cap: continue                   # no spoke of the hole reaches a piece of the net
        detail['holes'][h] = sum(C.via_current_a(TOMM(p.GetDrillSize().x)) for p in hp)
        cap.setdefault(('hole', h, 'in'), {})[('hole', h, 'out')] = detail['holes'][h]
    return _max_flow(cap, S, T), detail


def via_copper_layers(board, net, uv):
    """STACKUP names of the layers where a through via at uv = (u, v) mm really ties into its net's zone copper (bay B6, carry item 8:
    ktest_pcb's stitching-via check): a filled zone piece of `net` holds the point and carries current somewhere - it is the net's trunk
    there (the largest piece of a zone of the net on one of its joining_layers: GND -> the In1 plane), or a pad of the net touches it (a
    patch / pocket / helper that feeds a pin). B6 fix 1: a zone's largest piece no longer counts on a layer that is not a joining layer
    (every single-piece F.Cu GND patch would count by itself), nor does a piece that only holds other vias (two orphan vias in one
    island would vouch for each other). The ring of fill KiCad leaves round a via inside another net's pour leads nowhere and does not
    count. Run on a filled board."""
    pos = pcbnew.VECTOR2I(MM(uv[0]), MM(uv[1]))
    pads = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetname() == net]
    joining = joining_layers(board, net)
    out = []
    for name in C.STACKUP:
        lid = LAYER[name]
        for z in [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.IsOnLayer(lid)]:
            pcs = pieces(solid_fill(z, lid), min_area=0.0)
            here = [pc for pc in pcs if pc.Contains(pos)]
            if not here: continue
            pc, bb = here[0], here[0].BBox()
            bb.Inflate(MM(3))                                   # a pad's centre may lie outside the piece its spokes reach
            def touches(p):
                if not (p.IsOnLayer(lid) and bb.Contains(p.GetPosition())): return False
                x = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(x, lid, MM(0.01), MM(0.005), pcbnew.ERROR_OUTSIDE)
                x.BooleanIntersection(pc)
                return x.OutlineCount() > 0
            if (name in joining and pc is max(pcs, key=lambda one: one.Area())) or any(touches(p) for p in pads):
                out.append(name); break
    return out


def _reaches(pcs, pads, w, gap, tol):
    return {k for k, pc in enumerate(pcs) for p in pads
            if dist_mm(pc, p.GetPosition()) <= TOMM(max(p.GetSize().x, p.GetSize().y)) / 2 + gap + w / 2 + tol}


def _joined(fill, pins, gap, w, tol):
    """The fill eroded by w/2 has one piece that reaches (see narrowest_width) a pad of every pin."""
    er = fill.CloneDropTriangulation(); er.Deflate(MM(w / 2), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.005))
    pcs = pieces(er)
    return bool(pcs) and bool(set.intersection(*[_reaches(pcs, pads, w, gap, tol) for pads in pins]))


def narrowest_width(board, zone, layer, pins, tol=REACH_TOL):
    """The zone's true narrowest copper on `layer` between the pins (ruling PB11, contact level): the largest w (mm) for which the fill
    eroded by w/2 still has one piece that comes within pad radius + relief gap + w/2 (+ tol, the polygon round-off) of at least one pad of
    every pin. pins: [[pad, ...] per pin]. 0.0 if the zone does not join the pins at all."""
    lid = layer_id(layer)
    fill, gap = solid_fill(zone, lid), TOMM(zone.GetThermalReliefGap())
    if not _joined(fill, pins, gap, 0.02, tol): return 0.0
    lo, hi = 0.02, 20.0
    for _ in range(18):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _joined(fill, pins, gap, mid, tol) else (lo, mid)
    return round(lo, 3)


def trunk_capacity(board, net, tol=REACH_TOL, pins=None):
    """{'width_mm': {layer: w}, 'capacity_a': A} - the net's copper between its POWER_TRUNK pins (or `pins`, [(ref, pin)], a subset: B6's
    neck legs): on every layer where a zone of the net (trunk or helper pour) alone joins the pins, its true narrowest width
    (narrowest_width); A = copper.capacity_a of those widths, i.e. the layers in parallel (ruling PB11)."""
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    pins = [[p for p in fps[r].Pads() if p.GetNumber() == n] for r, n in (B.POWER_TRUNK[net] if pins is None else pins)]
    widths = {}
    for name in C.STACKUP:
        zs = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == net and z.GetLayer() == LAYER[name]]
        if zs:
            w = narrowest_width(board, zs[0], name, pins, tol)
            if w > 0: widths[name] = w
    return {'width_mm': widths, 'capacity_a': round(C.capacity_a(widths), 3)}


def trunk_problems(board):
    return [p for r in trunk_report(board).values() for p in r['problems']]


def gnd_main_piece(board, layer=C.GND_PLANE_LAYER):
    """Largest piece of the GND fill on `layer` (not eroded) and its area in mm2."""
    pcs = pieces(solid_fill(zone_of(board, 'GND', layer), layer_id(layer)), min_area=0.0)
    if not pcs: return None, 0.0                      # unfilled
    main = max(pcs, key=lambda one: one.Area())
    return main, area_mm2(main)


def gnd_pads_off_main_piece(board, layer=C.GND_PLANE_LAYER):
    """GND pads (every one on the board) that are not joined to the largest GND fill piece on `layer` through GND copper: fill pieces of
    that layer, pads, tracks and vias, touching on a shared copper layer (a pad on a plane island counts only if GND copper ties it, or
    its island, to the main piece - ruling (c) allows a GND track where a pad would otherwise be isolated). Overlapping pads of one
    footprint (the M3 hole's ring and via pads) are one piece of copper."""
    lid = layer_id(layer)
    main, _ = gnd_main_piece(board, layer)
    layers = [LAYER[L] for L in C.STACKUP]
    nodes = []                                            # (kind, key, {layer: polygon})

    def shape(item, L):
        ps = pcbnew.SHAPE_POLY_SET(); item.TransformShapeToPolygon(ps, L, MM(0.01), MM(0.005), pcbnew.ERROR_OUTSIDE)
        return ps
    if main is not None:
        for k, pc in enumerate(pieces(solid_fill(zone_of(board, 'GND', lid), lid), min_area=0.0)):
            nodes.append(('fill', k, {lid: pc}))
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == 'GND':
                nodes.append(('pad', (fp.GetReference(), p.GetNumber()), {L: shape(p, L) for L in layers if p.IsOnLayer(L)}))
    for t in board.GetTracks():
        if t.GetNetname() == 'GND':
            nodes.append(('track', None, {L: shape(t, L) for L in layers if t.IsOnLayer(L)}))
    group = list(range(len(nodes)))
    def root(i):
        while group[i] != i: i = group[i]
        return i
    boxes = [{L: ps.BBox() for L, ps in n[2].items()} for n in nodes]
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if root(i) == root(j): continue
            for L in set(nodes[i][2]) & set(nodes[j][2]):
                if not boxes[i][L].Intersects(boxes[j][L]): continue
                x = nodes[i][2][L].CloneDropTriangulation(); x.BooleanIntersection(nodes[j][2][L])
                if x.OutlineCount(): group[root(i)] = root(j); break
    if main is None: return [n[1] for n in nodes if n[0] == 'pad']
    main_i = max((i for i, n in enumerate(nodes) if n[0] == 'fill'), key=lambda i: nodes[i][2][lid].Area())
    return [n[1] for i, n in enumerate(nodes) if n[0] == 'pad' and root(i) != root(main_i)]


def gnd_pads_on_islands(board, layer=C.GND_PLANE_LAYER):
    """GND pads on `layer` whose own copper there does not touch the main GND piece of that layer (they sit on a plane island;
    gnd_pads_off_main_piece tells whether other GND copper ties them to the main piece). Overlapping pads of one footprint count as one
    piece of copper."""
    lid = layer_id(layer)
    main, _ = gnd_main_piece(board, layer)
    bad = []
    for fp in board.GetFootprints():
        shapes = []
        for p in fp.Pads():
            if p.GetNetname() != 'GND' or not p.IsOnLayer(lid): continue
            sh = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(sh, lid, MM(0.01), MM(0.005), pcbnew.ERROR_OUTSIDE)
            shapes.append((p.GetNumber(), sh))
        group = list(range(len(shapes)))
        def root(i):
            while group[i] != i: i = group[i]
            return i
        for i in range(len(shapes)):
            for j in range(i + 1, len(shapes)):
                x = shapes[i][1].CloneDropTriangulation(); x.BooleanIntersection(shapes[j][1])
                if x.OutlineCount(): group[root(i)] = root(j)
        touch = set()
        for i, (_, sh) in enumerate(shapes):
            if main is None: continue
            x = sh.CloneDropTriangulation(); x.BooleanIntersection(main)
            if x.OutlineCount(): touch.add(root(i))
        bad += [(fp.GetReference(), num) for i, (num, _) in enumerate(shapes) if root(i) not in touch]
    return bad


def _hull(pts):
    """Convex hull (Andrew's monotone chain) of [(x, y)], counter-clockwise."""
    pts = sorted(set(pts))
    if len(pts) < 3: return pts
    def half(seq):
        h = []
        for q in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0]) <= 0: h.pop()
            h.append(q)
        return h
    lo, hi = half(pts), half(pts[::-1])
    return lo[:-1] + hi[:-1]


def slot_length(pts):
    """Length of a void: the longer side of its minimum-area bounding rectangle (rotating calipers over the convex hull's edge
    directions) - a straight slot's length, a round hole's diameter."""
    h = _hull(pts)
    if len(h) < 3: return max((math.dist(a, b) for a in h for b in h), default=0.0)
    best = None
    for (x0, y0), (x1, y1) in zip(h, h[1:] + h[:1]):
        L = math.hypot(x1 - x0, y1 - y0)
        if L == 0: continue
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        a = [x * ux + y * uy for x, y in h]; b = [-x * uy + y * ux for x, y in h]
        w, t = max(a) - min(a), max(b) - min(b)
        if best is None or w * t < best[0]: best = (w * t, max(w, t))
    return best[1]


def _simplified(pts, tol=0.03):
    """The closed ring `pts` without near-duplicate and near-collinear vertices (each dropped vertex lies within tol mm of the chord
    joining its kept neighbours)."""
    out = []
    for q in pts:
        if out and math.dist(out[-1], q) < 1e-6: continue
        out.append(q)
    changed = True
    while changed and len(out) > 3:
        changed = False
        k = 0
        while k < len(out) and len(out) > 3:
            a, b, c = out[k - 1], out[k], out[(k + 1) % len(out)]
            L = math.dist(a, c)
            d = abs((c[0] - a[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (c[1] - a[1])) / L if L > 1e-9 else math.dist(a, b)
            if d < tol: out.pop(k); changed = True
            else: k += 1
    return out


def _inside(ring, x, y):
    """Point in the polygon `ring` (even-odd)."""
    inside = False
    for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0): inside = not inside
    return inside


def _on_boundary(ring, x, y, eps=1e-4):
    for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]):
        L2 = (x1 - x0) ** 2 + (y1 - y0) ** 2
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / L2))
        if math.hypot(x - x0 - t * (x1 - x0), y - y0 - t * (y1 - y0)) < eps: return True
    return False


def _visible(ring, i, j):
    """The segment from vertex i to vertex j stays inside the closed polygon `ring` (interior or boundary): split it at every point where
    it meets the boundary and test the middle of each piece."""
    (ax, ay), (bx, by) = ring[i], ring[j]
    dx, dy = bx - ax, by - ay
    ts = [0.0, 1.0]
    for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]):
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue                                           # parallel: the pieces' midpoints decide
        t = ((x0 - ax) * ey - (y0 - ay) * ex) / den
        u = ((x0 - ax) * dy - (y0 - ay) * dx) / den
        if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9: ts.append(min(1.0, max(0.0, t)))
    ts.sort()
    for t0, t1 in zip(ts, ts[1:]):
        if t1 - t0 < 1e-9: continue
        x, y = ax + dx * (t0 + t1) / 2, ay + dy * (t0 + t1) / 2
        if not (_inside(ring, x, y) or _on_boundary(ring, x, y)): return False
    return True


def void_length(pts):
    """Length of a void measured along its bends (hand-over 5, bay B5): its geodesic diameter - the longest of the shortest paths inside
    the void between two of its points. For a straight slot that is its length (its end-to-end diagonal), for a round hole its diameter,
    for an L- or U-shaped void the length round the bend. The geodesic diameter of a simple polygon is reached between two of its
    vertices, so it is the longest all-pairs shortest path over the visibility graph of the (simplified) outline's vertices. A void no
    longer than half its perimeter can be skipped by the caller: geodesic diameter <= half the perimeter."""
    ring = _simplified(pts)
    n = len(ring)
    if n < 3: return max((math.dist(a, b) for a in ring for b in ring), default=0.0)
    INF = math.inf
    d = [[0.0 if a == b else INF for b in range(n)] for a in range(n)]
    for a in range(n):
        for b in range(a + 1, n):
            if b == a + 1 or (a == 0 and b == n - 1) or _visible(ring, a, b):
                d[a][b] = d[b][a] = math.dist(ring[a], ring[b])
    for k in range(n):
        dk = d[k]
        for a in range(n):
            dak = d[a][k]
            if dak == INF: continue
            da = d[a]
            for b in range(n):
                if dak + dk[b] < da[b]: da[b] = dak + dk[b]
    return max(max(row) for row in d)


def in1_voids(board, layer=C.GND_PLANE_LAYER, rings=False):
    """The voids of the filled GND plane on `layer` (In1.Cu): every hole of the fill and every notch cut into it from the edge
    clearance - antipads round non-GND through-hole pads and NPTH holes, relief gaps, merged wherever a web between them is thinner than
    the zone's minimum width - as [(length mm, (u0, v0, u1, v1) bbox)], longest first. length = void_length of the void's outline, the
    length along its bends (bay B5, hand-over 5; slot_length, the longer side of the minimum-area bounding rectangle, under-reports a
    bent void), computed exactly only where it can matter: a void whose half perimeter is under 5 mm gets slot_length (both are then
    under 5 mm). Through vias leave antipads too, so this is run on the routed board as well. Voids that hold a mounting / clamp hole
    centre (H1-H6) or touch an outline cutout are left out (they are the holes themselves). rings: each entry also carries the void's
    outline [(u, v), ...] (route.py keeps the router's vias off the long voids)."""
    lid = layer_id(layer)
    fill = solid_fill(zone_of(board, 'GND', lid), lid)
    area = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(area)
    cutouts = pcbnew.SHAPE_POLY_SET()
    for k in range(area.OutlineCount()):
        for h in range(area.HoleCount(k)): cutouts.AddOutline(area.Hole(k, h))
    edge = max(TOMM(board.GetDesignSettings().m_CopperEdgeClearance), TOMM(zone_of(board, 'GND', lid).GetLocalClearance()))
    inner = area.CloneDropTriangulation()
    inner.Deflate(MM(edge + 0.05), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.005))     # the fill's own edge band left out
    inner.BooleanSubtract(fill)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    centres = [fps[h].GetPosition() for h in MOUNT_HOLES if h in fps]
    out = []
    for k in range(inner.OutlineCount()):
        ring = inner.Outline(k)
        one = pcbnew.SHAPE_POLY_SET(); one.AddOutline(ring)
        if area_mm2(one) < MIN_PIECE: continue
        if any(one.Contains(c) for c in centres): continue
        if cutouts.OutlineCount():
            x = one.CloneDropTriangulation(); x.BooleanIntersection(cutouts)
            if x.OutlineCount(): continue
        pts = [(TOMM(ring.CPoint(i).x), TOMM(ring.CPoint(i).y)) for i in range(ring.PointCount())]
        bb = one.BBox()
        half = sum(math.dist(a, b) for a, b in zip(pts, pts[1:] + pts[:1])) / 2
        length = slot_length(pts) if half < 5.0 else max(void_length(pts), slot_length(pts))
        out.append((round(length, 3), (TOMM(bb.GetX()), TOMM(bb.GetY()), TOMM(bb.GetRight()), TOMM(bb.GetBottom()))) + ((pts,) if rings else ()))
    return sorted(out, reverse=True)


# --- return paths through the GND plane (ruling R15) ------------------------------------------------------------------------------------
GRID = 0.25          # mm raster pitch for the return-path search


def gnd_copper(board, layer=C.GND_PLANE_LAYER):
    """GND copper on `layer` as one polygon set: the plane's fill plus every GND pad's (and GND track's / via's) copper on that layer."""
    lid = layer_id(layer)
    ps = solid_fill(zone_of(board, 'GND', lid), lid)
    items = [p for fp in board.GetFootprints() for p in fp.Pads()] + list(board.GetTracks())
    for it in items:
        if it.GetNetname() == 'GND' and it.IsOnLayer(lid):
            it.TransformShapeToPolygon(ps, lid, 0, MM(0.005), pcbnew.ERROR_INSIDE)
    ps.Simplify()
    return ps


def raster(ps, grid=GRID):
    """Cells of a grid x grid raster over the board box whose centres lie in `ps` (even-odd over outlines and holes, by scanline).
    Returns (bytearray of nx * ny, nx, ny, x0, y0); cell (i, j) is index j * nx + i, centre (x0 + (i + .5) g, y0 + (j + .5) g) mm."""
    bb = ps.BBox()
    x0, y0 = TOMM(bb.GetX()) - grid, TOMM(bb.GetY()) - grid
    nx, ny = int(TOMM(bb.GetWidth()) / grid) + 3, int(TOMM(bb.GetHeight()) / grid) + 3
    rows = [[] for _ in range(ny)]
    rings = []
    for k in range(ps.OutlineCount()):
        rings.append(ps.Outline(k))
        rings += [ps.Hole(k, h) for h in range(ps.HoleCount(k))]
    for ring in rings:
        pts = [(TOMM(ring.CPoint(i).x), TOMM(ring.CPoint(i).y)) for i in range(ring.PointCount())]
        for (xa, ya), (xb, yb) in zip(pts, pts[1:] + pts[:1]):
            if ya == yb: continue
            lo, hi = min(ya, yb), max(ya, yb)
            j0 = max(0, math.ceil((lo - y0) / grid - 0.5)); j1 = min(ny - 1, math.ceil((hi - y0) / grid - 0.5) - 1)
            for j in range(j0, j1 + 1):                    # rows whose centre y is in [lo, hi)
                y = y0 + (j + 0.5) * grid
                rows[j].append(xa + (y - ya) * (xb - xa) / (yb - ya))
    cells = bytearray(nx * ny)
    for j, xs in enumerate(rows):
        xs.sort()
        for a, b in zip(xs[0::2], xs[1::2]):
            i0 = max(0, math.ceil((a - x0) / grid - 0.5)); i1 = min(nx - 1, math.ceil((b - x0) / grid - 0.5) - 1)
            if i1 >= i0: cells[j * nx + i0: j * nx + i1 + 1] = b'\x01' * (i1 - i0 + 1)
    return cells, nx, ny, x0, y0


def _cell_of(r, u, v, grid=GRID, reach=1.0):
    """Index of the copper cell nearest (u, v) mm, within `reach` mm."""
    cells, nx, ny, x0, y0 = r
    ci, cj = int((u - x0) / grid), int((v - y0) / grid)
    best = None
    n = int(reach / grid) + 1
    for dj in range(-n, n + 1):
        for di in range(-n, n + 1):
            i, j = ci + di, cj + dj
            if 0 <= i < nx and 0 <= j < ny and cells[j * nx + i] and (best is None or di * di + dj * dj < best[0]):
                best = (di * di + dj * dj, j * nx + i)
    if best is None: raise ValueError(f'no GND copper within {reach} mm of ({u:.2f}, {v:.2f})')
    return best[1]


def _shortest(r, src, targets, grid=GRID):
    """Dijkstra on the raster (8 neighbours, no corner cutting) from cell src; returns {target cell: mm}, inf if unreachable."""
    cells, nx, ny, _, _ = r
    dist = {src: 0.0}
    heap = [(0.0, src)]
    left = set(targets)
    out = {}
    d2 = math.sqrt(2)
    while heap and left:
        d, c = heapq.heappop(heap)
        if d > dist[c]: continue
        if c in left: left.discard(c); out[c] = d * grid
        i, j = c % nx, c // nx
        for di, dj, w in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, d2), (1, -1, d2), (-1, 1, d2), (-1, -1, d2)):
            a, b = i + di, j + dj
            if not (0 <= a < nx and 0 <= b < ny): continue
            n = b * nx + a
            if not cells[n]: continue
            if di and dj and not (cells[j * nx + a] and cells[b * nx + i]): continue
            nd = d + w
            if nd < dist.get(n, math.inf):
                dist[n] = nd; heapq.heappush(heap, (nd, n))
    return {t: out.get(t, math.inf) for t in targets}


def _path(r, src, tgt, grid=GRID):
    """Dijkstra as _shortest, one target, with the route: (mm, [(u, v) cell centres from src to tgt]); (inf, []) if unreachable."""
    cells, nx, ny, x0, y0 = r
    dist, prev, heap, d2 = {src: 0.0}, {}, [(0.0, src)], math.sqrt(2)
    while heap:
        d, c = heapq.heappop(heap)
        if c == tgt: break
        if d > dist[c]: continue
        i, j = c % nx, c // nx
        for di, dj, w in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, d2), (1, -1, d2), (-1, 1, d2), (-1, -1, d2)):
            a, b = i + di, j + dj
            if not (0 <= a < nx and 0 <= b < ny): continue
            n = b * nx + a
            if not cells[n] or (di and dj and not (cells[j * nx + a] and cells[b * nx + i])): continue
            if d + w < dist.get(n, math.inf): dist[n] = d + w; prev[n] = c; heapq.heappush(heap, (d + w, n))
    if tgt not in dist: return math.inf, []
    pts, c = [], tgt
    while True:
        pts.append((x0 + (c % nx + .5) * grid, y0 + (c // nx + .5) * grid))
        if c == src: break
        c = prev[c]
    return dist[tgt] * grid, pts[::-1]


def gnd_paths(board, pairs, grid=GRID, layer=C.GND_PLANE_LAYER):
    """{(pad a, pad b): (mm, [(u, v), ...])} - gnd_path_lengths with the shortest route itself (route.py keeps the router's copper off
    the placement baseline's power-loop routes)."""
    r = raster(gnd_copper(board, layer), grid)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    def cell(pad):
        (p,) = [q for q in fps[pad[0]].Pads() if q.GetNumber() == pad[1]][:1]
        return _cell_of(r, TOMM(p.GetPosition().x), TOMM(p.GetPosition().y), grid)
    return {(a, b): _path(r, cell(a), cell(b), grid) for a, b in pairs}


def gnd_path_lengths(board, pairs, grid=GRID, layer=C.GND_PLANE_LAYER):
    """{(pad a, pad b): mm} - shortest path between the two GND pads' centres through the GND copper on `layer` (gnd_copper) on a
    `grid` mm raster; pads are (ref, number). inf if the plane does not join them."""
    r = raster(gnd_copper(board, layer), grid)
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    def cell(pad):
        (p,) = [q for q in fps[pad[0]].Pads() if q.GetNumber() == pad[1]][:1]
        return _cell_of(r, TOMM(p.GetPosition().x), TOMM(p.GetPosition().y), grid)
    by_src = {}
    for a, b in pairs: by_src.setdefault(a, []).append(b)
    out = {}
    for a, bs in by_src.items():
        tc = {b: cell(b) for b in bs}
        got = _shortest(r, cell(a), set(tc.values()), grid)
        for b in bs: out[(a, b)] = got[tc[b]]
    return out


# Ruling R15 pad pairs: (ref, pin). Power loops may grow 5 % over the baseline, motor / encoder / IMU returns 20 %.
POWER_LOOPS = [(('U3P', '2'), ('C1', '2')), (('U3P', '2'), B.J1_GND_PIN), (('U4P', '2'), ('C1', '2')), (('U4P', '2'), B.J1_GND_PIN),
               (('U5A', '2'), B.J1_GND_PIN), (('U5B', '2'), B.J1_GND_PIN)]          # the battery GND pin: J1.1 since PB20
POWER_LOOP_MAX = 1.05
SIGNAL_RETURNS = [(('J4', '3'), ('U3P', '2')), (('J5', '3'), ('U4P', '2')),              # motor returns
                  (('J4', '3'), B.q(1, 'S')), (('J4', '3'), B.q(2, 'S')),                 # left encoder: ENC_LA / LB level shifters (source = GND)
                  (('J5', '3'), B.q(3, 'S')), (('J5', '3'), B.q(4, 'S')),                 # right encoder: ENC_RA / RB
                  (('U2', '2'), ('U1B', '8'))]                                           # IMU to Pico
SIGNAL_RETURN_MAX = 1.2


def return_paths(board, baseline, layer=C.GND_PLANE_LAYER):
    """[(pair, baseline mm, board mm, ratio, limit)] for POWER_LOOPS + SIGNAL_RETURNS through the GND copper on `layer`; both boards
    filled."""
    pairs = POWER_LOOPS + SIGNAL_RETURNS
    base, got = gnd_path_lengths(baseline, pairs, layer=layer), gnd_path_lengths(board, pairs, layer=layer)
    return [(p, base[p], got[p], got[p] / base[p], POWER_LOOP_MAX if p in POWER_LOOPS else SIGNAL_RETURN_MAX) for p in pairs]


def _poly(item, layer):
    ps = pcbnew.SHAPE_POLY_SET(); item.TransformShapeToPolygon(ps, layer, 0, MM(0.005), pcbnew.ERROR_OUTSIDE)
    return ps


def via_pad_gaps(board):
    """[(gap mm, via net, (u, v), 'REF.pin', pad net)] for every via: the copper gap to the nearest pad of another net (R17a), smallest
    first."""
    pads = [(fp.GetReference(), p) for fp in board.GetFootprints() for p in fp.Pads()]
    out = []
    for v in [t for t in board.GetTracks() if t.GetClass() == 'PCB_VIA']:
        c, rad = v.GetPosition(), TOMM(v.GetWidth(pcbnew.F_Cu)) / 2
        best = (math.inf, None, None)
        for ref, p in pads:
            if p.GetNetname() == v.GetNetname(): continue
            if abs(TOMM(p.GetPosition().x - c.x)) > 8 or abs(TOMM(p.GetPosition().y - c.y)) > 8: continue
            d = min(dist_mm(_poly(p, L), c) for L in (pcbnew.F_Cu, pcbnew.B_Cu) if p.IsOnLayer(L)) - rad
            if d < best[0]: best = (d, f'{ref}.{p.GetNumber()}', p.GetNetname())
        out.append((round(best[0], 3), v.GetNetname(), (round(TOMM(c.x), 2), round(TOMM(c.y), 2)), best[1], best[2]))
    return sorted(out, key=lambda x: x[0])


def gnd_copper_problems(board):
    """Ruling (c) / R17 on a board with top-side SMD GND pads: no GND track at all, and a GND via only where it joins a top-only (SMD)
    GND pad to the GND plane (its copper overlaps that pad) or is a scripted GND stitching via (board_spec.STITCH_VIAS, B4 fix 1).
    [(kind, (u, v))] of everything else that carries GND."""
    smd = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == 'GND' and p.IsOnLayer(pcbnew.F_Cu) and not p.IsOnLayer(pcbnew.B_Cu): smd.append(_poly(p, pcbnew.F_Cu))
    bad = []
    for t in board.GetTracks():
        if t.GetNetname() != 'GND': continue
        at = t.GetPosition()
        where = (round(TOMM(at.x), 2), round(TOMM(at.y), 2))
        if t.GetClass() != 'PCB_VIA': bad.append((t.GetClass(), where)); continue
        if (round(TOMM(at.x), 3), round(TOMM(at.y), 3)) in {(round(u, 3), round(v, 3)) for n, (u, v) in B.STITCH_VIAS if n == 'GND'}:
            continue                                      # B4 fix 1: a scripted GND stitching via (patch / B.Cu island to In1)
        via = _poly(t, pcbnew.F_Cu)
        def touches(pad):
            x = via.CloneDropTriangulation(); x.BooleanIntersection(pad)
            return x.OutlineCount() > 0
        if not any(touches(pad) for pad in smd): bad.append(('PCB_VIA', where))
    return bad


MOUNT_HOLES = ('H1', 'H2', 'H3', 'H4', 'H5', 'H6')
HOLE_KEEP = 4.5      # mm: no copper but GND within this radius of an M3 hole's centre (R17b: the washer)
CLAMP_HOLES = ('H5', 'H6')   # bay B3 / ruling PB3: the Ø3.0 NPTH clamp holes of the rear Jetson standoff stacks
CLAMP_KEEP = 3.5     # mm: no copper but GND within this radius of a clamp hole's centre (the nylon standoff and the printed spacer)


def hole_keep(h):
    """The no-copper-but-GND radius round mounting hole h."""
    return CLAMP_KEEP if h in CLAMP_HOLES else HOLE_KEEP


def hole_intrusions(board, keep=None):
    """Non-GND copper (tracks, vias, pads, zone fill) on any copper layer (KEEP_LAYERS: F.Cu, In1.Cu, In2.Cu, B.Cu) within the keep-out
    radius (hole_keep, or `keep` for every hole) of the mounting-hole centres H1-H6: [(hole, kind, net / pad, layer name)], one entry per
    item (the first layer it intrudes on)."""
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    layers = [(L, LAYER[L]) for L in KEEP_LAYERS]
    bad = []
    for h in MOUNT_HOLES:
        c, k = fps[h].GetPosition(), (keep if keep is not None else hole_keep(h))
        near = lambda ps, c=c, k=k: dist_mm(ps, c) < k - 1e-6
        def first(item):
            return next((name for name, L in layers if item.IsOnLayer(L) and near(_poly(item, L))), None)
        for t in board.GetTracks():
            if t.GetNetname() != 'GND' and first(t): bad.append((h, t.GetClass(), t.GetNetname(), first(t)))
        for ref, fp in fps.items():
            for p in fp.Pads():
                if ref != h and p.GetNetname() != 'GND' and first(p): bad.append((h, 'pad', f'{ref}.{p.GetNumber()}', first(p)))
        for z in board.Zones():
            if z.GetIsRuleArea() or z.GetNetname() == 'GND': continue
            for name, L in layers:
                if not z.IsOnLayer(L): continue
                f = z.GetFilledPolysList(L)
                if f.OutlineCount() and near(f): bad.append((h, 'zone', z.GetNetname(), name)); break
    return bad

"""Placed (unrouted) board from board_spec: footprints at PLACEMENT with their NETS nets, the Edge.Cuts outline and cutouts from
board_outline.json, the M3 holes and the two clamp holes, the plug-in modules' outlines on F.Fab, the under-carrier height zone drawn on
F.Fab, and the copper zones of the 4-layer board (bay B4, copper.STACKUP): the solid GND plane on In1.Cu, the POWER_ZONES on F.Cu / In2.Cu /
B.Cu and a GND fill on B.Cu under them, every zone pad joined by a wide thermal relief. The board carries the ordered stack-up (1.6 mm,
copper.COPPER_OZ). Zones are saved unfilled (route.py fills them after routing).

Run with KiCad's Python:  $KPY pcb/build_pcb.py [OUT]  (writes pcb/r1_power_io.kicad_pcb, or OUT - route.py's start board; never touches
                          r1_power_io.kicad_pro)

Silkscreen (L5 fix 1, spec 2026-10-06 section 3): one label per part, "<ref> <value>" (or a short function word for connectors and
sockets, board_spec.silk_label), 0.8 mm / 0.15 mm stroke, placed by place_silk_labels clear of pads, other silk, part bodies and the
edges and nearer its own part than any other; the reference and value fields move to F.Fab. Polarity marks are the stock footprints', plus
the "+" / "-" at J1's pins (board_spec.PIN_MARKS, ruling PB20: J1's pinout changed) and (B12a) the "+" at pin 1 of the P03 terminals
and of J3, and (LIO-242) a "VIN" text at the DRV8874 carriers' VIN pin (U3P.1, U4P.1).

Schematic parity (bay B7): every footprint keeps its library nickname (load_footprint) and every no-connect pin carries the schematic
netlist's single-pin 'unconnected-(...)' net (add_no_connect_nets), so kicad-cli DRC --schematic-parity reports nothing.

Module bodies (low-profile L5): a socket keeps its own library courtyard and the module it carries is only drawn on F.Fab, so the
reflowed SMD parts may sit under the Pico, the DRV8874 carriers and the GY-521 (spec 2026-10-06-r1-low-profile section 3: they are
soldered before the modules are plugged in). ktest_pcb checks that nothing but top-side SMD parts lies under a module body.

KiCad 9 notes: a project-less pcbnew.BOARD() saved with SaveBoard(path, board, aSkipSettings=True) leaves the .kicad_pro next to it
untouched; LoadBoard(path) then loads that project, so the net-class patterns written by make_schematic.py apply (ruling R12).
pcbnew.NewBoard(path) would instead rewrite the .kicad_pro at once."""
import json, sys, tempfile
from pathlib import Path
import pcbnew
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, copper as C, tools
from pcb.make_schematic import read_netlist

D = Path(__file__).parent
OUT = D / 'r1_power_io.kicad_pcb'
MM = pcbnew.FromMM
def V(u, v): return pcbnew.VECTOR2I(MM(u), MM(v))

# Zone settings (ruling R13): every pad joined to a zone - the In1 GND plane, the B.Cu GND fill and the power zones on F.Cu / In2.Cu /
# B.Cu - gets a thermal relief, never a solid connection: a solid joint into a plane makes the socket GND pins and the VH pads very hard to
# hand-solder and risks melting the socket plastic. KiCad draws 4 spokes (fewer only where a neighbouring pad blocks one) of THERMAL_SPOKE
# across a THERMAL_GAP ring on each layer the pad meets its zone on; B6 sums the spokes over the layers against the pad's current
# (IPC-2221, copper.py; 4 x 1.2 mm on 1 oz carries 7.4 A).
ZONE_CLEARANCE = 0.4          # mm, zone fill to other-net copper (KiCad uses max(this, the other net's class clearance): 0.4 for Power 0.4,
                              # PowerTap 0.3, Default 0.15 (PB17), and 1.0 round the Motor nets' copper, R16).
ZONE_MIN_WIDTH = 0.3          # mm, thinnest zone fill
IN1_CLEARANCE, IN1_MIN_WIDTH = 0.2, 0.2   # mm, the In1 GND plane: a small clearance and minimum width keep the webs between 2.54 mm pins
                              # (R15: no In1 slot over 10 mm; the other nets' class clearances still apply, so Motor pins keep 1.0 mm)
THERMAL_GAP, THERMAL_SPOKE = 0.5, 1.2

# Stack-up (bay B4): JLCPCB's 4-layer 1.6 mm default (JLC04161H-7628): 7628 prepreg between each outer layer and its inner layer, a core in
# the middle; copper copper.COPPER_OZ x copper.OZ_MM (1 oz outer, 0.5 oz inner - confirmed when ordering, B7). The core takes up the rest of
# PCB_T so the stack sums to 1.6 mm with KiCad's 0.01 mm masks.
PREPREG_MM, MASK_MM, DIELECTRIC = 0.2104, 0.01, ('FR4', 4.4, 0.02)


def stackup_items():
    """[(KiCad stack-up layer name, type, thickness mm)] top to bottom, copper layers in copper.STACKUP order."""
    cu = {L: round(C.OZ_MM * C.COPPER_OZ[L], 6) for L in C.STACKUP}
    core = round(B.PCB_T - 2 * MASK_MM - sum(cu.values()) - 2 * PREPREG_MM, 6)
    F, I1, I2, Bk = C.STACKUP
    return [('F.Mask', 'Top Solder Mask', MASK_MM), (F, 'copper', cu[F]), ('dielectric 1', 'prepreg', PREPREG_MM), (I1, 'copper', cu[I1]),
            ('dielectric 2', 'core', core), (I2, 'copper', cu[I2]), ('dielectric 3', 'prepreg', PREPREG_MM), (Bk, 'copper', cu[Bk]),
            ('B.Mask', 'Bottom Solder Mask', MASK_MM)]


def stackup_sexpr():
    out = ['\t\t(stackup\n']
    for name, kind, t in stackup_items():
        extra = f' (material "{DIELECTRIC[0]}") (epsilon_r {DIELECTRIC[1]}) (loss_tangent {DIELECTRIC[2]})' if kind in ('prepreg', 'core') else ''
        out.append(f'\t\t\t(layer "{name}" (type "{kind}") (thickness {t:g}){extra})\n')
    out.append('\t\t\t(copper_finish "HAL lead-free")\n\t\t\t(dielectric_constraints no)\n\t\t)\n')
    return ''.join(out)


def new_board():
    """A blank board with the four copper layers and the ordered stack-up in its BOARD_STACKUP. KiCad 9's Python binding has no
    BOARD_STACKUP class (BOARD_DESIGN_SETTINGS.GetStackupDescriptor returns an opaque pointer), so the stack-up goes in through KiCad's own
    board reader: a blank 4-layer board is saved, the (stackup ...) block is put into its (setup ...), and LoadBoard parses it into the
    BOARD_STACKUP, which SaveBoard writes back with the finished board."""
    blank = pcbnew.BOARD()
    blank.SetCopperLayerCount(len(C.STACKUP))
    blank.GetDesignSettings().SetBoardThickness(MM(B.PCB_T))
    with tempfile.TemporaryDirectory() as t:
        path = Path(t) / 'stackup.kicad_pcb'
        assert pcbnew.SaveBoard(str(path), blank, True)
        text = path.read_text(encoding='utf-8')
        assert text.count('\t(setup\n') == 1, 'unexpected board file layout'
        path.write_bytes(text.replace('\t(setup\n', '\t(setup\n' + stackup_sexpr(), 1).encode())
        board = pcbnew.LoadBoard(str(path))
    # bay B5 (hand-over 1): In1.Cu is the solid GND plane, layer type power - the Specctra export writes it (type power) and Freerouting
    # then never routes on it (route.py checks the DSN); F.Cu / In2.Cu / B.Cu stay signal layers
    board.SetLayerType(getattr(pcbnew, C.GND_PLANE_LAYER.replace('.', '_')), pcbnew.LT_POWER)
    assert board.GetCopperLayerCount() == len(C.STACKUP) and board.GetDesignSettings().m_HasStackup
    assert abs(pcbnew.ToMM(board.GetDesignSettings().GetBoardThickness()) - B.PCB_T) < 1e-6
    return board


def load_footprint(fp_id):
    """The library footprint, carrying its full library id (B7: FootprintLoad leaves the nickname empty, which the schematic parity
    check reports as a footprint mismatch on every part)."""
    lib, name = fp_id.split(':')
    path = B.CUSTOM_FP_DIR if lib == 'R1' else tools.FOOTPRINT_DIR / f'{lib}.pretty'
    fp = pcbnew.FootprintLoad(str(path), name)
    assert fp is not None, fp_id
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def add_no_connect_nets(board):
    """B7, schematic parity: every no-connect pin (board_spec.NC) gets the single-pin net KiCad's netlist names it, 'unconnected-(<ref>-<pin
    name>-Pad<n>)', read from the schematic's netlist (r1_power_io.net, make_schematic). Run last in build(): the nets are new board items,
    and adding them before the others would shift every seeded UUID (and so the router's DSN order)."""
    nc = {name: pins for name, pins in read_netlist(D / 'r1_power_io.net').items() if name.startswith('unconnected-(')}
    assert sorted(p for pins in nc.values() for p in pins) == sorted(B.NC), 'r1_power_io.net is stale: run pcb/make_schematic.py'
    for name, pins in sorted(nc.items()):
        ni = pcbnew.NETINFO_ITEM(board, name); board.Add(ni)
        (ref, pin), = pins
        for pad in board.FindFootprintByReference(ref).Pads():
            if pad.GetNumber() == pin: pad.SetNet(ni)


def add_body_outline(fp, x0, y0, x1, y1):
    """Module outline on F.Fab (documentation: shows where the plug-in module sits)."""
    r = pcbnew.PCB_SHAPE(fp); r.SetShape(pcbnew.SHAPE_T_RECT); r.SetLayer(pcbnew.F_Fab); r.SetWidth(MM(0.1))
    r.SetStart(V(x0, y0)); r.SetEnd(V(x1, y1)); fp.Add(r)


def module_bodies():
    """socket ref -> the whole module body (x0, y0, x1, y1) in that socket's own footprint frame (B.MODULES: body in the first socket's
    frame, rows at their pin-1 offsets)."""
    out = {}
    for name, m in B.MODULES.items():
        x0, y0, x1, y1 = m['body']
        for ref, (rx, ry) in m['rows'].items():
            out[ref] = (x0 - rx, y0 - ry, x1 - rx, y1 - ry)
    return out


def box_mm(bb, grow=0.0):
    return (pcbnew.ToMM(bb.GetX()) - grow, pcbnew.ToMM(bb.GetY()) - grow, pcbnew.ToMM(bb.GetRight()) + grow, pcbnew.ToMM(bb.GetBottom()) + grow)


def box_gap(a, b):
    """mm between two boxes (0 if they touch or overlap)."""
    du = max(a[0] - b[2], b[0] - a[2], 0.0); dv = max(a[1] - b[3], b[1] - a[3], 0.0)
    return (du * du + dv * dv) ** 0.5


def boxes_hit(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


SILK_OWN_MARGIN = 0.2                # a label is at least this much nearer its own part's courtyard than any other part's (no ambiguity)
STRICT_LABELS = True                # False: scratch builds report the labels that found no room instead of failing
SILK_PAD_GAP, SILK_SILK_GAP, SILK_BODY_GAP, SILK_EDGE_GAP = 0.25, 0.2, 0.15, 0.5   # mm, label box to: pads, other silk, part courtyards, edges


def place_silk_labels(board, fps, board_area):
    """One F.SilkS label per part (board_spec.silk_label), placed next to its part: the nearest of a ring of candidate spots (four sides,
    both reading directions, growing gaps) whose text box keeps SILK_PAD_GAP from every pad, SILK_SILK_GAP from every silk graphic and every
    label placed before it, SILK_BODY_GAP from every other part's courtyard and stays off its own body (its courtyard less 0.45 mm): a
    label is SILK_OWN_MARGIN nearer its own part than any other, so it cannot be read as another part's; a
    label under a part body is hidden once the part is fitted. Module bodies are not courtyards, so labels may sit under a module (readable
    at reflow, before the modules go in). SILK_EDGE_GAP inside the board edges and cutouts. The reference and value fields go to F.Fab.
    Deterministic: parts in a fixed order (SMD first, then by position). Returns the refs that found no room."""
    obstacles, own = [], {}                         # (box mm, owner ref, kind); own: a part's body box, for its own label
    for ref, fp in fps.items():
        for pad in fp.Pads(): obstacles.append((box_mm(pad.GetBoundingBox(), SILK_PAD_GAP), ref, 'pad'))
        for it in fp.GraphicalItems():
            if it.GetLayer() == pcbnew.F_SilkS and it.GetClass() == 'PCB_SHAPE':
                obstacles.append((box_mm(it.GetBoundingBox(), SILK_SILK_GAP), ref, 'silk'))
        obstacles.append((box_mm(fp.GetCourtyard(pcbnew.F_CrtYd).BBox(), SILK_BODY_GAP), ref, 'body'))
        own[ref] = box_mm(fp.GetCourtyard(pcbnew.F_CrtYd).BBox(), -0.45)
        fp.Reference().SetLayer(pcbnew.F_Fab); fp.Value().SetLayer(pcbnew.F_Fab)
    body_box = {ref: box_mm(fp.GetCourtyard(pcbnew.F_CrtYd).BBox()) for ref, fp in fps.items()}
    inner = board_area.CloneDropTriangulation()
    inner.Deflate(MM(SILK_EDGE_GAP), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
    def on_board(bx):
        r = pcbnew.SHAPE_POLY_SET(); r.NewOutline()
        for u, v in ((bx[0], bx[1]), (bx[2], bx[1]), (bx[2], bx[3]), (bx[0], bx[3])): r.Append(MM(u), MM(v))
        r.BooleanSubtract(inner)
        return r.OutlineCount() == 0 or r.Area() < 1e3
    def new_text(fp, ref):
        t = pcbnew.PCB_TEXT(fp); t.SetLayer(pcbnew.F_SilkS); t.SetText(B.silk_label(ref))
        t.SetTextSize(pcbnew.VECTOR2I(MM(B.SILK_TEXT_W), MM(B.SILK_TEXT_H))); t.SetTextThickness(MM(B.SILK_STROKE))
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER); t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
        t.SetTextAngleDegrees(0); t.SetPosition(V(0, 0))
        return t
    rel = {}                                        # ref -> {angle: glyph box relative to the text anchor} (the strokes, not the text cell)
    for ref in (r for r in fps if B.silk_label(r)):
        x0, y0, x1, y1 = box_mm(new_text(fps[ref], ref).GetEffectiveTextShape().BBox())
        rel[ref] = {0: (x0, y0, x1, y1), 90: (y0, -x1, y1, -x0)}           # angle 90 (CCW on screen): (x, y) -> (y, -x)
    size = {ref: (r[0][2] - r[0][0], r[0][3] - r[0][1]) for ref, r in rel.items()}
    def candidates(ref):
        cx0, cy0, cx1, cy1 = box_mm(fps[ref].GetCourtyard(pcbnew.F_CrtYd).BBox())
        cu, cv = (cx0 + cx1) / 2, (cy0 + cy1) / 2
        out = []
        for ang in (0, 90):
            lw, lh = size[ref] if ang == 0 else size[ref][::-1]
            for gap in (0.1, 0.4, 0.8, 1.3, 2.0, 3.0, 4.2, 5.6):
                for f in (0.0, -0.1, 0.1, -0.25, 0.25, -0.5, 0.5, -0.75, 0.75, -1.0, 1.0, -1.3, 1.3):
                    for (u, v) in ((cu + f * lw, cy0 - gap - lh / 2), (cu + f * lw, cy1 + gap + lh / 2),
                                   (cx0 - gap - lw / 2, cv + f * lh), (cx1 + gap + lw / 2, cv + f * lh)):
                        rb = rel[ref][ang]                  # text anchor that puts the glyph box centred on (u, v)
                        au, av = u - (rb[0] + rb[2]) / 2, v - (rb[1] + rb[3]) / 2
                        out.append((round((u - cu) ** 2 + (v - cv) ** 2, 4), ang, round(au, 4), round(av, 4),
                                    (u - lw / 2, v - lh / 2, u + lw / 2, v + lh / 2)))
        out.sort(key=lambda c: c[:4])
        return [c for c in out if allowed(ref, c[4])]
    def allowed(ref, bx):
        mine = body_box[ref]
        return (not boxes_hit(bx, own[ref]) and on_board(bx)
                and box_gap(bx, mine) + SILK_OWN_MARGIN <= min(box_gap(bx, bb) for r2, bb in body_box.items() if r2 != ref)
                and not any(boxes_hit(bx, ob) for ob, owner, kind in obstacles if not (owner == ref and kind == 'body')))
    free = {ref: candidates(ref) for ref in size}
    fixed = {}                                      # B12a: board_spec.SILK_LABEL_AT, held to the same rules, placed before the rest
    for ref, (u, v, ang) in B.SILK_LABEL_AT.items():
        lw, lh = size[ref] if ang == 0 else size[ref][::-1]
        rb, bx = rel[ref][ang], (u - lw / 2, v - lh / 2, u + lw / 2, v + lh / 2)
        assert allowed(ref, bx), f'SILK_LABEL_AT[{ref!r}] breaks the label rules'
        fixed[ref] = (0.0, ang, round(u - (rb[0] + rb[2]) / 2, 4), round(v - (rb[1] + rb[3]) / 2, 4), bx)
    # Greedy in a fixed order; the labels that found no room go first in the next pass (at most 6 passes).
    order = sorted((r for r in size if r not in fixed), key=lambda r: (B.PARTS[r]['mount'] != 'smd_top', B.PLACEMENT[r][1], B.PLACEMENT[r][0]))
    for _pass in range(6):
        placed, unplaced = dict(fixed), []
        for ref in order:
            pad = lambda bx: (bx[0] - SILK_SILK_GAP, bx[1] - SILK_SILK_GAP, bx[2] + SILK_SILK_GAP, bx[3] + SILK_SILK_GAP)
            hit = next((c for c in free[ref] if not any(boxes_hit(pad(c[4]), p[4]) for p in placed.values())), None)
            if hit is None: unplaced.append(ref)
            else: placed[ref] = hit
        if not unplaced: break
        order = unplaced + [r for r in order if r not in unplaced]
    for ref, (_d, ang, u, v, _bx) in placed.items():
        t = new_text(fps[ref], ref); fps[ref].Add(t); t.SetTextAngleDegrees(ang); t.SetPosition(V(u, v))
    return unplaced


def polygon_zone(board, layer, pts, net=None, name='', priority=0):
    z = pcbnew.ZONE(board); z.SetLayer(layer)
    if net is not None: z.SetNet(net)
    o = z.Outline(); o.NewOutline()
    for u, v in pts: o.Append(MM(u), MM(v))
    z.SetZoneName(name); z.SetAssignedPriority(priority)
    return z


UUID_SEED = 1      # L5b: KiCad saves the items sorted by UUID and the router's DSN follows that order, so random UUIDs gave every
                   # build a different DSN (and a different Freerouting result); seeded, a rebuild is byte-identical


LAYER = {L: getattr(pcbnew, L.replace('.', '_')) for L in C.STACKUP}      # STACKUP name -> pcbnew layer id


def build(out=OUT):
    board = new_board()
    pcbnew.KIID.SeedGenerator(UUID_SEED)               # after the stack-up template is read: the board's items get the seeded UUIDs
    outline = json.loads((D / 'board_outline.json').read_text(encoding='utf-8'))

    for ring in [outline['outline_uv']] + outline['cutouts_uv']:
        for (u1, v1), (u2, v2) in zip(ring, ring[1:] + ring[:1]):
            s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.Edge_Cuts)
            s.SetStart(V(u1, v1)); s.SetEnd(V(u2, v2)); s.SetWidth(MM(0.1)); board.Add(s)

    nets = {}
    for name in B.NETS:
        ni = pcbnew.NETINFO_ITEM(board, name); board.Add(ni); nets[name] = ni
    pin_net = {p: n for n, pins in B.NETS.items() for p in pins}
    mods = module_bodies()
    holes = outline['holes_uv'] + outline['clamp_holes_uv']       # H1-H4 M3 holes, H5 / H6 the clamp holes (PB3: Ø3.0 NPTH)

    for ref, part in B.PARTS.items():
        fp = load_footprint(part['fp'])
        fp.SetReference(ref); fp.SetValue(part['value'])
        if part.get('dnp'):                                  # ruling PB8: placed and netted, not fitted (out of the BOM / position files)
            fp.SetDNP(True); fp.SetExcludedFromPosFiles(True)
            dnp = pcbnew.PCB_TEXT(fp); dnp.SetLayer(pcbnew.F_Fab); dnp.SetText('DNP')     # PB13: "DNP" on F.Fab (the silk label stays short)
            dnp.SetTextSize(pcbnew.VECTOR2I(MM(1.0), MM(1.0))); dnp.SetTextThickness(MM(0.15)); dnp.SetPosition(V(0, 0)); fp.Add(dnp)
        if ref in mods: add_body_outline(fp, *mods[ref])     # module outline on F.Fab, drawn at the origin before placing
        if ref.startswith('H') and ref[1:].isdigit():
            (u, v), rot = holes[int(ref[1:]) - 1], 0
        else:
            u, v, rot = B.PLACEMENT[ref]
        fp.SetPosition(V(u, v)); fp.SetOrientationDegrees(rot)
        for pad in fp.Pads():
            key = (ref, pad.GetNumber())
            if key in pin_net: pad.SetNet(nets[pin_net[key]])
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_INHERITED)   # R13: the zones' wide reliefs everywhere (the M3 hole footprint
                                                                           # asks for a solid joint)
            if key in B.PAD_SPOKE_ANGLE: pad.SetThermalSpokeAngleDegrees(B.PAD_SPOKE_ANGLE[key])   # PB21: U3P.2's spokes face its patch
        board.Add(fp)
        if ref in B.PIN_MARKS:                               # B12a: the stock '+' / '-' texts (the XT30's, past its housing ends, under J2 /
            for it in fp.GraphicalItems():                   # K1) go to F.Fab; PIN_MARKS are the silk polarity marks
                if it.GetClass() == 'PCB_TEXT' and it.GetLayer() == pcbnew.F_SilkS and it.GetText() in ('+', '-'): it.SetLayer(pcbnew.F_Fab)
        for pin, sign in B.PIN_MARKS.get(ref, {}).items():    # ruling PB20 / B12a: '+' / '-' at the connectors' supply / GND pins
            if sign not in ('+', '-'): continue              # LIO-242: the text marks go on last (end of build)
            at = next(p for p in fp.Pads() if p.GetNumber() == pin).GetPosition()
            cu, cv = pcbnew.ToMM(at.x) + B.PIN_MARK_AT[ref][pin][0], pcbnew.ToMM(at.y) + B.PIN_MARK_AT[ref][pin][1]   # outside the housing
            h = B.PIN_MARK_LEN / 2
            for a, b in [((cu, cv - h), (cu, cv + h))] + ([((cu - h, cv), (cu + h, cv))] if sign == '+' else []):
                s = pcbnew.PCB_SHAPE(fp); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.F_SilkS)
                s.SetWidth(MM(B.SILK_STROKE)); s.SetStart(V(*a)); s.SetEnd(V(*b)); fp.Add(s)

    board_area = pcbnew.SHAPE_POLY_SET(); board_area.NewOutline()
    for u, v in outline['outline_uv']: board_area.Append(MM(u), MM(v))
    for c in outline['cutouts_uv']:
        cut = pcbnew.SHAPE_POLY_SET(); cut.NewOutline()
        for u, v in c: cut.Append(MM(u), MM(v))
        board_area.BooleanSubtract(cut)
    unplaced = place_silk_labels(board, {fp.GetReference(): fp for fp in board.GetFootprints()}, board_area)
    if unplaced and STRICT_LABELS: raise AssertionError(f'no room for the silk labels of {unplaced}')
    board.unplaced_labels = unplaced

    # Height zone (board_spec.HEIGHT_ZONES): the area under the Jetson carrier, documentation on F.Fab (ktest_pcb checks every part's
    # height there). Not a rule area: KiCad's Specctra export would turn that into a router keepout.
    for u0, v0, u1, v1, h in B.HEIGHT_ZONES:
        r = pcbnew.PCB_SHAPE(board); r.SetShape(pcbnew.SHAPE_T_RECT); r.SetLayer(pcbnew.F_Fab); r.SetWidth(MM(0.15))
        r.SetStart(V(u0, v0)); r.SetEnd(V(u1, v1)); board.Add(r)
        label = pcbnew.PCB_TEXT(board); label.SetLayer(pcbnew.F_Fab)
        label.SetText(f'Under the Jetson carrier: parts <= {h:g} mm (mated plug and wire bend included)'); label.SetTextAngleDegrees(90)
        label.SetPosition(V((u0 + u1) / 2, (v0 + v1) / 2))
        label.SetTextSize(pcbnew.VECTOR2I(MM(1.5), MM(1.5))); label.SetTextThickness(MM(0.2)); board.Add(label)

    def copper_zone(layer, pts, net, name, priority, clearance=ZONE_CLEARANCE, min_width=ZONE_MIN_WIDTH, spoke=THERMAL_SPOKE):
        z = polygon_zone(board, LAYER[layer], pts, nets[net], name, priority)
        z.SetLocalClearance(MM(clearance)); z.SetMinThickness(MM(min_width))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(MM(THERMAL_GAP)); z.SetThermalReliefSpokeWidth(MM(spoke))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        board.Add(z)
    # In1.Cu: the solid GND plane over the whole outline (R15's return paths, LP12's SMD GND vias land on it).
    copper_zone(C.GND_PLANE_LAYER, outline['outline_uv'], 'GND', 'GND_PLANE', 0, IN1_CLEARANCE, IN1_MIN_WIDTH)
    # POWER_ZONES on their layers (polygons of one layer never overlap, so their mutual priorities decide nothing); every one sits above
    # the B.Cu GND fill (priority 0), which pours round them.
    for k, z in enumerate(B.POWER_ZONES):
        for L in z['layers']:
            copper_zone(L, z['poly'], z['net'], f"{z['net']}_{'TRUNK' if z.get('trunk', True) else 'POUR'}_{L.split('.')[0]}", 1 + k,
                        spoke=z.get('spoke', THERMAL_SPOKE))      # fix 1: BAT+ / F1_OUT 1.5 mm spokes (PB12)
    copper_zone('B.Cu', outline['outline_uv'], 'GND', 'GND_FILL', 0)
    # F.Cu GND patches (B4 fix 1, ruling PB12) round GND trunk pins that need outer-layer spokes: one zone per patch (KiCad 9 saves only the
    # first outline of a multi-outline zone), above every POWER_ZONES priority.
    for k, pts in enumerate(B.GND_PATCHES):
        copper_zone('F.Cu', pts, 'GND', f'GND_PATCH_{k + 1}', 1 + len(B.POWER_ZONES) + k)
    # Stitching vias (B4 fix 1): through, the net class's via (board_spec.net_via), locked like route.py's pre-route vias.
    for net, (u, v) in B.STITCH_VIAS:
        via = pcbnew.PCB_VIA(board); via.SetPosition(V(u, v)); via.SetViaType(pcbnew.VIATYPE_THROUGH)
        dia, drill = B.net_via(net)
        via.SetWidth(MM(dia)); via.SetDrill(MM(drill)); via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetNet(nets[net]); via.SetLocked(True); board.Add(via)

    add_no_connect_nets(board)
    # LIO-242: the text pin marks ('VIN' at the DRV8874 carriers' VIN pin), last for the same reason as add_no_connect_nets: added earlier
    # they would shift every seeded UUID after them. place_silk_labels keeps clear of silk shapes only, so the labels do not move either.
    for ref, marks in B.PIN_MARKS.items():
        fp = board.FindFootprintByReference(ref)
        for pin, mark in marks.items():
            if mark in ('+', '-'): continue
            at = next(p for p in fp.Pads() if p.GetNumber() == pin).GetPosition()
            t = pcbnew.PCB_TEXT(fp); t.SetLayer(pcbnew.F_SilkS); t.SetText(mark)
            t.SetTextSize(pcbnew.VECTOR2I(MM(B.SILK_TEXT_W), MM(B.SILK_TEXT_H))); t.SetTextThickness(MM(B.SILK_STROKE))
            t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER); t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
            t.SetPosition(V(pcbnew.ToMM(at.x) + B.PIN_MARK_AT[ref][pin][0], pcbnew.ToMM(at.y) + B.PIN_MARK_AT[ref][pin][1])); fp.Add(t)
    ok = pcbnew.SaveBoard(str(out), board, True)      # aSkipSettings=True: keep r1_power_io.kicad_pro (net classes) as written
    assert ok, 'SaveBoard failed'
    return board


if __name__ == '__main__':
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT     # B12a: route.py builds its start board into its work dir
    b = build(out)
    print('placed', len(B.PARTS), 'footprints,', sum(1 for z in b.Zones() if not z.GetIsRuleArea()), 'copper zones on',
          b.GetCopperLayerCount(), 'layers ->', out.name)

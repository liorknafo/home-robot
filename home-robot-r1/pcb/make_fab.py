"""Bay B7: the fabrication package of the routed 4-layer bay board -> pcb/fab/.

Run with KiCad's Python:  $KPY pcb/make_fab.py          (reads pcb/r1_power_io.kicad_pcb; never writes it)

Writes (pcb/fab/, committed; ORDERING.md and BRINGUP-NOTES.md there are hand-written):
 - r1_power_io-gerbers.zip  the ten layers JLCPCB needs (F.Cu, In1.Cu, In2.Cu, B.Cu, F/B.Mask, F/B.Silkscreen, F.Paste for the stencil,
                            Edge.Cuts; KiCad's X2 attributes on) + the Excellon drill, plated and non-plated holes in separate files:
                            12 files, no more. KiCad's Gerber job file (.gbrjob) is left out (B7 fix 1): JLCPCB does not need it and it
                            declares the outline's size with the 0.1 mm Edge.Cuts line width added (74.0 x 100.1 mm), which can make the
                            order page read 100.1 mm. Upload this one zip for the board and for the top stencil;
 - r1_power_io-bom.csv      from board_spec.PARTS (one row per part kind, the MPN as PARTS gives it; DNP rows and the bare test pads /
                            holes marked) + board_spec.BOM_EXTRAS (the modules and fuses that plug into the board's parts);
 - r1_power_io-pos.csv      KiCad's placement file (mm; x = u, y = -v; DNP rows and the bare pads / holes left out);
 - r1_power_io-1to1.pdf     four A4 pages at 1:1 (F.Fab with the pads sketched, F.Silkscreen, F.Cu, B.Cu; Edge.Cuts on each), the board
                            centred on the sheet (a copy moved, so a printer's margin cannot clip it) with a SCALE_BAR_MM bar to check
                            the print scale;
 - r1_power_io.step         for B8: the board (KiCad's body, tracks and vias) and every fitted part's 3D model (--subst-models; the DNP
                            spares left out), with box envelopes (envelope_boxes) for the parts KiCad has no model for - the board
                            connectors J1 (AMASS XT30UPB-M), J2 (JST VH) and J3 (Molex Micro-Fit 43650-0200; B12a) and the FLR fuse
                            holders F1 / F2 with their ATO fuses, and for C1 (ENVELOPE_OVER_MODEL: the
                            stock model is 10 mm tall, the part up to 20 mm). Frame: --user-origin 0x0mm, so STEP x = u,
                            y = -v (KiCad's board frame, y flipped), z 0 = the bottom of KiCad's laminate body (B.Cu copper -0.035, the
                            parts' seating plane z 1.595 = the board top): robot (x, y, z) = STEP (x + BOARD_X0, y + BOARD_Y0, z +
                            Z_PCB_BOTTOM). The stock models show untrimmed leads (down to about z -1.9); the assembled board trims them
                            to <= 1.4 mm (ORDERING.md);
 - SOURCE.sha256            the sha256 of the board, its DRC rules and its project (LF line endings, so a CRLF checkout hashes the
                            same), one `<sha256>  <file>` line each: test_fab fails when the board changed after the package was made.
The board itself is never changed: every export that needs a change (the PDF's page position, the STEP's envelope models, the DNP pads
off the stencil) works on a copy in a temp dir. pcb/tests/test_fab.py checks the package (pure)."""
import csv, hashlib, math, re, shutil, subprocess, sys, tempfile, zipfile
from pathlib import Path
import pcbnew
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B, tools

D = Path(__file__).parent
NAME = 'r1_power_io'
PCB, PRO, DRU = D / f'{NAME}.kicad_pcb', D / f'{NAME}.kicad_pro', D / f'{NAME}.kicad_dru'
FAB = D / 'fab'
MM, TOMM = pcbnew.FromMM, pcbnew.ToMM
MODEL_DIR = tools.MODEL_DIR

GERBER_LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu', 'F.Mask', 'B.Mask', 'F.Silkscreen', 'B.Silkscreen', 'F.Paste', 'Edge.Cuts']
PDF_LAYERS, PDF_COMMON = ['F.Fab', 'F.Silkscreen', 'F.Cu', 'B.Cu'], ['Edge.Cuts', 'Cmts.User']
PAGE = (297.0, 210.0)            # the board's sheet: A4 landscape (KiCad's default paper)
SCALE_BAR_MM = 50.0              # test_fab finds this bar on every page at this length

# Envelopes (no KiCad model): the footprint's own F.Fab outline as the plan, PARTS h as the top (board_spec, the height the B3 zone tests
# use). The FLR holder (pcb/reference/littelfuse-flr-178.6165.txt): housing 17.5 mm over the board; the ATO fuse above it, its body taken
# as the standard ATO blade fuse's 19.1 x 5.1 mm (verify before ordering), centred on the holder, up to PARTS h (30 mm).
FLR_HOUSING_H = 17.5
ATO_BODY = (19.1, 5.1)
ENVELOPE_RGB = {'Connector_JST': (0.93, 0.92, 0.86), 'Fuse': (0.08, 0.08, 0.08),       # natural nylon VH; black FLR housing
                'Capacitor_THT': (0.10, 0.10, 0.13),                                     # black can
                'Connector_AMASS': (0.95, 0.78, 0.10), 'Connector_Molex': (0.10, 0.10, 0.10)}   # B12a: yellow XT30 (J1); black Micro-Fit (J3)
# Parts that have a KiCad model but get an envelope as well (instead of the model): the stock model is lower than the part. C1: the stock
# CP_Radial_D10.0mm_P5.00mm model is 10 mm tall, the Panasonic EEU-FR1E471 (470 uF 25 V, D10) is up to 20 mm (PARTS h 20; the datasheet
# height is on the verify list). The other parts whose model and PARTS h differ are listed in ORDERING.md "STEP heights" (B8).
ENVELOPE_OVER_MODEL = ('C1',)
ATO_RGB = {'7.5 A ATO': (0.55, 0.35, 0.17), '5 A ATO': (0.82, 0.71, 0.55)}              # ATO colour code: 7.5 A brown, 5 A tan
GREY = (0.5, 0.5, 0.5)


def kcli(*args):
    r = subprocess.run([tools.KICAD_CLI, *map(str, args)], capture_output=True, text=True)
    if r.returncode: raise RuntimeError(f'kicad-cli {" ".join(map(str, args[:3]))} failed:\n{r.stdout}\n{r.stderr}')
    return r.stdout


def board_copy(tmp):
    """A copy of the board with its project and rules in tmp (the exports that change the board work on it)."""
    for f in (PCB, PRO, DRU): shutil.copy(f, tmp / f.name)
    return tmp / PCB.name


# --- Gerbers and drill ------------------------------------------------------------------------------------------------------------------
def gerbers(tmp):
    """LIO-226: from a board copy whose DNP parts' pads are off F.Paste - KiCad plots a DNP footprint's paste, and the stencil must leave
    D2's pads bare."""
    path = board_copy(tmp)
    b = pcbnew.LoadBoard(str(path))
    for fp in b.GetFootprints():
        if not fp.IsDNP(): continue
        for p in fp.Pads():
            ls = p.GetLayerSet(); ls.RemoveLayer(pcbnew.F_Paste); p.SetLayerSet(ls)
    pcbnew.SaveBoard(str(path), b, True)
    out = tmp / 'gerbers'; out.mkdir()
    kcli('pcb', 'export', 'gerbers', '--layers', ','.join(GERBER_LAYERS), '-o', f'{out}/', path)
    kcli('pcb', 'export', 'drill', '--format', 'excellon', '--excellon-units', 'mm', '--excellon-separate-th', '-o', f'{out}/', path)
    files = [f for f in sorted(out.iterdir()) if f.suffix != '.gbrjob']        # no job file: it states the size as 100.1 mm (line width added)
    assert any(f.name.endswith('-PTH.drl') for f in files) and any(f.name.endswith('-NPTH.drl') for f in files), files
    assert len(files) == len(GERBER_LAYERS) + 2, [f.name for f in files]       # one Gerber per layer + the PTH and NPTH drill files
    dest = FAB / f'{NAME}-gerbers.zip'
    with zipfile.ZipFile(dest, 'w', zipfile.ZIP_DEFLATED) as z:
        for f in files: z.write(f, f.name)
    return dest


# --- BOM and placement --------------------------------------------------------------------------------------------------------------------
BOM_FIELDS = ['Refs', 'Qty', 'Fit', 'Value', 'MPN', 'Footprint', 'Mount', 'Notes']
def part_note(ref):
    """The note a part puts on its BOM row (a row joins its refs' notes)."""
    if ref == 'J1': return 'battery: pin 1 = GND, pin 2 = BAT+ (PB20; silk - / +): the battery lead GND in cavity 1'
    if ref in ('C1', 'C3'): return 'polarised: + on silk' + ('; under the IMU module (5.8 mm can, PB16 / PB20)' if ref == 'C3' else '')
    if ref == 'D1' or ref.startswith('LED'): return 'cathode mark on silk'
    if ref == 'D2': return ("DNP (LIO-226): 5V to the Pico's VSYS, fit it to run the Pico and 3V3 without the Jetson USB (BRINGUP.md "
                            'section 4 item 2); under the DRV_R carrier; cathode mark on silk')
    if ref in ('F1', 'F2'):            # one note for both holders (the rows join equal notes once); the fuses are plug-in rows (BOM_EXTRAS)
        fuses = ', '.join(f'{B.PARTS[r]["value"]} in {r}' for r in ('F1', 'F2'))
        return (f'fuse holder only (the fuses are the plug-in rows: {fuses}); the KiCad footprint gives PCB <= 1.5 mm for its snap lock, '
                f'this board is {B.PCB_T:g} mm: hold it flat and solder all 8 pins')
    if ref in ('J4', 'J5'): return 'right-angle: the motor plug lies on the board'
    if ref in ('J14', 'J15'):
        n = len(B.symbol_pin_numbers(B.PARTS[ref]['sym']))
        return f'DNP spare pad row (PB8); a bare 1x{n} 2.54 mm header may be fitted later (<= {B.BARE_HEADER_H:g} mm tall)'
    if ref.startswith('TP'): return 'bare copper test pad, no part'
    if ref in ('H5', 'H6'): return 'M2.5 clamp hole (NPTH, PB3), no part'
    if ref.startswith('H'): return 'plated M3 mounting hole (GND ring), no part'
    return ''


def bom_value(ref):
    """The Value column: PARTS' value, except for a fuse holder, whose PARTS value is the fuse it takes (the fuses are plug-in rows)."""
    return 'ATO fuse holder (FLR)' if B.PARTS[ref]['fp'].startswith('Fuse:') else B.PARTS[ref]['value']


def natural(ref):
    m = re.match(r'([A-Z]+)(\d+)([A-Z]*)', ref)
    return (m.group(1), int(m.group(2)), m.group(3))


def fit(ref):
    return 'DNP' if B.PARTS[ref].get('dnp') else 'yes' if B.in_bom(ref) else 'no part'


def bom_rows():
    """One row per part to buy: PARTS grouped by (fit, MPN, footprint, mount) - the values (a connector's function) joined - fitted SMD
    rows first, then fitted THT, DNP, no part; then BOM_EXTRAS."""
    groups = {}
    for ref, p in B.PARTS.items():
        groups.setdefault((fit(ref), p['mpn'], p['fp'], p['mount']), []).append(ref)
    order = {'yes': 0, 'DNP': 1, 'no part': 2}
    rows = []
    for (f, mpn, fp, mount), refs in sorted(groups.items(), key=lambda kv: (order[kv[0][0]], kv[0][3] != 'smd_top',
                                                                             natural(min(kv[1], key=natural)))):
        refs.sort(key=natural)
        notes = '; '.join(dict.fromkeys(n for n in map(part_note, refs) if n))
        value = ' / '.join(dict.fromkeys(bom_value(r) for r in refs))
        rows.append(dict(Refs=' '.join(refs), Qty=len(refs), Fit=f, Value=value, MPN=mpn, Footprint=fp,
                         Mount='SMD top (reflow)' if mount == 'smd_top' else 'THT (iron)', Notes=notes))
    for item in B.BOM_EXTRAS:
        rows.append(dict(Refs='', Qty=item['qty'], Fit='plug-in', Value=item['item'], MPN=item['mpn'], Footprint='',
                         Mount='plugs in', Notes=f'into {", ".join(item["on"])}'))
    return rows


def bom():
    dest = FAB / f'{NAME}-bom.csv'
    with open(dest, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, BOM_FIELDS); w.writeheader(); w.writerows(bom_rows())
    return dest


def positions():
    dest = FAB / f'{NAME}-pos.csv'
    kcli('pcb', 'export', 'pos', '--format', 'csv', '--units', 'mm', '--side', 'both', '--exclude-dnp', '-o', dest, PCB)
    return dest


# --- 1:1 PDF ------------------------------------------------------------------------------------------------------------------------------
def pdf(tmp):
    """The board copy moved to the middle of its A4 sheet, a SCALE_BAR_MM bar and a print note on Cmts.User, the title block filled in;
    KiCad plots at 1:1 (kicad-cli has no scale option)."""
    path = board_copy(tmp)
    b = pcbnew.LoadBoard(str(path))
    off = pcbnew.VECTOR2I(MM((PAGE[0] - B.BOARD_W) / 2), MM(25.0))
    for it in [*b.GetFootprints(), *b.GetTracks(), *b.Zones(), *b.GetDrawings()]: it.Move(off)
    x0, y0 = (PAGE[0] - B.BOARD_W) / 2, 25.0 + B.BOARD_H + 8.0
    bar = pcbnew.PCB_SHAPE(b); bar.SetShape(pcbnew.SHAPE_T_SEGMENT); bar.SetLayer(pcbnew.Cmts_User); bar.SetWidth(MM(0.3))
    bar.SetStart(pcbnew.VECTOR2I(MM(x0), MM(y0))); bar.SetEnd(pcbnew.VECTOR2I(MM(x0 + SCALE_BAR_MM), MM(y0))); b.Add(bar)
    note = pcbnew.PCB_TEXT(b); note.SetLayer(pcbnew.Cmts_User)
    note.SetText(f'Print at 100 % (actual size): this bar is {SCALE_BAR_MM:g} mm, the board {B.BOARD_W:g} x {B.BOARD_H:g} mm. Top view.')
    note.SetTextSize(pcbnew.VECTOR2I(MM(2.0), MM(2.0))); note.SetTextThickness(MM(0.25))
    note.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT); note.SetPosition(pcbnew.VECTOR2I(MM(x0), MM(y0 + 5.0))); b.Add(note)
    tb = b.GetTitleBlock()
    tb.SetTitle(f'R1 bay board, 4 layers, {B.BOARD_W:g} x {B.BOARD_H:g} mm, 1:1')
    tb.SetCompany('home-robot R1'); tb.SetRevision('B7')
    tb.SetComment(0, 'Pages: F.Fab, F.Silkscreen, F.Cu, B.Cu - top view')
    tb.SetComment(1, 'J1: pin 1 = GND, pin 2 = BAT+. J14 / J15 / D2 DNP.')
    pcbnew.SaveBoard(str(path), b, True)
    out = tmp / 'pdf'; out.mkdir()
    kcli('pcb', 'export', 'pdf', '--mode-multipage', '--layers', ','.join(PDF_LAYERS), '--common-layers', ','.join(PDF_COMMON),
         '--include-border-title', '--black-and-white', '--sketch-pads-on-fab-layers', '--crossout-DNP-footprints-on-fab-layers',
         '-o', f'{out}/', path)
    dest = FAB / f'{NAME}-1to1.pdf'
    shutil.move(next(out.glob('*.pdf')), dest)
    return dest


# --- STEP ---------------------------------------------------------------------------------------------------------------------------------
def model_missing(fp):
    """True when none of the footprint's 3D models (or a .step of the same name, --subst-models) is on disk."""
    for m in fp.Models():
        p = Path(m.m_Filename.replace('${KICAD9_3DMODEL_DIR}', str(MODEL_DIR)))
        if p.exists() or p.with_suffix('.step').exists(): return False
    return True


def fab_box(fp_id):
    """(x0, y0, x1, y1) of the library footprint's F.Fab drawing (centrelines), in its own frame (KiCad: y down)."""
    lib, name = fp_id.split(':')
    fp = pcbnew.FootprintLoad(str(tools.FOOTPRINT_DIR / f'{lib}.pretty'), name)
    pts = []
    for it in fp.GraphicalItems():
        if it.GetLayer() != pcbnew.F_Fab or it.GetClass() != 'PCB_SHAPE': continue
        s = it.GetShape()
        if s == pcbnew.SHAPE_T_CIRCLE:
            c, r = it.GetCenter(), it.GetRadius(); pts += [(c.x - r, c.y - r), (c.x + r, c.y + r)]
        elif s == pcbnew.SHAPE_T_POLY:
            o = it.GetPolyShape().Outline(0); pts += [(o.CPoint(i).x, o.CPoint(i).y) for i in range(o.PointCount())]
        else:
            pts += [(p.x, p.y) for p in (it.GetStart(), it.GetEnd())]
    xs, ys = [TOMM(p[0]) for p in pts], [TOMM(p[1]) for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def envelope_boxes(ref):
    """[(x0, y0, z0, x1, y1, z1, rgb)] in the 3D model frame (mm; x as the footprint, y = -footprint y, z up from the board top)."""
    part = B.PARTS[ref]
    x0, y0, x1, y1 = fab_box(part['fp'])
    lib = part['fp'].split(':')[0]
    if lib == 'Fuse':
        cx, cy = (x0 + x1) / 2, -(y0 + y1) / 2
        (w, d), rgb = ATO_BODY, ATO_RGB.get(part['value'], GREY)
        return [(x0, -y1, 0.0, x1, -y0, FLR_HOUSING_H, ENVELOPE_RGB.get(lib, GREY)),
                (cx - w / 2, cy - d / 2, FLR_HOUSING_H, cx + w / 2, cy + d / 2, part['h'], rgb)]
    return [(x0, -y1, 0.0, x1, -y0, part['h'], ENVELOPE_RGB.get(lib, GREY))]


def box_step(path, name, boxes):
    """A minimal AP214 STEP of axis-aligned boxes (one coloured MANIFOLD_SOLID_BREP each), for KiCad's STEP exporter to place like a model."""
    ents = []
    def add(s):
        ents.append(s)
        return f'#{len(ents)}'
    def real(v): return f'{v:.6f}'.rstrip('0')                     # '1.', '0.5', '-3.65': STEP reals keep the point
    def pt(p): return add("CARTESIAN_POINT('',(%s))" % ','.join(real(c) for c in p))
    def dr(d): return add("DIRECTION('',(%s))" % ','.join(real(c) for c in d))
    app = add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")
    add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,%s)" % app)
    prod = add("PRODUCT('%s','%s','',(%s))" % (name, name, add("PRODUCT_CONTEXT('',%s,'mechanical')" % app)))
    add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(%s))" % prod)
    pdef = add("PRODUCT_DEFINITION('design','',%s,%s)" % (add("PRODUCT_DEFINITION_FORMATION('','',%s)" % prod),
                                                          add("PRODUCT_DEFINITION_CONTEXT('part definition',%s,'design')" % app)))
    pds = add("PRODUCT_DEFINITION_SHAPE('','',%s)" % pdef)
    mm = add('( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )')
    rad = add('( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )')
    sr = add('( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )')
    unc = add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),%s,'distance_accuracy_value','confusion accuracy')" % mm)
    ctx = add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((%s)) GLOBAL_UNIT_ASSIGNED_CONTEXT((%s,%s,%s)) "
              "REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )" % (unc, mm, rad, sr))
    origin = add("AXIS2_PLACEMENT_3D('',%s,%s,%s)" % (pt((0, 0, 0)), dr((0, 0, 1)), dr((1, 0, 0))))
    # the unit box's faces: corner (i, j, l) loops, counter-clockwise seen from outside, and the outward normal
    faces = [((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0), (0, 0, -1)), ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1), (0, 0, 1)),
             ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (0, -1, 0)), ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0), (0, 1, 0)),
             ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0), (-1, 0, 0)), ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1), (1, 0, 0))]
    solids, styled = [], []
    for k, (x0, y0, z0, x1, y1, z1, rgb) in enumerate(boxes):
        P = {(i, j, l): ((x0, x1)[i], (y0, y1)[j], (z0, z1)[l]) for i in (0, 1) for j in (0, 1) for l in (0, 1)}
        V = {c: add("VERTEX_POINT('',%s)" % pt(p)) for c, p in P.items()}
        E = {}
        def oriented(a, b):
            key = tuple(sorted((a, b)))
            if key not in E:                                        # one EDGE_CURVE per box edge, from its lower to its upper corner
                s, e = P[key[0]], P[key[1]]
                d = [e[i] - s[i] for i in range(3)]; n = math.sqrt(sum(c * c for c in d))
                vec = add("VECTOR('',%s,%s)" % (dr([c / n for c in d]), real(n)))
                E[key] = add("EDGE_CURVE('',%s,%s,%s,.T.)" % (V[key[0]], V[key[1]], add("LINE('',%s,%s)" % (pt(s), vec))))
            return add("ORIENTED_EDGE('',*,*,%s,%s)" % (E[key], '.T.' if (a, b) == key else '.F.'))
        fids = []
        for *loop, nrm in faces:
            lp = add("EDGE_LOOP('',(%s))" % ','.join(oriented(loop[i], loop[(i + 1) % 4]) for i in range(4)))
            ax = add("AXIS2_PLACEMENT_3D('',%s,%s,%s)" % (pt(P[loop[0]]), dr(nrm), dr((1, 0, 0) if nrm[0] == 0 else (0, 1, 0))))
            fids.append(add("ADVANCED_FACE('',(%s),%s,.T.)" % (add("FACE_OUTER_BOUND('',%s,.T.)" % lp), add("PLANE('',%s)" % ax))))
        solid = add("MANIFOLD_SOLID_BREP('%s %d',%s)" % (name, k + 1, add("CLOSED_SHELL('',(%s))" % ','.join(fids))))
        solids.append(solid)
        fas = add("FILL_AREA_STYLE('',(%s))" % add("FILL_AREA_STYLE_COLOUR('',%s)" % add("COLOUR_RGB('',%s)" % ','.join(real(c) for c in rgb))))
        sss = add("SURFACE_SIDE_STYLE('',(%s))" % add('SURFACE_STYLE_FILL_AREA(%s)' % fas))
        psa = add('PRESENTATION_STYLE_ASSIGNMENT((%s))' % add('SURFACE_STYLE_USAGE(.BOTH.,%s)' % sss))
        styled.append(add("STYLED_ITEM('color',(%s),%s)" % (psa, solid)))
    rep = add("ADVANCED_BREP_SHAPE_REPRESENTATION('%s',(%s),%s)" % (name, ','.join(solids + [origin]), ctx))
    add('SHAPE_DEFINITION_REPRESENTATION(%s,%s)' % (pds, rep))
    add("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(%s),%s)" % (','.join(styled), ctx))
    head = ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('envelope of a part with no KiCad 3D model (pcb/make_fab.py)'),'2;1');\n"
            f"FILE_NAME('{path.name}','2026-10-09T00:00:00',(''),(''),'pcb/make_fab.py','pcb/make_fab.py','');\n"
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n")
    path.write_bytes((head + ''.join(f'#{i}={e};\n' for i, e in enumerate(ents, 1)) + 'ENDSEC;\nEND-ISO-10303-21;\n').encode())
    return path


def step(tmp):
    """The board copy with an envelope model (box_step) on every fitted part whose models are all missing, and on ENVELOPE_OVER_MODEL
    (instead of their too-low model), exported with KiCad's STEP exporter. Returns (STEP path, refs that got an envelope)."""
    path = board_copy(tmp)
    b = pcbnew.LoadBoard(str(path))
    env = []
    for fp in b.GetFootprints():
        ref = fp.GetReference()
        if not B.in_bom(ref) or B.PARTS[ref].get('dnp') or not (model_missing(fp) or ref in ENVELOPE_OVER_MODEL): continue
        model = box_step(tmp / f'{ref}_envelope.step', f'{ref}_envelope_{B.PARTS[ref]["fp"].split(":")[1]}', envelope_boxes(ref))
        fp.Models().clear()
        m = pcbnew.FP_3DMODEL(); m.m_Filename = str(model); m.m_Show = True
        fp.Models().push_back(m)
        env.append(ref)
    pcbnew.SaveBoard(str(path), b, True)
    dest = FAB / f'{NAME}.step'
    log = kcli('pcb', 'export', 'step', '--subst-models', '--include-tracks', '--no-dnp', '--user-origin', '0x0mm', '-f', '-o', dest, path)
    missing = re.findall(r'Could not add 3D model for (\S+)\.', log)
    assert not missing, f'no 3D model for {missing}:\n{log}'
    return dest, sorted(env, key=natural)


def source_sha256():
    return ''.join(hashlib.sha256(p.read_bytes().replace(b'\r\n', b'\n')).hexdigest() + f'  {p.name}\n' for p in (PCB, DRU, PRO))


def main():
    FAB.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        out = [gerbers(tmp), bom(), positions()]
        (tmp / 'p').mkdir(); out.append(pdf(tmp / 'p'))
        (tmp / 's').mkdir(); st, env = step(tmp / 's'); out.append(st)
    out.append(FAB / 'SOURCE.sha256'); out[-1].write_bytes(source_sha256().encode())
    for p in out: print(f'{p.relative_to(D.parent)}  {p.stat().st_size / 1e3:9.1f} kB')
    print('envelopes (no KiCad model, or too low a one):', ', '.join(env))
    return 0


if __name__ == '__main__':
    sys.exit(main())

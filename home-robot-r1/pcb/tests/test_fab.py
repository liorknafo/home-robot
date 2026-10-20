"""Bay B7: the fabrication package in pcb/fab (pure: reads the committed files; `$KPY pcb/make_fab.py` writes them).

The package: r1_power_io-gerbers.zip (the ten plotted layers + PTH / NPTH Excellon drill, 12 files, no job file), -bom.csv (from board_spec.PARTS and BOM_EXTRAS),
-pos.csv (placement, DNP rows left out), -1to1.pdf (printable at 100 %), r1_power_io.step (B8 reads it: every fitted part, envelopes
where KiCad has no model), ORDERING.md and BRINGUP-NOTES.md (for B10)."""
import csv, hashlib, json, math, re, unittest, zipfile, zlib
from pathlib import Path
from pcb import board_spec as B
from pcb.tests.left_out import needs_kicad

FAB = Path(B.__file__).parent / 'fab'
ZIP, BOM, POS, PDF, STEP = (FAB / f'r1_power_io{s}' for s in ('-gerbers.zip', '-bom.csv', '-pos.csv', '-1to1.pdf', '.step'))
ORDERING, BRINGUP = FAB / 'ORDERING.md', FAB / 'BRINGUP-NOTES.md'
BOARD = FAB.parent / 'r1_power_io.kicad_pcb'        # the routed board the package is made from (the vias are counted in it)

# The X2 file function KiCad writes in each layer's Gerber header (JLCPCB's viewer reads them).
LAYER_FUNCTION = {'F.Cu': 'Copper,L1,Top', 'In1.Cu': 'Copper,L2,Inr', 'In2.Cu': 'Copper,L3,Inr', 'B.Cu': 'Copper,L4,Bot',
                  'F.Mask': 'Soldermask,Top', 'B.Mask': 'Soldermask,Bot', 'F.Silkscreen': 'Legend,Top', 'B.Silkscreen': 'Legend,Bot',
                  'F.Paste': 'Paste,Top', 'Edge.Cuts': 'Profile,NP'}
VIA_DRILLS = sorted({d for _, d in B.CLASS_VIA.values()} | {B.GND_VIA[1]})     # 0.3 (signal, PB17), 0.4 (GND / PowerTap / Motor), 0.6
SCALE_BAR_MM = 50.0              # make_fab draws a bar this long on every PDF page: it must print at this length


def fitted(ref):
    """A bought part that goes on this board (not DNP, not a bare copper pad or a hole)."""
    return B.in_bom(ref) and not B.PARTS[ref].get('dnp')


def gerber_points(text):
    """[(x, y, op)] in mm of every D01 / D02 / D03 coordinate of an RS-274X file (FSLAX46Y46, metric; modal X / Y)."""
    assert '%MOMM*%' in text and '%FSLAX46Y46*%' in text
    x = y = 0.0; out = []
    for m in re.finditer(r'(?:X(-?\d+))?(?:Y(-?\d+))?(?:I-?\d+J-?\d+)?D0([123])\*', text):
        if m.group(1) is None and m.group(2) is None: continue
        if m.group(1) is not None: x = int(m.group(1)) / 1e6
        if m.group(2) is not None: y = int(m.group(2)) / 1e6
        out.append((x, y, int(m.group(3))))
    return out


def excellon(text):
    """{drill mm: [(x, y)]} of an Excellon file (metric, decimal)."""
    tools, holes, cur = {}, {}, None
    for line in text.splitlines():
        if m := re.fullmatch(r'T(\d+)C([\d.]+)', line): tools[m.group(1)] = float(m.group(2)); continue
        if m := re.fullmatch(r'T(\d+)', line): cur = tools[m.group(1)]; continue
        if m := re.fullmatch(r'X(-?[\d.]+)Y(-?[\d.]+)', line): holes.setdefault(cur, []).append((float(m.group(1)), float(m.group(2))))
    return holes


def via_holes(text):
    """[(x, y, drill mm)] of the PTH file's via drills: the tools KiCad tags `TA.AperFunction,Plated,PTH,ViaDrill` (not the part holes)."""
    sizes, via_tools, cur, tag, out = {}, set(), None, '', []
    for line in text.splitlines():
        if line.startswith('; #@! TA.AperFunction'): tag = line; continue
        if m := re.fullmatch(r'T(\d+)C([\d.]+)', line):
            sizes[m.group(1)] = float(m.group(2))
            if tag.endswith(',ViaDrill'): via_tools.add(m.group(1))
            tag = ''; continue
        if m := re.fullmatch(r'T(\d+)', line): cur = m.group(1); continue
        if m := re.fullmatch(r'X(-?[\d.]+)Y(-?[\d.]+)', line):
            if cur in via_tools: out.append((float(m.group(1)), float(m.group(2)), sizes[cur]))
    return out


def gerber_flashes(text):
    """[(aperture, x, y)] of every flash in a flash-only RS-274X file (a solder-mask layer of this board: circles, rectangles, obrounds and
    KiCad's RoundRect macro; no regions, no draws, no clear polarity - anything else fails loudly). The aperture is (kind, numbers) as
    written: C d | R w, h | O w, h | RoundRect r, then the four corners (x, y)."""
    assert '%MOMM*%' in text and '%FSLAX46Y46*%' in text
    assert 'G36' not in text and '%LPC' not in text and not re.search(r'D01\*', text), 'a mask layer with regions, draws or clear polarity'
    aps = {int(m.group(1)): (m.group(2), [float(v) for v in m.group(3).split('X')])
           for m in re.finditer(r'%ADD(\d+)(C|R|O|RoundRect),([^*]*)\*%', text)}
    cur, out = None, []
    for line in text.splitlines():
        if m := re.fullmatch(r'D(\d+)\*', line): cur = int(m.group(1)); continue
        if m := re.fullmatch(r'X(-?\d+)Y(-?\d+)D03\*', line): out.append((aps[cur], int(m.group(1)) / 1e6, int(m.group(2)) / 1e6))
    return out


def _segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def flash_distance(flash, px, py):
    """Signed distance in mm from the point (px, py) to the edge of a flash (negative inside) - every aperture here is convex."""
    (kind, v), cx, cy = flash
    x, y = px - cx, py - cy
    if kind == 'C': return math.hypot(x, y) - v[0] / 2
    if kind == 'R':
        dx, dy = abs(x) - v[0] / 2, abs(y) - v[1] / 2
        return math.hypot(max(dx, 0.0), max(dy, 0.0)) if max(dx, dy) > 0 else max(dx, dy)
    if kind == 'O':                                                    # a stadium: the segment along the long side, grown by the short half
        w, h = v[0], v[1]
        if w >= h: return _segment_distance(x, y, -(w - h) / 2, 0, (w - h) / 2, 0) - h / 2
        return _segment_distance(x, y, 0, -(h - w) / 2, 0, (h - w) / 2) - w / 2
    r, pts = v[0], [(v[1 + 2 * i], v[2 + 2 * i]) for i in range(4)]    # RoundRect: the corner quad grown by r
    edges = [_segment_distance(x, y, *pts[i], *pts[(i + 1) % 4]) for i in range(4)]
    cross = [(pts[(i + 1) % 4][0] - pts[i][0]) * (y - pts[i][1]) - (pts[(i + 1) % 4][1] - pts[i][1]) * (x - pts[i][0]) for i in range(4)]
    inside = all(c >= 0 for c in cross) or all(c <= 0 for c in cross)
    return (-min(edges) if inside else min(edges)) - r


def step_entities(text):
    """{id: 'ENTITY(...)'} of a STEP file's DATA section (whitespace collapsed)."""
    data = text[text.index('DATA;') + 5: text.rindex('ENDSEC;')]
    return {int(m.group(1)): ' '.join(m.group(2).split()) for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)', data, re.S)}


class FabPackageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.zip = zipfile.ZipFile(ZIP)
        cls.files = {n: cls.zip.read(n).decode('ascii', 'replace') for n in cls.zip.namelist()}
        cls.layer_file = {}
        for L, func in LAYER_FUNCTION.items():
            hits = [n for n, t in cls.files.items() if L.replace('.', '_') in n and f'%TF.FileFunction,{func}*%' in t]
            cls.layer_file[L] = hits[0] if len(hits) == 1 else None
        with open(BOM, newline='', encoding='utf-8') as f: cls.bom = list(csv.DictReader(f))

    # --- brief step 1: the zip ---
    def test_zip_holds_every_layer(self):
        """F.Cu, In1.Cu, In2.Cu, B.Cu, F/B.Mask, F/B.Silkscreen, F.Paste and Edge.Cuts, one Gerber each, recognised by its X2 file function."""
        self.assertEqual({L for L, n in self.layer_file.items() if n is None}, set())

    def test_zip_holds_exactly_the_expected_files(self):
        """B7 fix 1: the ten layers + PTH + NPTH, 12 files and nothing else: no extra layer (B.Paste, a second B.Mask, ...) and no job file
        (the .gbrjob states the size as 100.1 mm, the outline's line width added: JLCPCB would read it)."""
        names = self.zip.namelist()
        self.assertEqual([n for n in names if n.endswith('.gbrjob') or 'job' in n.lower()], [])
        pth = [n for n in names if n.endswith('-PTH.drl')]; npth = [n for n in names if n.endswith('-NPTH.drl')]
        self.assertEqual((len(pth), len(npth)), (1, 1))
        self.assertNotIn(None, self.layer_file.values())              # each layer: exactly one file with its X2 function
        self.assertEqual(sorted(names), sorted([*self.layer_file.values(), pth[0], npth[0]]))
        self.assertEqual(len(names), len(LAYER_FUNCTION) + 2)

    def test_zip_holds_pth_and_npth_drill(self):
        pth = [n for n, t in self.files.items() if n.endswith('-PTH.drl') and 'TF.FileFunction,Plated,1,4,PTH' in t]
        npth = [n for n, t in self.files.items() if n.endswith('-NPTH.drl') and 'TF.FileFunction,NonPlated,1,4,NPTH' in t]
        self.assertEqual((len(pth), len(npth)), (1, 1))
        holes = excellon(self.files[pth[0]])
        self.assertEqual(min(holes), min(VIA_DRILLS))               # the smallest hole is the PB17 signal via (ORDERING.md states it)
        for d in VIA_DRILLS: self.assertIn(d, holes)
        np_holes = excellon(self.files[npth[0]])
        for u, v in json.loads((FAB.parent / 'board_outline.json').read_text(encoding='utf-8'))['clamp_holes_uv']:
            self.assertTrue(any(abs(x - u) < 0.01 and abs(y + v) < 0.01 for x, y in np_holes.get(3.0, [])), (u, v))  # PB3: Ø3.0 NPTH
        # LIO-229: H5, H6 and J3's Micro-Fit peg at Ø3.0, the two FLR pegs at Ø2.4, as ORDERING.md states
        self.assertEqual({d: len(p) for d, p in np_holes.items()}, {3.0: 3, 2.4: 2})
        self.assertIn('NPTH holes are three Ø3.0 mm', ORDERING.read_text(encoding='utf-8'))

    def test_edge_cuts_extent_fits_the_price_tier(self):
        """The Edge.Cuts outline spans <= 100.0 x 100.0 mm (JLCPCB's 4-layer price tier); it is the 74 x 100 mm bay board."""
        pts = [(x, y) for x, y, _ in gerber_points(self.files[self.layer_file['Edge.Cuts']])]
        w = max(x for x, _ in pts) - min(x for x, _ in pts); h = max(y for _, y in pts) - min(y for _, y in pts)
        self.assertLessEqual(w, 100.0); self.assertLessEqual(h, 100.0)
        self.assertAlmostEqual(w, B.BOARD_W, delta=0.5); self.assertAlmostEqual(h, B.BOARD_H, delta=0.5)

    def test_paste_layer_opens_every_smd_pad(self):
        """F.Paste (the stencil) has exactly one aperture for each pad of each fitted top-side SMD part (73: no more, no fewer)."""
        smd_pads = sum({'R': 2, 'C': 2, 'Q': 3, 'D': 2, 'L': 2}[r[0]] for r, p in B.PARTS.items() if p['mount'] == 'smd_top' and fitted(r))
        flashes = [p for p in gerber_points(self.files[self.layer_file['F.Paste']]) if p[2] == 3]
        self.assertEqual(len(flashes), smd_pads)

    def test_via_drills_are_the_boards_vias(self):
        """The PTH file's via drills (KiCad's ViaDrill tools) are the PB17 via sizes, and there is one hit per via of the routed board: a
        re-route (or a via added) without `$KPY pcb/make_fab.py` leaves the Gerbers behind the board and fails here."""
        pth = next(t for n, t in self.files.items() if n.endswith('-PTH.drl'))
        vias = via_holes(pth)
        self.assertEqual(sorted({d for _, _, d in vias}), VIA_DRILLS)
        self.assertEqual(len(vias), len(re.findall(r'^\s*\(via\b', BOARD.read_text(encoding='utf-8'), re.M)))

    def test_vias_are_tented(self):
        """Every via's drill is covered by solder mask on both sides: no mask opening overlaps the drill circle (as ORDERING.md orders
        tented vias). Geometric, from the mask Gerbers' flashes and the PTH via hits. The web is the mask between the drill's edge and the
        nearest opening; the thinnest one (F.Mask, a GND via whose ring lies in the next pad's opening, pad-to-mask clearance 0) is quoted in
        ORDERING.md."""
        pth = next(t for n, t in self.files.items() if n.endswith('-PTH.drl'))
        vias = via_holes(pth)
        self.assertGreater(len(vias), 50)
        webs = {}
        for L in ('F.Mask', 'B.Mask'):
            openings = gerber_flashes(self.files[self.layer_file[L]])
            self.assertGreater(len(openings), 100, L)
            webs[L] = sorted((min(flash_distance(o, x, y) for o in openings) - d / 2, x, y, d) for x, y, d in vias)
            self.assertEqual([w for w in webs[L] if w[0] <= 0], [], f'{L}: a mask opening reaches a via drill (web, x, y, drill)')
        web = min(w[0] for L in webs for w in webs[L])
        print(f'\ntented vias: minimum mask web over a drill {web:.3f} mm; F.Mask {webs["F.Mask"][0][0]:.3f} mm, B.Mask {webs["B.Mask"][0][0]:.3f} mm; '
              f'{sum(1 for w in webs["F.Mask"] if w[0] < 0.15)} F.Mask vias under 0.15 mm')
        self.assertGreaterEqual(web, 0.09)                      # the LP12 GND vias keep 0.095 mm: a thinner web is a regression
        self.assertIn(f'{web:.3f} mm', ' '.join(ORDERING.read_text(encoding='utf-8').split()), 'ORDERING.md quotes the minimum web')

    # --- brief step 1: the BOM ---
    def test_bom_lists_every_part_with_its_mpn(self):
        """Every PARTS ref is in exactly one PARTS row, with PARTS' MPN; a fitted part has an MPN; the DNP rows (J14 / J15, PB8) say DNP; the
        bare test pads and the holes are listed as no part."""
        seen = {}
        for row in self.bom:
            for ref in row['Refs'].split():
                self.assertNotIn(ref, seen, ref); seen[ref] = row
        self.assertEqual(set(seen), set(B.PARTS))
        for ref, row in seen.items():
            part = B.PARTS[ref]
            self.assertEqual(row['MPN'], part['mpn'], ref)
            self.assertEqual(int(row['Qty']), len(row['Refs'].split()), ref)
            self.assertEqual(row['Fit'], 'DNP' if part.get('dnp') else 'yes' if B.in_bom(ref) else 'no part', ref)
            if fitted(ref): self.assertTrue(part['mpn'].strip(), ref)
        self.assertEqual({r for r, row in seen.items() if row['Fit'] == 'DNP'}, {'J14', 'J15'})
        self.assertIn('EEE-FK1C101P', seen['C3']['MPN'])                  # PB16 / PB20: C3's part

    def test_bom_lists_the_plug_in_items(self):
        """The modules and fuses that plug into the board's parts (board_spec.BOM_EXTRAS) are rows of their own, naming what they plug into."""
        extra = [row for row in self.bom if not row['Refs']]
        self.assertEqual(len(extra), len(B.BOM_EXTRAS))
        for item, row in zip(B.BOM_EXTRAS, extra):
            self.assertEqual((row['MPN'], int(row['Qty'])), (item['mpn'], item['qty']))
            for ref in item['on']: self.assertIn(ref, row['Notes'])

    def test_bom_fuse_holder_row_is_clear(self):
        """B7 fix 1: the F1 / F2 row is the holder (one note, not repeated; no fuse rating in its Value), and the fuses are their own
        plug-in rows, 7.5 A into F1 and 5 A into F2."""
        row = next(r for r in self.bom if 'F1' in r['Refs'].split())
        self.assertEqual((row['Refs'].split(), row['Fit']), (['F1', 'F2'], 'yes'))
        self.assertIn('holder', row['Value']); self.assertNotIn(' A', row['Value'])
        self.assertEqual(row['Notes'].count('snap lock'), 1, row['Notes'])
        self.assertEqual(row['Notes'].count('hold it flat'), 1, row['Notes'])
        self.assertEqual(row['Notes'].count('holder'), 1, row['Notes'])
        fuses = {r['Notes']: r for r in self.bom if r['Fit'] == 'plug-in' and 'ATO blade fuse' in r['Value']}
        self.assertEqual(set(fuses), {'into F1', 'into F2'})
        self.assertIn('7.5 A', fuses['into F1']['Value']); self.assertIn('5 A', fuses['into F2']['Value'])

    def test_bom_pico_row_is_consistent(self):
        """B7 fix 1: the Pico is the user's own (E02). SC0915 is the Pico without headers, so the row must say the user solders the headers
        on - not call a headerless part number 'with headers'."""
        row = next(r for r in self.bom if r['Fit'] == 'plug-in' and 'Raspberry Pi Pico' in r['Value'])
        text = row['Value'] + ' ' + row['MPN']
        self.assertIn('owned', text); self.assertIn('E02', text)
        if 'SC0915' in text: self.assertIn('solder', text); self.assertIn('no headers', text)

    # the MPNs quoted from memory (every one is "verify before ordering") and the manufacturer the checklist names for each
    MEMORY_MPN_REFS = ['U1A', 'U1B', 'U2', 'U3L', 'U3P', 'U4L', 'U4P', 'J4', 'J5', 'U5A', 'U5B', 'D1', 'C2', 'C3', 'LED1', 'LED2', 'LED3',
                       *(f'R{i}' for i in range(1, 24))]

    def test_memory_mpns_are_marked_and_on_the_checklist(self):
        """B7 fix 1: every MPN quoted from memory says 'verify before ordering' in the BOM (the Samsung C2 and the Yageo resistors too), and the
        'Verify before ordering' checklist names the manufacturer of each such row."""
        rows = {ref: row for row in self.bom for ref in row['Refs'].split()}
        for ref in self.MEMORY_MPN_REFS: self.assertIn('verify before ordering', rows[ref]['MPN'], ref)
        text = ORDERING.read_text(encoding='utf-8')
        sec = ' '.join(text[text.index('## Verify before ordering'):].split())
        marked = [row for row in self.bom if 'verify before ordering' in row['MPN']]
        self.assertGreaterEqual(len(marked), 10)
        for row in marked: self.assertIn(row['MPN'].split()[0], sec, row['MPN'])
        for mfr in ('Samsung', 'Yageo', 'Sullins', 'Amphenol', 'Phoenix', 'Vishay', 'Lite-On', 'Littelfuse'): self.assertIn(mfr, sec, mfr)

    def test_position_file_leaves_out_dnp_and_bare_pads(self):
        with open(POS, newline='', encoding='utf-8') as f: refs = {row['Ref'] for row in csv.DictReader(f)}
        self.assertEqual(refs, {r for r in B.PARTS if fitted(r)})

    def test_position_file_matches_placement(self):
        """Every row is where board_spec.PLACEMENT puts the part: x = u, y = -v, the rotation mod 360 (all parts on the top side)."""
        with open(POS, newline='', encoding='utf-8') as f:
            for row in csv.DictReader(f):
                u, v, rot = B.PLACEMENT[row['Ref']]
                self.assertEqual(row['Side'], 'top', row['Ref'])
                for got, want in ((float(row['PosX']), u), (float(row['PosY']), -v), (float(row['Rot']) % 360, rot % 360)):
                    self.assertAlmostEqual(got, want, places=3, msg=row)

    def test_package_is_made_from_the_committed_board(self):
        """fab/SOURCE.sha256 (make_fab) holds the sha256 of the board, its DRC rules and project as they were when the package was made
        (LF line endings: the checkout may be CRLF): an edited board without `$KPY pcb/make_fab.py` fails here."""
        want = {name: h for h, name in (ln.split() for ln in (FAB / 'SOURCE.sha256').read_text(encoding='utf-8').splitlines())}
        got = {p.name: hashlib.sha256(p.read_bytes().replace(b'\r\n', b'\n')).hexdigest()
               for p in (BOARD, BOARD.with_suffix('.kicad_dru'), BOARD.with_suffix('.kicad_pro'))}
        self.assertEqual(got, want, 'the board changed after the fab package was made: run $KPY pcb/make_fab.py')

    # --- brief step 1: ORDERING.md (the brief's strings, the carry list's facts) ---
    def test_ordering_states_the_order(self):
        text = ORDERING.read_text(encoding='utf-8')
        for s in ('4 layers', '1 oz outer', '0.5 oz inner', 'stencil', 'reflow', 'trim', 'tented',
                  'JLCPCB', f'{B.PCB_T:g} mm', f'{B.BOARD_W:g} × {B.BOARD_H:g} mm', '≤ 100 × 100', '5 pcs', '$8', 'check on the order page',
                  'FLR', 'up-angled micro-USB', 'no D24V50F5', 'P03', 'foam tape', 'measure it first',
                  'M2.5 × 20 mm nylon F-F standoffs', 'M2.5 × 12 countersunk', 'M3 heat-set inserts', 'printed spacers',
                  'reflow SMD → inspect → iron THT', '≤ 1.4 mm', "user's OK"):
            self.assertIn(s, text, s)
        last = [ln for ln in text.splitlines() if ln.strip()][-1]
        self.assertIn('agents never order: the user places the order', last)

    def test_ordering_has_the_size_check_step(self):
        """B7 fix 1 (ruling PB23): the outline is not changed (73.9 x 100.0 mm on the line centre: Edge.Cuts u 0.1-74.0, LIO-229); the
        order page may read the line width (the Gerber image, 74.0 x 100.1), so ORDERING.md says what to check before paying and what to do
        if it shows 100.1."""
        flat = ' '.join(ORDERING.read_text(encoding='utf-8').split())
        w, h = f'{B.BOARD_W:g}', f'{B.BOARD_H:g}'
        us, vs = zip(*json.loads((FAB.parent / 'board_outline.json').read_text(encoding='utf-8'))['outline_uv'])
        cw, ch = max(us) - min(us), max(vs) - min(vs)          # on the Edge.Cuts line centre; the 0.1 mm line adds 0.1 to the image
        for s in (f'check the detected size reads {w} × {h} mm', 'the price is the ≤ 100 × 100 tier', 'before paying',
                  f'{cw:.1f} × {ch:.1f} mm on the centre', f'{cw + 0.1:.1f} × {ch + 0.1:.1f}', f'set the size to {w} × {h} or ask JLCPCB support',
                  "don't pay for a larger tier", 'no job file', '±0.2 mm'):
            self.assertIn(s, flat, s)
        self.assertLess(flat.index('check the detected size reads'), flat.index('Add a **stencil**'))      # next to the size line, not far below

    def test_ordering_recommends_green_mask_and_the_inspect_check(self):
        """B7 fix 1: green mask (the smallest mask dam, verify) instead of 'any colour'; the GND joints are inspected for solder wicking
        into the tented vias; the order-number mark stays at its default; the inner-layer order is given in case the page asks."""
        flat = ' '.join(ORDERING.read_text(encoding='utf-8').split())
        self.assertNotIn('Any (green', flat)
        for s in ('**Green** mask', 'smallest solder-mask dam (verify)', 'GND 0805 / SOT-23 joints for solder wicking into the tented vias',
                  'Order number mark', 'removing it costs extra', 'L2 = In1.Cu', '.g1', 'L3 = In2.Cu', '.g2'):
            self.assertIn(s, flat, s)

    def test_ordering_states_the_design_rules(self):
        """Carry (PB17): the minimum track / space and via sizes, and that no surcharge option is needed."""
        text = ORDERING.read_text(encoding='utf-8')
        w, c = B.DEFAULT_CLASS['width'], B.DEFAULT_CLASS['clearance']
        for s in (f'{w:g} / {c:g} mm', '6 / 6 mil', 'no surcharge',
                  *(f'{d:g} / {h:g} mm' for d, h in {*B.CLASS_VIA.values(), B.GND_VIA})):
            self.assertIn(s, text, s)

    def test_ordering_puts_the_j1_pinout_in_bold(self):
        """Carry (PB20): J1 pin 1 = GND, pin 2 = BAT+, the battery lead GND in cavity 1 - in bold."""
        self.assertEqual(B.J1_PINOUT, {'1': 'GND', '2': 'BAT+'})
        self.assertRegex(ORDERING.read_text(encoding='utf-8'), r'\*\*[^*]*J1[^*]*pin 1 = GND, pin 2 = BAT\+[^*]*GND in cavity 1[^*]*\*\*')

    def test_ordering_has_the_verify_checklist(self):
        """Carry items 3 / 4: the figures quoted from memory, each with its part number and parameter."""
        text = ORDERING.read_text(encoding='utf-8')
        sec = text[text.index('## Verify before ordering'):]
        for s in ('hole-wall copper', 'DRV8874', 'R_DS(on)', 'EEE-FK1C101P', 'ESR', 'ripple', 'EEU-FR1E471', 'G5LE-1', 'coil',
                  'encoder', 'VL53L0X', '3V3', 'JLC04161H-7628', 'silk'):
            self.assertIn(s, sec, s)

    def test_ordering_names_c3_and_the_dnp_rows(self):
        text = ORDERING.read_text(encoding='utf-8')
        for s in ('EEE-FK1C101P', 'J14', 'J15', 'DNP'): self.assertIn(s, text, s)

    # --- carry item 5: bring-up notes for B10 ---
    @needs_kicad()
    def test_bringup_notes_for_b10(self):
        text = BRINGUP.read_text(encoding='utf-8')
        for s in ('1.8 A', f'{B.DRV_PWM_HZ / 1e3:g} kHz', 'pin 1 = GND, pin 2 = BAT+', 'under the carrier'):
            self.assertIn(s, text, s)
        for ref in [r for r in B.PARTS if r.startswith(('LED', 'TP')) and B.UNDER_CARRIER(r)] + ['TP2']:
            self.assertIn(ref, text, ref)

    # --- the 1:1 PDF ---
    def test_pdf_prints_one_to_one(self):
        """Every page carries make_fab's SCALE_BAR_MM bar at that length in PDF units (KiCad's 0.0001 in user unit): the plot is 1:1."""
        data = PDF.read_bytes()
        self.assertTrue(data.startswith(b'%PDF'))
        pages = re.findall(rb'/Type\s*/Page[^s]', data)
        self.assertGreaterEqual(len(pages), 4)
        streams = [zlib.decompress(s) for s in re.findall(rb'stream\r?\n(.*?)\r?\nendstream', data, re.S)]
        unit_mm = 25.4e-4
        bars = 0
        for s in streams:
            for m in re.finditer(rb'([\d.]+) ([\d.]+) m\s+([\d.]+) ([\d.]+) l', s):
                x0, y0, x1, y1 = map(float, m.groups())
                if abs(y1 - y0) < 1e-3 and abs(abs(x1 - x0) * unit_mm - SCALE_BAR_MM) < 0.01: bars += 1; break
        self.assertGreaterEqual(bars, len(pages))

    # --- the STEP for B8 ---
    def test_step_holds_every_fitted_part_at_its_place(self):
        """Every fitted part is a component of the STEP (DNP rows, bare pads and holes are not), named by its reference, placed at
        (u, -v, board top) with its rotation: STEP x = u, y = -v (KiCad's frame, --user-origin 0x0mm), so robot = STEP + (BOARD_X0,
        BOARD_Y0, Z_PCB_BOTTOM). The parts with no KiCad model are envelopes."""
        e = step_entities(STEP.read_text(encoding='utf-8'))
        nauo = {i: re.match(r"NEXT_ASSEMBLY_USAGE_OCCURRENCE\('[^']*','([^']*)','[^']*',\s*#(\d+),\s*#(\d+)", s) for i, s in e.items()
                if s.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE')}
        refs = {m.group(1): (i, int(m.group(3))) for i, m in nauo.items() if m.group(1) in B.PARTS}
        self.assertEqual(set(refs), {r for r in B.PARTS if fitted(r)})
        pds = {int(re.search(r'#(\d+)\)$', s).group(1)): i for i, s in e.items() if re.match(r"PRODUCT_DEFINITION_SHAPE\(\s*'Placement'", s)}
        cdsr = {int(re.findall(r'#(\d+)', s)[1]): int(re.findall(r'#(\d+)', s)[0]) for i, s in e.items()
                if s.startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION')}
        def nums(i): return [float(x) for x in re.findall(r'-?[\d.]+(?:E[-+]?\d+)?', e[i].split('(', 2)[2])]
        def product_name(pd):
            pdf = int(re.findall(r'#(\d+)', e[pd])[0]); prod = int(re.findall(r'#(\d+)', e[pdf])[0])
            return re.match(r"PRODUCT\(\s*'([^']*)'", e[prod]).group(1)
        envelopes = set()
        for ref, (n, child) in refs.items():
            rr = cdsr[pds[n]]
            idt = int(re.search(r'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)', e[rr]).group(1))
            ax = int(re.findall(r'#(\d+)', e[idt])[1])
            pt, _, xdir = (int(k) for k in re.findall(r'#(\d+)', e[ax]))
            u, v, rot = B.PLACEMENT[ref]
            x, y, z = nums(pt)
            self.assertAlmostEqual(x, u, 3, ref); self.assertAlmostEqual(y, -v, 3, ref); self.assertAlmostEqual(z, B.PCB_T, delta=0.01)
            dx, dy, _ = nums(xdir)
            self.assertAlmostEqual(math.atan2(dy, dx) % (2 * math.pi), math.radians(rot) % (2 * math.pi), 3, ref)
            if 'envelope' in product_name(child): envelopes.add(ref)
        self.assertEqual(envelopes, {'C1', 'F1', 'F2', 'J1', 'J2', 'J3'})       # + C1: its stock model is 10 mm, the part up to 20 (PARTS h)

    def step_tops(self):
        """{ref: top z (mm, STEP frame: the board's bottom is z 0) of that component's solid} - skips the test without OCP."""
        try:
            from OCP.STEPCAFControl import STEPCAFControl_Reader
            from OCP.TDocStd import TDocStd_Document
            from OCP.TCollection import TCollection_ExtendedString
            from OCP.XCAFDoc import XCAFDoc_DocumentTool
            from OCP.TDF import TDF_LabelSequence
            from OCP.TDataStd import TDataStd_Name
            from OCP.Bnd import Bnd_Box
            from OCP.BRepBndLib import BRepBndLib
        except ImportError:
            self.skipTest('no OCP in this interpreter')
        doc = TDocStd_Document(TCollection_ExtendedString('fab'))
        r = STEPCAFControl_Reader(); r.SetNameMode(True); r.ReadFile(str(STEP)); r.Transfer(doc)
        st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
        comps = TDF_LabelSequence(); st.GetComponents_s(roots.Value(1), comps, False)
        top = {}
        for k in range(1, comps.Length() + 1):
            c, n = comps.Value(k), TDataStd_Name()
            if not c.FindAttribute(TDataStd_Name.GetID_s(), n): continue
            b = Bnd_Box(); BRepBndLib.AddOptimal_s(st.GetShape_s(c), b, False, False)
            top[n.Get().ToExtString()] = b.Get()[5]
        return top

    def test_step_envelopes_have_the_part_heights(self):
        """The envelope of a part with no KiCad model rises PARTS h over the board top (the FLR holders 30 mm with their fuses), and so
        does C1's (B7 fix 1: the stock model is 10 mm, PARTS h 20)."""
        top = self.step_tops()
        for ref in ('C1', 'F1', 'F2', 'J1', 'J2', 'J3'):
            self.assertAlmostEqual(top[ref], B.PCB_T + B.PARTS[ref]['h'], delta=0.02, msg=ref)

    # B7 fix 1 (item 4): where the STEP height (the KiCad model, or the envelope) and board_spec PARTS h differ by more than 0.2 mm. B8 takes
    # max(model, PARTS h); ORDERING.md "STEP heights" lists these refs. Lower: the STEP is the bare part (the XH plug and the modules in the
    # sockets are not in it; D1's PARTS h is the SMA's maximum); higher: the model is taller than PARTS h.
    STEP_LOWER = {'D1', *(f'J{i}' for i in range(6, 14)), 'U1A', 'U1B', 'U2', 'U3L', 'U3P', 'U4L', 'U4P'}
    STEP_HIGHER = {'U5A', 'U5B'}

    def test_step_heights_are_documented_for_b8(self):
        """Every fitted part's STEP height is within 0.2 mm of PARTS h except the parts ORDERING.md lists under 'STEP heights: max(model,
        PARTS h)' - lower (STEP_LOWER) or higher (STEP_HIGHER) - so a model or PARTS h change cannot leave B8 with an undocumented gap."""
        top = self.step_tops()
        seat = B.PCB_T
        lower = {r for r in top if r in B.PARTS and top[r] - seat < B.PARTS[r]['h'] - 0.2}
        higher = {r for r in top if r in B.PARTS and top[r] - seat > B.PARTS[r]['h'] + 0.2}
        self.assertEqual((lower, higher), (self.STEP_LOWER, self.STEP_HIGHER))
        text = ORDERING.read_text(encoding='utf-8')
        sec = ' '.join(text[text.index('### STEP heights'):text.index('## The board order')].split())
        self.assertIn('STEP heights: max(model, PARTS h)', sec)
        for ref in sorted(lower | higher | {'C1'}): self.assertRegex(sec, rf'\b{ref}\b', ref)

    def test_fab_files_stay_small(self):
        sizes = {p.name: p.stat().st_size for p in FAB.iterdir() if p.is_file()}
        self.assertLess(sizes[STEP.name], 10e6)
        self.assertLess(sum(sizes.values()), 12e6, sizes)


if __name__ == '__main__':
    unittest.main()

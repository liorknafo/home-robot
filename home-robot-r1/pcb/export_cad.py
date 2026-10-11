"""Bay B8: the routed board's data for the robot CAD -> pcb/board_cad.json (robot frame, mm).

Run with KiCad's Python:  $KPY pcb/export_cad.py        (reads pcb/r1_power_io.kicad_pcb; never writes it)

cad/build_robot.py imports the board itself from the fab STEP (pcb/fab/r1_power_io.step: robot = STEP + (BOARD_X0, BOARD_Y0, Z_PCB_BOTTOM))
and reads everything it places or checks around it from this file, so the CAD follows the board, not the spike positions:
 - frame:        board_spec's frame (robot x = u + BOARD_X0, y = BOARD_Y0 - v; board bottom / top z), the STEP offset and the lead trim;
 - outline_xy:   the Edge.Cuts outline (robot x, y) and its area; cutouts_xy;
 - holes:        the four M3 holes H1-H4 (pad drill Ø3.2) and clamp_holes H5 / H6 (Ø3.0 NPTH, ruling PB3);
 - tht_pads:     every plated pad of every fitted through-hole part, and the fuse holders' NPTH pegs (pin ''), with its copper bounding box
                 and drill: the leads that hang under the board (trimmed to <= LEAD_TRIM) - the ribs and the hold-down keep off them;
 - parts:        every footprint's courtyard box, PARTS h, mount, DNP and the under-carrier flag;
 - connectors:   every board_spec.EDGE_CONNECTORS ref: courtyard and F.Fab body boxes, centre, entry direction (board -> robot unit vector),
                 family, vertical or not, mated height (MATED_H), the wire bend over a vertical plug (CABLE_BEND_MM), the lay-flat reach
                 (MATED_REACH), the off-board reach and bend of a plug that stands out past the board edge (B12a: OFFBOARD_REACH /
                 OFFBOARD_BEND_MM, J3) with its far end (plug_end_xy, where the wires leave it), pins, under-carrier flag;
 - modules:      the plug-in modules on their socket pads (Pico, DRV_L, DRV_R, IMU): every socket pin, the module body (board_spec.MODULES on
                 its first socket), the module PCB underside z (board top + SOCKET_BODY_H + HEADER_H) and PARTS h;
 - motor_cables: B3's motor-cable model (exit on the motor face, the band beside it, the W x T section) for the CAD to re-assert;
 - pico_usb:     the Pico micro-USB receptacle face centre (robot x, y, z) and the way the plug goes in (out_xy: the plug leaves the board
                 along +y), from the Pico body's USB end and the receptacle of Raspberry Pi's Pico-R3.step (centred on the Pico's axis,
                 7.58 x 2.52 mm, 1.26 mm over the Pico PCB top to its centre).
pcb/tests/test_board_cad.py checks it against board_spec (placement parity) and board_outline.json."""
import json, sys
from pathlib import Path
import pcbnew
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcb import board_spec as B

D = Path(__file__).parent
PCB, OUT = D / 'r1_power_io.kicad_pcb', D / 'board_cad.json'
T = pcbnew.ToMM
LEAD_TRIM = 1.4                    # mm: THT leads trimmed to <= 1.4 mm under the board (spec, Mounting; ORDERING.md)
SOCKET_BODY_H = 8.5                # mm: the 2.54 mm female sockets' body (board_spec.SOCKET_MPN)
HEADER_H = 2.5                     # mm: the module's male header plastic, seated on the socket: module PCB underside = board top + 11
PICO_PCB_T = 1.0                   # mm: Raspberry Pi Pico datasheet (and Pico-R3.step: PCB y -0.5..0.5)
PICO_USB = dict(w=7.58, h=2.52, centre_over_pcb=1.26)   # micro-B receptacle, from Pico-R3.step (x 6.71..14.29, y 0.5..3.02)
DIRS = {'+u': (1.0, 0.0), '-u': (-1.0, 0.0), '+v': (0.0, -1.0), '-v': (0.0, 1.0)}   # board direction -> robot (x, y)
MODULE_FIRST = {'Pico': 'U1A', 'DRV_L': 'U3L', 'DRV_R': 'U4L', 'IMU': 'U2'}


def xy(u, v): return [round(x, 4) for x in B.board_to_robot(u, v)]


def rbox(x0, y0, x1, y1):
    """Board-frame box (u0, v0, u1, v1) -> robot box (x0, y0, x1, y1)."""
    (xa, ya), (xb, yb) = B.board_to_robot(x0, y0), B.board_to_robot(x1, y1)
    return [round(min(xa, xb), 4), round(min(ya, yb), 4), round(max(xa, xb), 4), round(max(ya, yb), 4)]


def bbox_mm(b): return (T(b.GetX()), T(b.GetY()), T(b.GetRight()), T(b.GetBottom()))


def area(poly): return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]))) / 2


def fab_body(fp):
    """Bounding box (board u, v) of the footprint's F.Fab drawings (the part body), or None."""
    bb = None
    for item in fp.GraphicalItems():
        if item.GetLayer() != pcbnew.F_Fab or item.GetClass() != 'PCB_SHAPE': continue
        b = bbox_mm(item.GetBoundingBox())
        bb = b if bb is None else (min(bb[0], b[0]), min(bb[1], b[1]), max(bb[2], b[2]), max(bb[3], b[3]))
    return bb


def main():
    board = pcbnew.LoadBoard(str(PCB))
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    assert set(fps) == set(B.PARTS), set(fps) ^ set(B.PARTS)
    # outline: the board's Edge.Cuts polygon set
    ps = pcbnew.SHAPE_POLY_SET()
    assert board.GetBoardPolygonOutlines(ps), 'Edge.Cuts is not a closed outline'
    assert ps.OutlineCount() == 1, ps.OutlineCount()
    o = ps.Outline(0)
    outline = [xy(T(o.CPoint(k).x), T(o.CPoint(k).y)) for k in range(o.PointCount())]
    cutouts = [[xy(T(ps.Hole(0, h).CPoint(k).x), T(ps.Hole(0, h).CPoint(k).y)) for k in range(ps.Hole(0, h).PointCount())] for h in range(ps.HoleCount(0))]
    net_area = area(outline) - sum(area(c) for c in cutouts)

    def hole(ref):
        fp = fps[ref]; big = max(fp.Pads(), key=lambda p: T(p.GetDrillSize().x))
        return dict(ref=ref, xy=xy(T(fp.GetPosition().x), T(fp.GetPosition().y)), d=round(T(big.GetDrillSize().x), 3),
                    kind='M3' if ref in ('H1', 'H2', 'H3', 'H4') else 'clamp', plated=big.GetAttribute() == pcbnew.PAD_ATTRIB_PTH)

    tht, parts = [], {}
    for ref in sorted(fps, key=lambda r: (r.rstrip('0123456789'), int(''.join(c for c in r if c.isdigit()) or 0))):
        fp, spec = fps[ref], B.PARTS[ref]
        cy = fp.GetCourtyard(pcbnew.F_CrtYd)
        parts[ref] = dict(courtyard_xy=rbox(*bbox_mm(cy.BBox())) if cy.OutlineCount() else None, h=spec['h'], mount=spec['mount'],
                          dnp=bool(spec.get('dnp')), under_carrier=B.UNDER_CARRIER(ref))
        if spec['mount'] != 'tht' or spec.get('dnp') or ref.startswith('H'): continue
        for p in fp.Pads():
            kind = p.GetAttribute()
            if kind not in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH): continue
            b = bbox_mm(p.GetBoundingBox())
            tht.append(dict(ref=ref, pin=p.GetNumber(), net=p.GetNetname(), xy=xy(T(p.GetPosition().x), T(p.GetPosition().y)),
                            bbox_xy=rbox(*b), drill=round(T(p.GetDrillSize().x), 3), plated=kind == pcbnew.PAD_ATTRIB_PTH))

    connectors = {}
    for ref, d in B.EDGE_CONNECTORS.items():
        fp = fps[ref]; fam = B.connector_family(ref); vertical = B.PARTS[ref]['fp'].endswith('_Vertical')
        cyb = bbox_mm(fp.GetCourtyard(pcbnew.F_CrtYd).BBox())
        c = rbox(*cyb)
        connectors[ref] = dict(courtyard_xy=c, centre_xy=[round((c[0] + c[2]) / 2, 4), round((c[1] + c[3]) / 2, 4)], body_xy=rbox(*fab_body(fp)),
                               entry=d, entry_xy=list(DIRS[d]), family=fam, vertical=vertical, mated_h=B.MATED_H[ref],
                               bend=B.CABLE_BEND_MM[fam] if vertical else 0.0, mated_reach=B.MATED_REACH.get(ref, 0.0),
                               offboard_reach=B.OFFBOARD_REACH.get(ref, 0.0), offboard_bend=B.OFFBOARD_BEND_MM if ref in B.OFFBOARD_REACH else 0.0,
                               pins_xy=[xy(T(p.GetPosition().x), T(p.GetPosition().y)) for p in sorted(fp.Pads(), key=lambda p: (len(p.GetNumber()), p.GetNumber())) if p.GetNumber()],
                               under_carrier=B.UNDER_CARRIER(ref))
        if ref in B.OFFBOARD_REACH:      # B12a: a right-angle header whose plug stands out past the board edge (J3): where its wires leave the plug
            bx = connectors[ref]['body_xy']; ex, ey = DIRS[d]
            face = (bx[2] if ex > 0 else bx[0]) if ex else (bx[3] if ey > 0 else bx[1])
            mid = [round((bx[1] + bx[3]) / 2, 4) if ex else round((bx[0] + bx[2]) / 2, 4)]
            connectors[ref]['plug_end_xy'] = ([round(face + ex * B.OFFBOARD_REACH[ref], 4)] + mid) if ex else (mid + [round(face + ey * B.OFFBOARD_REACH[ref], 4)])

    z_module = round(B.Z_PCB_TOP + SOCKET_BODY_H + HEADER_H, 3)
    modules = {}
    for name, first in MODULE_FIRST.items():
        spec = B.MODULES[name]
        socks = {}
        for sock in spec['rows']:
            pads = {p.GetNumber(): xy(T(p.GetPosition().x), T(p.GetPosition().y)) for p in fps[sock].Pads() if p.GetNumber()}
            socks[sock] = dict(sorted(pads.items(), key=lambda kv: int(kv[0])))
        modules[name] = dict(sockets=socks, body_xy=[xy(*B.local_to_board(B.PLACEMENT[first], p)) for p in B.rect_pts(spec['body'])],
                             pcb_underside_z=z_module, h=B.PARTS[first]['h'])
    # Pico micro-USB: on the Pico's axis (half way between its rows), at the body's USB end (MODULES body y0: the receptacle overhang)
    x0, y0, x1, y1 = B.MODULES['Pico']['body']
    face = xy(*B.local_to_board(B.PLACEMENT['U1A'], (B.PICO_ROW_SPACING / 2, y0)))
    pico_usb = dict(face_centre_xyz=face + [round(z_module + PICO_PCB_T + PICO_USB['centre_over_pcb'], 3)], out_xy=[0.0, 1.0],
                    width=PICO_USB['w'], height=PICO_USB['h'], note='plug pushed in along -y from the board left edge; centre height from Pico-R3.step')

    # B3 fix 1's motor-cable model (board_spec MOTOR_CABLE_EXIT / _W / _T) in robot terms: the CAD re-asserts it against its real cable route
    motor = {}
    for ref, (eu, ev, eh) in B.MOTOR_CABLE_EXIT.items():
        ex, ey = B.board_to_robot(eu, ev)
        motor[ref] = dict(exit_xyz=[ex, ey, round(B.Z_PCB_TOP + eh, 3)], band_y=sorted([ey, ey - (B.MOTOR_CABLE_W if ey > 0 else -B.MOTOR_CABLE_W)]),
                          w=B.MOTOR_CABLE_W, t=B.MOTOR_CABLE_T)
    out = dict(generated_by='pcb/export_cad.py', board='pcb/r1_power_io.kicad_pcb', step='pcb/fab/r1_power_io.step',
               frame=dict(board_x0=B.BOARD_X0, board_y0=B.BOARD_Y0, z_bottom=B.Z_PCB_BOTTOM, z_top=B.Z_PCB_TOP, thickness=B.PCB_T,
                          step_to_robot_mm=[B.BOARD_X0, B.BOARD_Y0, B.Z_PCB_BOTTOM], lead_trim_mm=LEAD_TRIM, socket_body_h=SOCKET_BODY_H,
                          header_h=HEADER_H, note='robot x = u + board_x0, y = board_y0 - v; STEP x = u, y = -v, z 0 = the laminate bottom'),
               outline_xy=outline, cutouts_xy=cutouts, outline_area_mm2=round(net_area, 2),
               holes=[hole(r) for r in ('H1', 'H2', 'H3', 'H4')], clamp_holes=[hole(r) for r in ('H5', 'H6')],
               tht_pads=tht, parts=parts, connectors=connectors, modules=modules, pico_usb=pico_usb, motor_cables=motor)
    OUT.write_text(json.dumps(out, indent=1))
    print(f'{OUT.name}: outline {len(outline)} points, {net_area:.1f} mm2; holes {len(out["holes"])} + clamps {len(out["clamp_holes"])}; '
          f'{len(tht)} THT pads; {len(connectors)} edge connectors; modules {", ".join(modules)}; Pico USB face {pico_usb["face_centre_xyz"]}')


if __name__ == '__main__':
    main()

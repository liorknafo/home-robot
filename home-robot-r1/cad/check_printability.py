"""Support-free FDM printability gate for every print-parts/*.stl (Prusa MK3S, PETG, 0.4 mm nozzle).

Run:  python cad/check_printability.py [STL_DIR] [OUT_JSON]      (trimesh + numpy only)
Defaults: ../print-parts and cad/printability.json. Exit code 1 when any part fails.

For each part the six axis-aligned orientations are tried and the best support-free one is kept:
  * bed fit 250 x 210 x 210 mm (either XY rotation)
  * flat bed contact: down-facing faces (n_z < -0.999) within 0.05 mm of the bed, >= 20 mm2 required
  * overhang: down-facing faces more than 45 deg from vertical (n_z < -0.7071) above the first 0.3 mm.
    Horizontal down-facing regions are classified first: a short bridge (every point within 5 mm of an
    edge that a wall supports from below, i.e. span <= 10 mm, and >= 50 % of its boundary supported)
    or a narrow ledge (every point within 1.5 mm of a supported edge) is allowed and reported as
    bridge_mm2. Everything else counts as unsupported overhang.
  * wall thickness (3-D): rays cast inward from face centres; minimum and area-weighted 1st percentile
  * in-layer wall width (dense section probe): the part is sliced in its print orientation every 0.4 mm;
    from points every 0.25 mm along every section edge a 2-D ray is cast inward to the opposite edge.
    This is what the slicer sees, and it catches slivers on long faces that a per-face probe misses.
    Thin samples (< 1.2 mm) are grouped per layer; a group that extends over >= 1.0 mm is a sliver
    (sharp-corner tips of less than 1.0 mm are tolerated: the slicer only rounds them off).
  * closed (watertight) and consistently wound mesh
Fails when: not watertight, does not fit, no flat face, unsupported overhang > 50 mm2,
1st-percentile wall < 1.2 mm (3 x 0.4 mm perimeters), or any in-layer sliver (< 1.2 mm over >= 1.0 mm).
"""
from pathlib import Path
import sys, json
import numpy as np
import trimesh

BED = (250., 210., 210.)
MIN_CONTACT, MAX_OVERHANG, MIN_WALL_P1 = 20., 50., 1.2
LAYER_STEP, EDGE_STEP, SLIVER_EXTENT = .4, .25, 1.0
COS45 = np.cos(np.radians(45)) + 1e-3          # strictly steeper than 45 deg
ORIENT = {'as exported': np.eye(3),
          'flip': np.diag([1., -1., -1.]),
          '+x down': np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float),
          '-x down': np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float),
          '+y down': np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),
          '-y down': np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)}


def components(n, pairs):
    """Connected components of n nodes given an (k,2) edge array (union-find)."""
    parent = np.arange(n)
    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    for a, b in pairs:
        ra, rb = find(a), find(b)
        if ra != rb: parent[ra] = rb
    return np.array([find(i) for i in range(n)])


def seg_dist(p, a, b):
    """Distance from points p (n,2) to segments a,b (m,2) -> (n,m)."""
    ab = b - a; L = (ab ** 2).sum(1); L[L == 0] = 1e-12
    t = np.clip(((p[:, None, :] - a[None]) * ab[None]).sum(-1) / L[None], 0, 1)
    q = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(p[:, None, :] - q, axis=-1)


def orientation(m, R):
    v = m.vertices @ R.T; z0 = v[:, 2].min(); size = v.max(0) - v.min(0)
    fn = m.face_normals @ R.T; fz = v[m.faces][:, :, 2] - z0; area = m.area_faces
    fits = bool(((size[0] <= BED[0] and size[1] <= BED[1]) or (size[1] <= BED[0] and size[0] <= BED[1])) and size[2] <= BED[2])
    contact = float(area[(fn[:, 2] < -.999) & (fz.max(1) < .05)].sum())
    over = (fn[:, 2] < -COS45) & (fz.min(1) > .3)
    flat = over & (fn[:, 2] < -.97)
    allowed = np.zeros(len(area), bool); regions = []
    idx = np.flatnonzero(flat)
    if len(idx):
        local = -np.ones(len(area), int); local[idx] = np.arange(len(idx))
        adj, adj_e = m.face_adjacency, m.face_adjacency_edges
        inside = flat[adj[:, 0]] & flat[adj[:, 1]]
        comp = components(len(idx), local[adj[inside]])
        cz = v[m.faces].mean(1)[:, 2] - z0
        for c in np.unique(comp):
            faces = idx[comp == c]; member = np.zeros(len(area), bool); member[faces] = True
            border = member[adj[:, 0]] ^ member[adj[:, 1]]
            sup_a, sup_b, total, sup_len = [], [], 0., 0.
            for (fa, fb), e in zip(adj[border], adj_e[border]):
                other = fb if member[fa] else fa
                p0, p1 = v[e[0]], v[e[1]]; L = float(np.linalg.norm(p1 - p0)); total += L
                if cz[other] < min(p0[2], p1[2]) - z0 - .05:      # neighbour face lies below: wall support
                    sup_a.append(p0[:2]); sup_b.append(p1[:2]); sup_len += L
            pts = np.vstack([v[m.faces[faces]].mean(1)[:, :2], v[m.faces[faces]].reshape(-1, 3)[:, :2]])
            maxd = float(seg_dist(pts, np.array(sup_a), np.array(sup_b)).min(1).max()) if sup_a else 1e9
            frac = sup_len / total if total else 0.
            ok = maxd <= 1.5 or (maxd <= 5.25 and frac >= .5)
            regions.append(dict(area_mm2=round(float(area[faces].sum()), 1), max_dist_to_support_mm=round(maxd, 2),
                                supported_boundary=round(frac, 2), allowed=bool(ok)))
            if ok: allowed[faces] = True
    unsupported = float(area[over & ~allowed].sum())
    return dict(size_mm=[round(float(s), 1) for s in size], fits_bed=fits, bed_contact_mm2=round(contact, 1),
                overhang_mm2=round(unsupported, 1), bridge_mm2=round(float(area[allowed].sum()), 1),
                overhang_total_mm2=round(float(area[over].sum()), 1),
                bridge_regions=[r for r in regions if r['area_mm2'] >= 1])


def wall_thickness(m, max_rays=7000, reach=30.):
    """Inward ray cast from face centres; returns (min, area-weighted 1st percentile, % area < 1.2 mm)."""
    rng = np.random.default_rng(1)
    faces = np.arange(len(m.faces))
    if len(faces) > max_rays: faces = rng.choice(faces, max_rays, replace=False, p=m.area_faces / m.area_faces.sum())
    tri = m.triangles; v0 = tri[:, 0]; e1 = tri[:, 1] - v0; e2 = tri[:, 2] - v0
    lo, hi = tri.min(1), tri.max(1)
    orig = m.triangles_center[faces] - m.face_normals[faces] * 1e-4; d = -m.face_normals[faces]
    order = np.lexsort(np.floor(orig / 12.).T)           # spatially coherent chunks
    dist = np.full(len(faces), np.inf)
    for ch in np.array_split(order, max(1, len(order) // 48)):
        o, dd = orig[ch], d[ch]
        end = o + dd * reach; blo = np.minimum(o, end).min(0) - 1e-3; bhi = np.maximum(o, end).max(0) + 1e-3
        cand = np.flatnonzero(np.all(hi >= blo, 1) & np.all(lo <= bhi, 1))
        if not len(cand): continue
        E1, E2, V0 = e1[cand][None], e2[cand][None], v0[cand][None]
        p = np.cross(dd[:, None, :], E2); det = (E1 * p).sum(-1)
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1. / det; s = o[:, None, :] - V0
            u = (s * p).sum(-1) * inv; q = np.cross(s, E1)
            w = (dd[:, None, :] * q).sum(-1) * inv; t = (E2 * q).sum(-1) * inv
            hit = (np.abs(det) > 1e-12) & (u >= -1e-9) & (w >= -1e-9) & (u + w <= 1 + 1e-9) & (t > 1e-3) & (t < reach)
        t = np.where(hit, t, np.inf); self_hit = cand[None, :] == faces[ch][:, None]
        dist[ch] = np.where(self_hit, np.inf, t).min(1)
    ok = np.isfinite(dist)
    if not ok.any(): return None, None, 0.
    dv, a = dist[ok], m.area_faces[faces][ok]; s = np.argsort(dv); dv, a = dv[s], a[s]
    p1 = dv[np.searchsorted(np.cumsum(a), .01 * a.sum())]
    return round(float(dv[0]), 2), round(float(p1), 2), round(float(100 * a[dv < MIN_WALL_P1].sum() / a.sum()), 2)


def section_widths(m, R, thin=MIN_WALL_P1):
    """Dense in-layer width probe in print orientation R. Returns (min width, sliver groups)."""
    v = m.vertices @ R.T; v = v - [0, 0, v[:, 2].min()]
    mm = trimesh.Trimesh(v, m.faces, process=False); fn = mm.face_normals
    wmin, groups = np.inf, []
    for z in np.arange(.2, v[:, 2].max() - .05, LAYER_STEP):
        segs, fi = trimesh.intersections.mesh_plane(mm, [0, 0, 1], [0, 0, z], return_faces=True)
        if len(segs) < 3: continue
        a, e = segs[:, 0, :2], segs[:, 1, :2] - segs[:, 0, :2]
        nxy = fn[fi][:, :2]; L = np.linalg.norm(nxy, axis=1)
        k = np.maximum(1, np.ceil(np.linalg.norm(e, axis=1) / EDGE_STEP)).astype(int)
        idx = np.repeat(np.arange(len(a)), k); t = np.concatenate([(np.arange(n) + .5) / n for n in k])
        keep = L[idx] > .3; idx, t = idx[keep], t[keep]            # near-horizontal facets: covered by the 3-D probe
        P = a[idx] + e[idx] * t[:, None]; D = -nxy[idx] / L[idx, None]; O = P + D * 1e-4
        w = np.full(len(P), np.inf)
        for c in range(0, len(P), 800):
            o, d = O[c:c + 800], D[c:c + 800]; ao = a[None] - o[:, None]
            den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
            with np.errstate(divide='ignore', invalid='ignore'):
                tt = (ao[..., 0] * e[None, :, 1] - ao[..., 1] * e[None, :, 0]) / den
                uu = (ao[..., 0] * d[:, None, 1] - ao[..., 1] * d[:, None, 0]) / den
            hit = (np.abs(den) > 1e-12) & (tt > 1e-3) & (uu >= -1e-9) & (uu <= 1 + 1e-9)
            w[c:c + 800] = np.where(hit, tt, np.inf).min(1)
        if np.isfinite(w).any(): wmin = min(wmin, float(w[np.isfinite(w)].min()))
        T = np.flatnonzero(w < thin)
        if not len(T): continue
        Q = P[T]; near = np.linalg.norm(Q[:, None] - Q[None], axis=-1) < 2.4 * EDGE_STEP
        comp = components(len(T), np.argwhere(np.triu(near, 1)))
        for c in np.unique(comp):
            C = Q[comp == c]; ext = float(np.linalg.norm(C.max(0) - C.min(0)))
            groups.append(dict(z_print_mm=round(float(z), 1), extent_mm=round(ext, 2), min_width_mm=round(float(w[T][comp == c].min()), 2),
                               at_print_xy=[round(float(x), 1) for x in C.mean(0)]))
    groups.sort(key=lambda g: -g['extent_mm'])
    return (round(wmin, 2) if np.isfinite(wmin) else None), groups


def check(path):
    m = trimesh.load(path, force='mesh')
    options = {k: orientation(m, R) for k, R in ORIENT.items()}
    best = max(options, key=lambda k: (options[k]['fits_bed'], options[k]['bed_contact_mm2'] >= MIN_CONTACT,
                                       -options[k]['overhang_mm2'], options[k]['bed_contact_mm2']))
    tmin, tp1, thin = wall_thickness(m)
    smin, groups = section_widths(m, ORIENT[best])
    slivers = [g for g in groups if g['extent_mm'] >= SLIVER_EXTENT]
    r = dict(part=path.stem, best_orientation=best, **options[best], watertight=bool(m.is_watertight),
             winding_consistent=bool(m.is_winding_consistent), min_wall_mm=tmin, wall_p1_mm=tp1,
             area_thinner_than_1p2_pct=thin, section_min_width_mm=smin, section_slivers=slivers[:5],
             section_thin_tips=len(groups) - len(slivers), triangles=int(len(m.faces)))
    issues = []
    if not (r['watertight'] and r['winding_consistent']): issues.append('mesh not watertight / inconsistent winding')
    if not r['fits_bed']: issues.append('does not fit the MK3S volume')
    if r['bed_contact_mm2'] < MIN_CONTACT: issues.append('no flat face on the bed')
    if r['overhang_mm2'] > MAX_OVERHANG: issues.append(f"{r['overhang_mm2']:.0f} mm2 unsupported overhang > 45 deg")
    if tp1 is None or tp1 < MIN_WALL_P1: issues.append(f'thin walls (1st pct {tp1} mm, min {tmin} mm)')
    if slivers: issues.append(f"in-layer sliver {slivers[0]['min_width_mm']} mm wide over {slivers[0]['extent_mm']} mm (print z {slivers[0]['z_print_mm']})")
    r['issues'] = issues; r['pass'] = not issues
    return r


def main(src=None, out=None):
    cad = Path(__file__).resolve().parent
    src = Path(src) if src else cad.parent / 'print-parts'; out = Path(out) if out else cad / 'printability.json'
    report = []
    for p in sorted(src.glob('*.stl')):
        r = check(p); report.append(r)
        print(f"{'OK  ' if r['pass'] else 'FAIL'} {r['part']:24s} {r['best_orientation']:12s} size {r['size_mm']} contact {r['bed_contact_mm2']:8.1f}"
              f" overhang {r['overhang_mm2']:6.1f} (bridges {r['bridge_mm2']:.1f}) walls min {r['min_wall_mm']} p1 {r['wall_p1_mm']} layer-min {r['section_min_width_mm']}"
              + ('' if r['pass'] else '  <- ' + '; '.join(r['issues'])), flush=True)
    failed = [r['part'] for r in report if not r['pass']]
    out.write_text(json.dumps(dict(printer='Prusa MK3S 250 x 210 x 210 mm, PETG, 0.4 mm nozzle',
        rules=dict(min_bed_contact_mm2=MIN_CONTACT, max_unsupported_overhang_mm2=MAX_OVERHANG, overhang_angle_from_vertical_deg=45,
                   ignore_first_mm=.3, bridge_max_span_mm=10, ledge_max_mm=1.5, min_wall_p1_mm=MIN_WALL_P1,
                   section_probe=dict(layer_step_mm=LAYER_STEP, edge_step_mm=EDGE_STEP, thin_below_mm=MIN_WALL_P1, sliver_min_extent_mm=SLIVER_EXTENT)),
        parts_checked=len(report), parts_failed=failed, all_pass=not failed, parts=report), indent=1))
    print(f'PRINTABILITY {"PASS" if not failed else "FAIL"}: {len(report) - len(failed)}/{len(report)} parts pass -> {out}')
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:3]))

"""Grid search of route.py's stage F (PB18): the pure part, no KiCad - the layers a net may use, A* over per-layer obstacle grids with
layer-change vias, and the found cell path turned into straight runs and via points. route.py rasterises the board into the grids
(grid_obstacles) and adds the result to it (add_grid_path); this module only searches.

A grid is a bytes / bytearray of w * h cells (row-major, cell (x, y) at index y * w + x), 1 = no copper centre of this net may sit there.
"""
import heapq

from pcb import board_spec as B

LAYERS = ('F.Cu', 'In2.Cu', 'B.Cu')       # the signal layers (In1.Cu is the GND plane, never routed)
OUTER = ('F.Cu', 'B.Cu')
# R4: these classes' widths are sized for 1 oz outer copper (In2 is 0.5 oz internal: a 1.0 mm Motor track there carries 0.7 A, not
# 2.1 A), so neither Freerouting (route.outer_only) nor stage F puts them on an inner layer. Nor does stage F change their layer: one
# through via carries about 0.94 A (Motor, 0.8 / 0.4 mm; copper.via_current_a, B6) against the 2.1 A of the track - three are needed
OUTER_CLASSES = ('Motor', 'Power')
VIA_COST = 25.0                           # grid steps a layer change costs
STEPS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]


def net_class(net):
    return next((name for name, c in B.NETCLASSES.items() if net in c['nets']), 'Default')


def layers_for(net):
    """The signal layers stage F may put `net` on: F.Cu / B.Cu only for OUTER_CLASSES (R4), all three for the rest."""
    return OUTER if net_class(net) in OUTER_CLASSES else LAYERS


def vias_allowed(net):
    """Whether stage F may change `net`'s layer with one via: not for OUTER_CLASSES (one via is too thin for their current)."""
    return net_class(net) not in OUTER_CLASSES


def search(grids, w, h, via_ok, starts, goals, layers, vias=True):
    """A* from any of `starts` to any of `goals` ((layer name, x, y) cells) over the given `layers` only. grids: {layer: grid} (the cell
    a track centre of the net may not use); via_ok: grid, 1 where a via of the net may go (every layer free for it, no pad, no In1
    void keep-out). A start / goal cell is free on its own layer even if its grid says no (the net's own pad or track is there); a
    layer change needs via_ok, at the ends too, and vias=True (vias=False: one layer from end to end, e.g. a THT pad's two layers are
    both starts). Returns [(layer name, x, y)] or None."""
    idx = {L: k for k, L in enumerate(layers)}
    g = [grids[L] for L in layers]
    starts = [(idx[L], x, y) for L, x, y in starts if L in idx]
    goals = {(idx[L], x, y) for L, x, y in goals if L in idx}
    if not starts or not goals: return None
    ends = set(starts) | goals
    targets = {(x, y) for _, x, y in goals}

    def free(l, x, y):
        return 0 <= x < w and 0 <= y < h and ((l, x, y) in ends or not g[l][y * w + x])

    def heur(x, y):   # octile distance to the nearest goal: admissible with several goals (ponytail: O(goals) per push)
        return min(max(abs(x - tx), abs(y - ty)) + 0.4142 * min(abs(x - tx), abs(y - ty)) for tx, ty in targets)
    dist, prev, pq = {}, {}, []
    for s in starts:
        dist[s] = 0.0
        heapq.heappush(pq, (heur(s[1], s[2]), 0.0, s))
    found = None
    while pq:
        _, d, s = heapq.heappop(pq)
        if d > dist.get(s, 1e18): continue
        if s in goals: found = s; break
        l, x, y = s
        for dx, dy, cost in STEPS:
            nx, ny = x + dx, y + dy
            if not free(l, nx, ny) or (dx and dy and not (free(l, x + dx, y) or free(l, x, y + dy))): continue
            t, nd = (l, nx, ny), d + cost
            if nd < dist.get(t, 1e18): dist[t] = nd; prev[t] = s; heapq.heappush(pq, (nd + heur(nx, ny), nd, t))
        if vias and via_ok[y * w + x]:
            for l2 in range(len(layers)):
                if l2 == l or not free(l2, x, y): continue
                t, nd = (l2, x, y), d + VIA_COST
                if nd < dist.get(t, 1e18): dist[t] = nd; prev[t] = s; heapq.heappush(pq, (nd + heur(x, y), nd, t))
    if not found: return None
    path = [found]
    while path[-1] in prev: path.append(prev[path[-1]])
    return [(layers[l], x, y) for l, x, y in reversed(path)]


def runs(path, res, a, c):
    """The cell path as [(layer, [(u, v), ...])] straight-segment runs (collinear cells merged; the first and last points moved onto the
    exact ends a and c) and [(u, v)] via points at the layer changes."""
    pts = [(L, x * res, y * res) for L, x, y in path]
    pts[0], pts[-1] = (pts[0][0], a[0], a[1]), (pts[-1][0], c[0], c[1])
    out, vias, run = [], [], [pts[0]]
    for q in pts[1:]:
        if q[0] != run[-1][0]:
            vias.append(run[-1][1:]); out.append(run); run = [(q[0],) + run[-1][1:]]
        else: run.append(q)
    out.append(run)
    res_runs = []
    for run in out:
        keep = [run[0]]
        for k in range(1, len(run) - 1):
            (ax, ay), (bx, by), (cx, cy) = keep[-1][1:], run[k][1:], run[k + 1][1:]
            if abs((bx - ax) * (cy - by) - (by - ay) * (cx - bx)) > 1e-9: keep.append(run[k])
        if len(run) > 1: keep.append(run[-1])
        res_runs.append((run[0][0], [p[1:] for p in keep]))
    return res_runs, vias

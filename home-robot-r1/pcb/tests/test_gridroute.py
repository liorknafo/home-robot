"""Stage F's grid search (pcb/gridroute.py, pure): the layers each class may use (R4) and the A* itself on small synthetic grids."""
import unittest

from pcb import board_spec as B, gridroute as G


def grids(w, h, walls):
    """{layer: bytearray}: walls = {layer: [(x, y), ...]} blocked cells."""
    out = {L: bytearray(w * h) for L in G.LAYERS}
    for L, cells in walls.items():
        for x, y in cells: out[L][y * w + x] = 1
    return out


class GridRouteTests(unittest.TestCase):
    def test_outer_classes_get_outer_layers_only(self):
        """R4: Motor and Power nets on F.Cu / B.Cu only; every other net may also use In2."""
        for cls in G.OUTER_CLASSES:
            for net in B.NETCLASSES[cls]['nets']: self.assertEqual(G.layers_for(net), ('F.Cu', 'B.Cu'), net)
        for net in ('SPARE3', 'SDA', '3V3', 'VMOT', '5V'): self.assertEqual(G.layers_for(net), ('F.Cu', 'In2.Cu', 'B.Cu'), net)

    def test_open_motor_link_never_proposes_inner_cells(self):
        """A wall across F.Cu and B.Cu, In2 open: a signal net crosses on In2; an open Motor link gets no path at all (it would need
        In2). With a gap in the F.Cu wall instead, the Motor path goes through it on F.Cu alone - no inner cell, no layer change."""
        w, h = 20, 12
        wall = [(10, y) for y in range(h)]
        g = grids(w, h, {'F.Cu': wall, 'B.Cu': wall})
        ok = bytes([1]) * (w * h)
        sig = G.search(g, w, h, ok, [('F.Cu', 2, 5)], [('F.Cu', 17, 5)], G.layers_for('SPARE3'), G.vias_allowed('SPARE3'))
        self.assertIsNotNone(sig)
        self.assertIn('In2.Cu', {L for L, _, _ in sig})
        self.assertIsNone(G.search(g, w, h, ok, [('F.Cu', 2, 5)], [('F.Cu', 17, 5)], G.layers_for('L_M1'), G.vias_allowed('L_M1')))
        g2 = grids(w, h, {'F.Cu': [c for c in wall if c[1] != 8], 'B.Cu': wall})
        mot = G.search(g2, w, h, ok, [('F.Cu', 2, 5)], [('F.Cu', 17, 5)], G.layers_for('L_M1'), G.vias_allowed('L_M1'))
        self.assertIsNotNone(mot)
        self.assertEqual({L for L, _, _ in mot}, {'F.Cu'})

    def test_motor_and_power_links_never_change_layer(self):
        """Stage F draws Motor / Power links on one outer layer, no via (one 0.8 / 0.4 mm via carries ~1.1 A of the 2.1 A): F.Cu closed
        by a wall, B.Cu open past it, the goal on B.Cu - a Motor / Power link whose only path needs the layer change gets none, a signal
        net changes layer there; with a single-layer path (B.Cu wall-free from end to end) the Motor link routes and needs no via."""
        w, h = 20, 12
        ok = bytes([1]) * (w * h)
        wall = [(10, y) for y in range(h)]
        g = grids(w, h, {'F.Cu': wall})
        a, c = [('F.Cu', 2, 5)], [('B.Cu', 17, 5)]
        for cls in G.OUTER_CLASSES:
            for net in B.NETCLASSES[cls]['nets']:
                self.assertFalse(G.vias_allowed(net), net)
                self.assertIsNone(G.search(g, w, h, ok, a, c, G.layers_for(net), G.vias_allowed(net)), net)
        self.assertTrue(G.vias_allowed('SPARE3'))
        sig = G.search(g, w, h, ok, a, c, G.layers_for('SPARE3'), G.vias_allowed('SPARE3'))
        self.assertEqual([L for L, _, _ in sig][0], 'F.Cu'); self.assertEqual(sig[-1][0], 'B.Cu')
        for net in ('L_M1', 'BAT+'):
            same = G.search(g, w, h, ok, [('F.Cu', 2, 5), ('B.Cu', 2, 5)], c, G.layers_for(net), G.vias_allowed(net))   # a THT start: both layers
            self.assertIsNotNone(same, net)
            self.assertEqual({L for L, _, _ in same}, {'B.Cu'}, net)
            self.assertEqual(G.runs(same, 0.1, (0.2, 0.5), (1.7, 0.5))[1], [], net)                                    # no via point

    def test_layer_change_needs_via_ok_even_at_the_ends(self):
        """No via where via_ok says no - also at the start / goal cell (a via in a pad): with via_ok 0 at the start cell the path must
        leave on its own layer first."""
        w, h = 12, 5
        g = grids(w, h, {'F.Cu': [(x, y) for x in range(4, 12) for y in range(h)]})     # F.Cu closed past x 3: only B.Cu reaches the goal
        ok = bytearray([1]) * (w * h)
        ok[2 * w + 1] = 0                                                                  # the start cell (1, 2): no via there
        p = G.search(g, w, h, bytes(ok), [('F.Cu', 1, 2)], [('B.Cu', 10, 2)], G.LAYERS)
        self.assertIsNotNone(p)
        change = next(i for i in range(1, len(p)) if p[i][0] != p[i - 1][0])
        self.assertNotEqual(p[change][1:], (1, 2))
        self.assertTrue(ok[p[change][2] * w + p[change][1]])

    def test_runs_merge_collinear_cells_and_hit_the_ends(self):
        path = [('F.Cu', 0, 0), ('F.Cu', 1, 0), ('F.Cu', 2, 0), ('F.Cu', 3, 1), ('B.Cu', 3, 1), ('B.Cu', 3, 2)]
        runs, vias = G.runs(path, 0.1, (0.01, 0.0), (0.3, 0.2))
        self.assertEqual(vias, [(0.30000000000000004, 0.1)])
        self.assertEqual(runs[0][0], 'F.Cu'); self.assertEqual(runs[0][1][0], (0.01, 0.0)); self.assertEqual(len(runs[0][1]), 3)
        self.assertEqual(runs[1][0], 'B.Cu'); self.assertEqual(runs[1][1][-1], (0.3, 0.2))


if __name__ == '__main__':
    unittest.main()

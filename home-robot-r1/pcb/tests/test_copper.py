import math, unittest
from pcb import copper as C

class CopperTests(unittest.TestCase):
    def test_ipc2221_reference_widths(self):
        self.assertAlmostEqual(C.ipc2221_width_mm(7.5, 1.0), 4.838, delta=0.01)
        self.assertAlmostEqual(C.ipc2221_width_mm(5.0, 1.0), 2.765, delta=0.01)
        self.assertAlmostEqual(C.ipc2221_width_mm(2.1, 1.0), 0.836, delta=0.01)
        self.assertAlmostEqual(C.ipc2221_width_mm(1.0, 0.5, internal=True), 1.563, delta=0.01)

    def test_current_is_the_inverse_of_width(self):
        for i in (0.5, 2.1, 5.0, 7.5):
            self.assertAlmostEqual(C.ipc2221_current_a(C.ipc2221_width_mm(i, 1.0), 1.0), i, places=6)

    def test_stacked_copper_is_rated_on_its_combined_area(self):
        """LIO-227: layers in parallel are rated on their combined cross-section, each scaled to its external equivalent
        (A_i (k_i / 0.048)^(1 / 0.725)), not by summing per-layer currents. One layer is exactly ipc2221_current_a."""
        one = C.capacity_a({'F.Cu': 3.0})
        self.assertAlmostEqual(one, C.ipc2221_current_a(3.0, 1.0), places=9)
        self.assertAlmostEqual(C.capacity_a({'In2.Cu': 2.0}), C.ipc2221_current_a(2.0, 0.5, internal=True), places=9)
        two = C.capacity_a({'F.Cu': 3.0, 'B.Cu': 3.0})
        self.assertAlmostEqual(two, C.ipc2221_current_a(6.0, 1.0), places=9)         # two 1 oz layers = one track of the summed width
        self.assertAlmostEqual(two, 8.77, delta=0.01)
        self.assertLess(two, 2 * one)                                                # the per-layer sum (10.6 A) is optimistic
        self.assertGreater(two, 7.5)                                                 # BAT+ on two outer layers
        # an internal 0.5 oz layer adds its area scaled by (0.024 / 0.048)^(1 / 0.725)
        a_eq = (3.0 + 2.0 * 0.5 * 0.5 ** (1 / 0.725)) / C.MIL * 1.378
        self.assertAlmostEqual(C.capacity_a({'F.Cu': 3.0, 'In2.Cu': 2.0}), 0.048 * C.RISE_C ** 0.44 * a_eq ** 0.725, places=9)
        self.assertLess(C.capacity_a({'F.Cu': 3.0, 'In2.Cu': 2.0}), one + C.capacity_a({'In2.Cu': 2.0}))
        self.assertEqual(C.capacity_a({}), 0.0)

    def test_stackup_is_the_ordered_default(self):
        self.assertEqual(C.STACKUP, ('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'))
        self.assertEqual(C.COPPER_OZ, {'F.Cu': 1.0, 'In1.Cu': 0.5, 'In2.Cu': 0.5, 'B.Cu': 1.0})
        self.assertEqual(C.GND_PLANE_LAYER, 'In1.Cu')

    def test_via_barrel_current(self):
        """B6 carry item 1: a through via's barrel - plating VIA_PLATING_MM round its drill - carries IPC-2221 internal current on its
        cross-section (pi x drill x plating): 0.8 / 0.4 mm about 0.94 A, Power 1.2 / 0.6 about 1.26 A at 10 degC; three Motor vias carry
        the 2.1 A of a motor track."""
        self.assertAlmostEqual(C.VIA_PLATING_MM, 0.020)
        area = math.pi * 0.4 * C.VIA_PLATING_MM / C.MIL ** 2
        self.assertAlmostEqual(C.via_current_a(0.4), 0.024 * C.RISE_C ** 0.44 * area ** 0.725, places=9)
        self.assertAlmostEqual(C.via_current_a(0.4), 0.941, delta=0.002)
        self.assertAlmostEqual(C.via_current_a(0.6), 1.262, delta=0.002)
        self.assertLess(C.via_current_a(0.3), C.via_current_a(0.4))
        self.assertGreaterEqual(3 * C.via_current_a(0.4), 2.1)

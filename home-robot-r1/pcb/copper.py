"""Copper stack-up of the ordered board and IPC-2221 widths (bay spec 2026-10-07 §2)."""
import math
STACKUP = ('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu')
COPPER_OZ = {'F.Cu': 1.0, 'In1.Cu': 0.5, 'In2.Cu': 0.5, 'B.Cu': 1.0}   # JLCPCB 4-layer default; confirm when ordering
GND_PLANE_LAYER = 'In1.Cu'
RISE_C = 10.0
MIL = 0.0254

def is_internal(layer):
    return layer.startswith('In')

def ipc2221_width_mm(current_a, oz, rise_c=RISE_C, internal=False):
    """Track width (mm) that carries current_a at rise_c on copper of weight oz (IPC-2221 chart fit)."""
    k = 0.024 if internal else 0.048
    area_mil2 = (current_a / (k * rise_c ** 0.44)) ** (1 / 0.725)
    return area_mil2 / (1.378 * oz) * MIL

def ipc2221_current_a(width_mm, oz, rise_c=RISE_C, internal=False):
    """Current (A) a track of width_mm carries at rise_c (inverse of ipc2221_width_mm)."""
    k = 0.024 if internal else 0.048
    area_mil2 = width_mm / MIL * 1.378 * oz
    return k * rise_c ** 0.44 * area_mil2 ** 0.725

def capacity_a(widths_by_layer, rise_c=RISE_C):
    """Summed IPC current of parallel copper: {layer: width_mm} -> A."""
    return sum(ipc2221_current_a(w, COPPER_OZ[l], rise_c, is_internal(l)) for l, w in widths_by_layer.items() if w > 0)


# Via barrels (bay B6, carry item 1). Hole-wall copper: IPC-6012 class 2 average plating, 20 um (JLCPCB builds to class 2; its own
# hole-wall figure is not in the repo - confirm when ordering, B7). The barrel is buried in FR-4, so it is rated as an internal conductor
# (IPC-2221 k = 0.024) on its cross-section pi x drill x plating: 0.8 / 0.4 mm about 0.94 A, 1.2 / 0.6 mm about 1.26 A at 10 degC.
# A via carries at most this in all (the current of every layer it joins shares the one barrel).
VIA_PLATING_MM = 0.020


def via_current_a(drill_mm, plating_mm=VIA_PLATING_MM, rise_c=RISE_C):
    """Current (A) a plated via barrel of drill_mm carries at rise_c (IPC-2221, internal, on the barrel's annular cross-section)."""
    area_mil2 = math.pi * drill_mm * plating_mm / MIL ** 2
    return 0.024 * rise_c ** 0.44 * area_mil2 ** 0.725

OZ_MM = 0.035        # mm of copper per oz/ft2: the stack-up's copper thickness is COPPER_OZ x OZ_MM (build_pcb)

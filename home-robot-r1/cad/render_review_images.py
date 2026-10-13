"""Regenerate the package review images (robot-assembled / robot-internal / robot-exploded.png).

Blender -b --factory-startup --python render_review_images.py -- GLB MANIFEST_JSON OUT_DIR
Modes follow the review page: 'internal' hides the cover group and P_deck; 'exploded' applies each
part's explode offset from component-manifest.json and hides cable routes.
"""
import bpy, sys, json, math
from pathlib import Path
from mathutils import Vector, Matrix

a = sys.argv[sys.argv.index('--') + 1:]
glb, manifest, outdir = a[0], json.loads(Path(a[1]).read_text()), Path(a[2])
info = {p['id']: p for p in manifest}
lin = lambda c: c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4


def setup():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=glb)
    fix = Matrix.Rotation(math.radians(-90), 4, 'X')   # trimesh Z-up data in a Y-up container
    for o in bpy.context.scene.objects:
        if o.parent is None: o.matrix_world = fix @ o.matrix_world
    bpy.context.view_layer.update()
    mats = {}
    for o in [o for o in bpy.context.scene.objects if o.type == 'MESH']:
        p = info.get(o.name.split('.')[0]); col = p['color'] if p else '#808080'
        if p and p.get('vertex_colors') and o.data.color_attributes:   # a render-detail part: its colours are per vertex (the GLB's COLOR_0)
            m = bpy.data.materials.new(p['id']); m.use_nodes = True; nt = m.node_tree; b = nt.nodes['Principled BSDF']
            vc = nt.nodes.new('ShaderNodeVertexColor'); vc.layer_name = o.data.color_attributes[0].name
            nt.links.new(vc.outputs['Color'], b.inputs['Base Color']); b.inputs['Roughness'].default_value = .5
            o.data.materials.clear(); o.data.materials.append(m); continue
        if col not in mats:
            m = bpy.data.materials.new(col); m.use_nodes = True; b = m.node_tree.nodes['Principled BSDF']
            b.inputs['Base Color'].default_value = tuple(lin(int(col[i:i + 2], 16) / 255) for i in (1, 3, 5)) + (1,)
            b.inputs['Roughness'].default_value = .5
            if p and p['category'] == 'metal': b.inputs['Metallic'].default_value = .7; b.inputs['Roughness'].default_value = .3
            mats[col] = m
        o.data.materials.clear(); o.data.materials.append(mats[col])
    sc = bpy.context.scene; sc.render.engine = 'BLENDER_EEVEE_NEXT'; sc.view_settings.view_transform = 'Standard'
    sc.render.resolution_x, sc.render.resolution_y = 1600, 1200
    w = bpy.data.worlds.new('w'); sc.world = w; w.use_nodes = True
    w.node_tree.nodes['Background'].inputs[0].default_value = (.78, .8, .82, 1); w.node_tree.nodes['Background'].inputs[1].default_value = .45
    for rot, e in [((math.radians(35), math.radians(10), math.radians(40)), 2.4), ((math.radians(70), 0, math.radians(-130)), .8)]:
        L = bpy.data.objects.new('sun', bpy.data.lights.new('sun', 'SUN')); L.data.energy = e; L.data.angle = math.radians(8)
        L.rotation_euler = rot; sc.collection.objects.link(L)
    bpy.ops.mesh.primitive_plane_add(size=6, location=(0, 0, 0)); fl = bpy.context.active_object
    mf = bpy.data.materials.new('floor'); mf.use_nodes = True
    mf.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = (.36, .37, .39, 1); fl.data.materials.append(mf)
    return sc


def shoot(mode, name):
    sc = setup()
    for o in [o for o in sc.objects if o.type == 'MESH' and o.name != 'Plane']:
        p = info.get(o.name.split('.')[0])
        if not p: continue
        wire = p['id'].startswith('W') or p['id'].startswith('motor_cable')
        if mode == 'internal' and (p['group'] == 'cover' or p['id'] == 'P_deck'): o.hide_render = True
        if mode == 'exploded':
            if wire: o.hide_render = True
            o.location = o.location + Vector(p['explode']) / 1000
    bpy.context.view_layer.update()
    vis = [o for o in sc.objects if o.type == 'MESH' and o.name != 'Plane' and not o.hide_render]
    pts = [o.matrix_world @ Vector(c) for o in vis for c in o.bound_box]
    lo = Vector([min(p[i] for p in pts) for i in range(3)]); hi = Vector([max(p[i] for p in pts) for i in range(3)])
    ctr = (lo + hi) / 2; rad = (hi - lo).length / 2
    cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam')); sc.collection.objects.link(cam); sc.camera = cam
    cam.data.lens = 50; cam.data.clip_end = 20
    d = Vector((1.0, -0.85, 0.78)).normalized(); dist = rad / math.tan(cam.data.angle / 2) * (1.25 if mode == 'exploded' else 1.22)
    cam.location = ctr + d * dist; cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = str(outdir / name); bpy.ops.render.render(write_still=True)


for mode, name in [('assembled', 'robot-assembled.png'), ('internal', 'robot-internal.png'), ('exploded', 'robot-exploded.png')]:
    shoot(mode, name)
print('REVIEW IMAGES DONE')

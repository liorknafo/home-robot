"""Integrity checks for Blender exports consumed by rendering and collision."""
import json,unittest
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1]

class HouseAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene=json.loads((R/'Content/RobotData/house-scene.json').read_text())
        cls.assets=json.loads((R/'Content/RobotData/house-meshes.json').read_text())['assets']

    def test_every_scene_entry_has_a_renderable_mesh(self):
        self.assertEqual(self.scene['version'],3)
        for o in self.scene['objects']:
            self.assertIn(o['mesh'],self.assets)
            self.assertTrue(self.assets[o['mesh']]['parts'],o['name'])

    def test_valid_triangles_normals_and_uvs(self):
        for asset in self.assets.values():
            for p in asset['parts']:
                v=np.asarray(p['vertices']).reshape(-1,3);n=np.asarray(p['normals']).reshape(-1,3)
                uv=np.asarray(p['uv']).reshape(-1,2);t=np.asarray(p['triangles']).reshape(-1,3)
                self.assertEqual(len(v),len(n));self.assertEqual(len(v),len(uv))
                self.assertTrue(np.isfinite(v).all() and np.isfinite(n).all() and np.isfinite(uv).all())
                self.assertGreaterEqual(t.min(),0);self.assertLess(t.max(),len(v))
                area=np.linalg.norm(np.cross(v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]]),axis=1)
                self.assertTrue((area>1e-8).all(),asset['name'])
                self.assertTrue(np.allclose(np.linalg.norm(n,axis=1),1,atol=1e-4),asset['name'])

    def test_declared_bounds_cover_geometry(self):
        for o in self.scene['objects']:
            lo,hi=np.asarray(o['mesh_bounds_local_cm'])
            for p in self.assets[o['mesh']]['parts']:
                v=np.asarray(p['vertices']).reshape(-1,3)
                self.assertTrue((v>=lo-1e-4).all() and (v<=hi+1e-4).all(),o['name'])

    def test_all_surface_materials_are_available(self):
        for asset in self.assets.values():
            for p in asset['parts']:
                kind=p['surface']
                if kind=='terrazzo':continue
                self.assertTrue((R/'assets/house'/f'{kind}.png').exists(),kind)
                self.assertTrue((R/'Content/HouseMaterials'/f'M_{kind}.uasset').exists(),kind)

if __name__=='__main__':unittest.main()

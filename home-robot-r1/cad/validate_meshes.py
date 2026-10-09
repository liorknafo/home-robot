from pathlib import Path
import json,trimesh,numpy as np
R=Path(__file__).resolve().parents[1];out=[]
man={p['id']:p for p in json.loads((R/'cad/component-manifest.json').read_text())}
for p in sorted((R/'print-parts').glob('*.stl')):
 m=trimesh.load(p,force='mesh');e=man.get(p.stem,{}).get('export');c=None
 if e:   # undo the export translation, z turn and print flip (180 deg about x) to give the mass centre in the robot frame
  c=np.array(m.center_mass)-e['translate_mm'];a=-np.radians(e['rot_z_deg']);c=[c[0]*np.cos(a)-c[1]*np.sin(a),c[0]*np.sin(a)+c[1]*np.cos(a),c[2]]
  if e.get('rot_x_deg')==180:c=[c[0],-c[1],-c[2]]
 out.append(dict(file=p.name,watertight=bool(m.is_watertight),winding_consistent=bool(m.is_winding_consistent),volume_mm3=round(float(m.volume),1),size_mm=[round(float(v),3) for v in m.extents],
  center_mass_mm=[round(float(v),2) for v in c] if c is not None else None,area_mm2=round(float(m.area),1)))
(R/'cad/mesh-validation.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))

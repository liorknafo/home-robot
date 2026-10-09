"""Procedural physical-scale terrazzo and white paint. No tracking markers.
Terrazzo appearance guided by Israeli supplier/reference photographs, not tuned
against feature-tracker performance. This is a material approximation.
"""
from pathlib import Path
import cv2,numpy as np,json
R=Path(__file__).resolve().parents[1];rng=np.random.default_rng(103)
W,H=4096,3072;scale=W/800
image=np.full((H,W,3),(211,206,194),np.uint8)
# Small natural binder variation, below the contrast of the stone aggregate.
coarse=rng.normal(0,.7,(60,80)).astype(np.float32);variation=cv2.resize(coarse,(W,H),interpolation=cv2.INTER_CUBIC)
image=np.clip(image.astype(np.float32)+variation[:,:,None],0,255).astype(np.uint8)
palette=np.array([[232,228,216],[225,219,201],[195,190,176],[165,161,150],[145,134,118],[117,114,105],[176,160,138]],np.uint8)
N=165000
centers=rng.uniform([0,0],[W,H],(N,2));radii=rng.uniform(.18,.73,N)*scale;colors=rng.choice(len(palette),N,p=[.26,.22,.19,.14,.09,.025,.075])
for c,rad,col in zip(centers,radii,colors):
    count=int(rng.integers(4,8));angles=np.sort(rng.uniform(0,2*np.pi,count));radius=rad*rng.uniform(.6,1.2,count)
    points=np.rint(c+np.column_stack((np.cos(angles),np.sin(angles)))*radius[:,None]).astype(np.int32)
    cv2.fillConvexPoly(image,points,tuple(int(x) for x in palette[col]),lineType=cv2.LINE_AA)
# Traditional 30 cm square tiles; about 2 mm joints, no exaggerated dark grid.
for cm in range(0,801,30):
    x=round(cm*scale);cv2.line(image,(x,0),(x,H),(166,162,152),1,cv2.LINE_AA)
for cm in range(0,601,30):
    y=round(cm*scale);cv2.line(image,(0,y),(W,y),(166,162,152),1,cv2.LINE_AA)
cv2.imwrite(str(R/'assets/terrazzo-floor.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
# Warm off-white paint with broad, weak tonal variation and fine surface grain.
P=1024;low=cv2.resize(rng.normal(0,1.1,(12,12)).astype(np.float32),(P,P),interpolation=cv2.INTER_CUBIC)
paint=np.clip(np.array([231,231,228],float)+low[:,:,None]+rng.normal(0,.30,(P,P,1)),0,255).astype(np.uint8)
cv2.imwrite(str(R/'assets/white-paint.png'),cv2.cvtColor(paint,cv2.COLOR_RGB2BGR))
# Roughness reflects a clean, lightly polished finish, not mirror metal.
(R/'assets/surface-specification.json').write_text(json.dumps({'floor':{'type':'traditional Israeli terrazzo / balatot','tile_size_cm':30,'joint_mm':round(10/scale,3),'aggregate_diameter_cm_range':[.36,1.46],'polished_roughness':.30,'matte_roughness':.80,'metallic':0,'specular':.4,'coverage_cm':[800,600]},'walls_and_ceiling':{'type':'off-white matte paint','roughness':.87,'base_srgb':[231,231,228],'low_frequency_color_variation_sigma_8bit':1.1},'references':['https://www.ruhama-sharon.co.il/post/terrazzo-טרצו','https://www.alony.co.il/מגזין/טרצו_-_לא_מה_שחשבתם'],'limitations':['Appearance and material values are approximate, not measured from this home','Screen-space reflections omit off-screen objects','Materials are not features inserted to assist localization']},ensure_ascii=False,indent=2))
print('Generated 8 × 6 m terrazzo material and subtle white paint.')

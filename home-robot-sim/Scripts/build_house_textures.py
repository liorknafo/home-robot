"""Small original repeatable material textures; no vision markers or test inputs."""
from pathlib import Path
import numpy as np,cv2
R=Path(__file__).resolve().parents[1];D=R/'assets/house';D.mkdir(exist_ok=True);rng=np.random.default_rng(233);n=512;y,x=np.mgrid[:n,:n]/n

def save(name,rgb):cv2.imwrite(str(D/(name+'.png')),cv2.cvtColor(np.clip(rgb,0,255).astype(np.uint8),cv2.COLOR_RGB2BGR))

def mono(a):return np.repeat(a[:,:,None],3,axis=2)
noise=rng.normal(0,1,(n,n));broad=cv2.resize(rng.normal(0,1,(12,12)).astype(np.float32),(n,n),interpolation=cv2.INTER_CUBIC)
grain=np.sin(2*np.pi*(x*24+np.sin(y*6)*.8+np.sin(y*14+x*4)*.22));grain2=np.sin(2*np.pi*(x*100+np.sin(y*6)*2.9));wood=228+10*grain+4*grain2+2*noise
save('wood',mono(wood));weave=228+6*np.cos(2*np.pi*x*100)+6*np.cos(2*np.pi*y*100)+2*noise+1.5*broad;save('fabric',mono(weave))
for name,amp in [('paint',.7),('paper',1.2),('book',1.8),('cardboard',3),('rubber',1),('ceramic',.3),('metal',.7),('plastic',.6),('mirror',0),('fruit',1.2)]:save(name,mono(245+noise*amp+broad*amp))
save('soil',mono(170+22*broad+20*noise));save('leaf',mono(225+12*np.cos(x*30+y*16)+4*noise))
# Lettered wooden play blocks: ordinary household texture, not a fiducial.
toy=mono(wood).astype(np.uint8);cv2.putText(toy,'A',(155,355),cv2.FONT_HERSHEY_TRIPLEX,8,(135,91,71),16,cv2.LINE_AA);save('toy',toy)
# Abstract landscape print used for the existing three wall pictures.
a=np.zeros((512,512,3),np.uint8);a[:]=[219,211,190]
for pts,c in [([(0,280),(130,155),(270,325),(400,200),(512,320),(512,512),(0,512)],(167,174,148)), ([(0,350),(190,240),(310,345),(512,290),(512,512),(0,512)],(120,144,130)), ([(0,430),(280,310),(512,400),(512,512),(0,512)],(81,112,109))]:cv2.fillPoly(a,[np.array(pts)],c)
cv2.circle(a,(356,109),42,(193,144,104),-1,cv2.LINE_AA);a=np.clip(a.astype(float)+noise[:,:,None]*1.2,0,255);save('art',a)
print('Original surface textures written',len(list(D.glob('*.png'))))

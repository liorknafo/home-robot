"""Interchangeable feature extraction / matching, with explicit model selection."""
import cv2,numpy as np,time
class Features:
    def __init__(self,backend='orb',device=None):
        self.backend=backend;self.last_ms=0
        if backend=='orb':
            self.extractor=cv2.ORB_create(nfeatures=3000,fastThreshold=6,edgeThreshold=10);self.matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
        elif backend=='superpoint_lightglue':
            import torch
            from lightglue import SuperPoint,LightGlue
            self.torch=torch;self.device=device or ('cuda' if torch.cuda.is_available() else 'cpu')
            self.extractor=SuperPoint(max_num_keypoints=1024).eval().to(self.device)
            self.matcher=LightGlue(features='superpoint',flash=False,mp=False).eval().to(self.device)
        else:raise ValueError('Unknown feature backend')
    def extract(self,gray):
        t=time.perf_counter()
        if self.backend=='orb':
            k,d=self.extractor.detectAndCompute(gray,None);uv=np.array([x.pt for x in k],np.float32).reshape(-1,2)
            if d is None:d=np.empty((0,32),np.uint8)
        else:
            torch=self.torch
            with torch.inference_mode():
                o=self.extractor.extract(torch.from_numpy(gray.astype(np.float32)/255)[None].to(self.device))
                uv=o['keypoints'][0].cpu().numpy();d=o['descriptors'][0].cpu().numpy()
        self.last_ms=(time.perf_counter()-t)*1000
        return {'uv':uv,'descriptors':d,'size':np.array([gray.shape[1],gray.shape[0]],np.float32)}
    def match(self,a,b):
        if len(a['uv'])<2 or len(b['uv'])<2:return np.empty((0,2),int)
        if self.backend=='orb':
            matches=[]
            for pair in self.matcher.knnMatch(a['descriptors'],b['descriptors'],k=2):
                if len(pair)==2 and pair[0].distance<.75*pair[1].distance:matches.append((pair[0].queryIdx,pair[0].trainIdx))
            return np.array(matches,int).reshape(-1,2)
        torch=self.torch
        def convert(o):return {k:torch.from_numpy(o[v])[None].to(self.device) for k,v in [('keypoints','uv'),('descriptors','descriptors'),('image_size','size')]}
        with torch.inference_mode():out=self.matcher({'image0':convert(a),'image1':convert(b)})
        return out['matches'][0].cpu().numpy()

"""Stereo-only occupancy evidence, conservative footprint inflation and A*.
Unknown cells remain blocked. No house model or simulator depth is consulted.
"""
import math,heapq,json,cv2,numpy as np
from .stereo import optical_to_world
from .geometry import PLAN_RADIUS,BODY_CLEAR
class Grid:
    def __init__(self,resolution=.05,size=320):
        self.res=resolution;self.n=size;self.origin=np.array([-size*resolution/2]*2);self.logodds=np.zeros((size,size),np.float32);self.seen=np.zeros((size,size),np.uint16);self.last_pose=None;self.updates=0
        # Legacy stereo Brain: keeps its original .26 m inflation (PLAN_RADIUS + 1 cm for stereo
        # depth error); LidarGrid sets PLAN_RADIUS.
        self.radius=PLAN_RADIUS+.01
    def cell(self,xy):
        c=np.floor((np.asarray(xy)[:2]-self.origin)/self.res).astype(int);return int(c[1]),int(c[0])
    def world(self,cell):return self.origin+(np.array([cell[1],cell[0]])+.5)*self.res
    def valid(self,c):return 0<=c[0]<self.n and 0<=c[1]<self.n
    def ray(self,a,b):
        count=max(abs(b[0]-a[0]),abs(b[1]-a[1]))+1
        return np.rint(np.linspace(a,b,count)).astype(int)
    def integrate(self,points,pose):
        if self.last_pose is not None:
            d=np.linalg.norm(pose[:2]-self.last_pose[:2]);a=abs(math.atan2(math.sin(pose[2]-self.last_pose[2]),math.cos(pose[2]-self.last_pose[2])))
            if d<.035 and a<.09:return
        self.last_pose=pose.copy();self.updates+=1
        world=optical_to_world(points,pose);relative=world[:,:2]-pose[:2];dist=np.linalg.norm(relative,axis=1)
        # Reject points reconstructed below the real floor: common reflection artifacts.
        keep=(world[:,2]>-.025)&(world[:,2]<.27)&(dist>.20)&(dist<3.3);world,relative,dist=world[keep],relative[keep],dist[keep]
        angles=np.arctan2(relative[:,1],relative[:,0]);bins=np.rint(angles/.015).astype(int)
        free=set();occupied=set();start=self.cell(pose[:2])
        for b in np.unique(bins):
            ids=np.flatnonzero(bins==b)
            obstacles=ids[world[ids,2]>.035]
            if len(obstacles)>=2:
                order=obstacles[np.argsort(dist[obstacles])];idx=order[min(1,len(order)-1)];hit=True
            else:
                floor=ids[np.abs(world[ids,2])<.025]
                if len(floor)<2:continue
                idx=floor[np.argmax(dist[floor])];hit=False
            end=self.cell(world[idx,:2]);ray=self.ray(start,end)
            for c in ray[:-1 if hit else None]:
                t=tuple(c)
                if self.valid(t):free.add(t)
            if hit and self.valid(end):occupied.add(end)
        # Only the area physically occupied by the chassis is initially known clear.
        for iy in range(start[0]-2,start[0]+3):
            for ix in range(start[1]-2,start[1]+3):
                if self.valid((iy,ix)):free.add((iy,ix))
        for c in free-occupied:self.logodds[c]=max(-5,self.logodds[c]-.55);self.seen[c]=min(65535,int(self.seen[c])+1)
        for c in occupied:self.logodds[c]=min(5,self.logodds[c]+1.0);self.seen[c]=min(65535,int(self.seen[c])+1)
    def mark_obstacles(self,points,pose):
        world=optical_to_world(points,pose);distance=np.linalg.norm(world[:,:2]-pose[:2],axis=1)
        world=world[(world[:,2]>.035)&(world[:,2]<.27)&(distance<.6)]
        # Require repeated stereo samples in a cell before forcing a near obstacle.
        counts={}
        for point in world:
            cell=self.cell(point[:2]);counts[cell]=counts.get(cell,0)+1
        for cell,count in counts.items():
            if count>=2 and self.valid(cell):self.logodds[cell]=max(2.,self.logodds[cell]);self.seen[cell]=min(65535,int(self.seen[cell])+1)
    def masks(self):
        occupied=self.logodds>.8;free=self.logodds<-.8;unknown=~(occupied|free)
        # Exact Euclidean distances: the 5x5 approximation shortens diagonals by
        # 1%, which closed a measured 21 cm passage by a fraction of a millimetre.
        obstacle_clearance=cv2.distanceTransform((~occupied).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*self.res
        free_clearance=cv2.distanceTransform(free.astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*self.res
        safe=free&(obstacle_clearance>=self.radius)&(free_clearance>=self.radius)
        return free,occupied,unknown,safe,obstacle_clearance
    def path_step_cost(self,clearance):
        return 1+.05/max(.05,clearance)
    def traversable(self,start,free,safe,clearance):
        """Safe cells, plus a short exit when the start lies inside a margin.
        A new obstacle (a bump, a camera detection) can inflate a margin around
        the robot. The exit uses measured free cells within 45 cm that are no
        closer to an obstacle than the start (one cell of quantization), so
        leaving cannot approach anything; the local planner checks the sweep."""
        # Ground the robot has already driven over is passable however new
        # obstacle margins later overlap it, while it stays free and clear of
        # the round body (radius R + 8 mm = BODY_CLEAR) by the measured clearance.
        driven=getattr(self,'driven',None)
        if driven is not None:safe=safe|(driven&free&(clearance>=BODY_CLEAR))
        if not self.valid(start) or safe[start] or not free[start]:return safe
        y,x=start;r=int(round(.45/self.res));near=np.zeros_like(safe);near[max(0,y-r):y+r+1,max(0,x-r):x+r+1]=True
        return safe|(free&near&(clearance>=min(BODY_CLEAR,clearance[start]-.01)))
    def path(self,start_xy,goal_xy):
        free,_,_,safe,clearance=self.masks();start=self.cell(start_xy);goal=self.cell(goal_xy)
        if not(self.valid(start) and self.valid(goal)) or not safe[goal]:return []
        # Start can be slightly inside an uncertain margin, but may only exit
        # through cells that get no closer to obstacles (see traversable).
        if self.logodds[start]>.8:return []
        safe=self.traversable(start,free,safe,clearance)
        cost={start:0};parent={};queue=[(0,start)];end=None
        while queue:
            _,c=heapq.heappop(queue)
            if c==goal:end=c;break
            for dy,dx in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                n=(c[0]+dy,c[1]+dx)
                if not self.valid(n) or not safe[n]:continue
                if dy and dx and (not safe[c[0],n[1]] or not safe[n[0],c[1]]):continue
                nc=cost[c]+math.hypot(dx,dy)*self.path_step_cost(clearance[n])
                if nc<cost.get(n,math.inf):cost[n]=nc;parent[n]=c;heapq.heappush(queue,(nc+math.hypot(goal[0]-n[0],goal[1]-n[1]),n))
        if end is None:return []
        cells=[end]
        while cells[-1]!=start:cells.append(parent[cells[-1]])
        return [self.world(c) for c in reversed(cells)]
    def frontiers(self,pose,blacklist=()):
        free,occupied,unknown,safe,clearance=self.masks()
        # Disconnected proposals must not consume the bounded path-search budget.
        reachable=self.reachable(pose[:2],safe)
        # A frontier must touch unknown space through free space, rather than
        # merely being near unknown cells on the far side of a known wall.
        # Unknown cells that visits nearby repeatedly failed to reveal (under
        # furniture, behind an open door) no longer attract the robot.
        ignored=getattr(self,'unobservable',None)
        if ignored is not None:unknown=unknown&~ignored
        count,regions,stats,_=cv2.connectedComponentsWithStats(unknown.astype(np.uint8),8)
        substantial=np.zeros_like(unknown)
        for i in range(1,count):
            if stats[i,cv2.CC_STAT_AREA]>=12:substantial|=regions==i
        # Keep frontier cells 10 cm from measured obstacles. Gaps of a few
        # centimetres in a grazed wall or cabinet front otherwise expose the
        # whole unknown exterior as a frontier that no visit can reveal; a real
        # opening the robot or its LiDAR can use is wider than that.
        boundary=free&(cv2.dilate(substantial.astype(np.uint8),np.ones((3,3),np.uint8))>0)&~(cv2.dilate(occupied.astype(np.uint8),np.ones((5,5),np.uint8))>0)
        count,labels,stats,_=cv2.connectedComponentsWithStats(boundary.astype(np.uint8),8)
        proposals=[]
        for i in range(1,count):
            if stats[i,cv2.CC_STAT_AREA]<3:continue
            edge=np.argwhere(labels==i)
            candidates=reachable&(clearance>=self.radius+.025)&(cv2.dilate((labels==i).astype(np.uint8),np.ones((13,13),np.uint8))>0)
            # A narrow pocket (behind an open door) may have no safe cell within
            # 30 cm of its edge; look from up to 60 cm away (LiDAR sees far).
            if not candidates.any():candidates=reachable&(clearance>=self.radius+.025)&(cv2.dilate((labels==i).astype(np.uint8),np.ones((25,25),np.uint8))>0)
            cells=np.argwhere(candidates)
            if not len(cells):continue
            order=np.argsort(np.linalg.norm(cells-np.array(self.cell(pose[:2])),axis=1))
            for j in order[np.linspace(0,len(order)-1,min(12,len(order)),dtype=int)]:
                cell=tuple(cells[j]);xy=self.world(cell)
                if np.linalg.norm(xy-pose[:2])<.35 or any(np.linalg.norm(xy-p)<.35 for p in blacklist):continue
                closest=edge[np.argmin(np.linalg.norm(edge-np.array(cell),axis=1))];ray=self.ray(cell,closest)
                if occupied[ray[:,0],ray[:,1]].any():continue
                y,x=cell;y0,y1=max(0,y-15),min(self.n,y+16);x0,x1=max(0,x-15),min(self.n,x+16)
                # Cap the gain (~0.3 m2): beside a wall the window also counts the
                # unknown exterior, and an uncapped gain sent the robot across the
                # house past nearer frontiers again and again.
                value=min(120,int(substantial[y0:y1,x0:x1].sum()))*min(1.,clearance[cell]/.6);distance=np.linalg.norm(xy-pose[:2]);proposals.append((value/(1+distance),xy,value))
        choices=[];considered=[]
        # Bound expensive path searches. Nearby proposals describe the same view.
        for score,xy,value in sorted(proposals,key=lambda p:-p[0]):
            if any(np.linalg.norm(xy-p)<.25 for p in considered):continue
            considered.append(xy)
            route=self.path(pose[:2],xy)
            # Rank by the length of the drive, not the straight-line distance.
            if route:choices.append((value/(1+float(np.linalg.norm(np.diff(np.asarray(route),axis=0),axis=1).sum())),xy,route))
            if len(choices)>=5 or len(considered)>=16:break
        return choices
    def reachable(self,start_xy,safe=None):
        free,_,_,all_safe,clearance=self.masks()
        if safe is None:safe=all_safe
        start=self.cell(start_xy);result=np.zeros_like(safe)
        if not self.valid(start) or self.logodds[start]>.8:return result
        # Four-connected components match A*'s prohibition on cutting corners.
        traversable=self.traversable(start,free,safe,clearance).copy();traversable[start]=True
        _,labels=cv2.connectedComponents(traversable.astype(np.uint8),connectivity=4)
        return (labels==labels[start])&safe
    def summary(self):
        free,occupied,unknown,safe,_=self.masks();observed=self.seen>0
        p=1/(1+np.exp(-self.logodds));entropy=-(p*np.log2(np.clip(p,1e-8,1))+(1-p)*np.log2(np.clip(1-p,1e-8,1)))
        return dict(observed_area_m2=float(observed.sum()*self.res**2),free_area_m2=float(free.sum()*self.res**2),uncertain_observed_fraction=float((observed&unknown).sum()/max(1,observed.sum())),mean_observed_entropy_bits=float(entropy[observed].mean()) if observed.any() else 1.,updates=self.updates)
    def save(self,folder):
        folder.mkdir(parents=True,exist_ok=True);np.savez_compressed(folder/'occupancy.npz',logodds=self.logodds,seen=self.seen,res=self.res,origin=self.origin)
        (folder/'map-summary.json').write_text(json.dumps(self.summary(),indent=2))
    def load(self,folder):
        d=np.load(folder/'occupancy.npz',allow_pickle=False);self.logodds=d['logodds'];self.seen=d['seen'];self.res=float(d['res']);self.origin=d['origin'];self.n=self.logodds.shape[0]
    def view_bounds(self,pose=None,goal=None):
        cells=np.argwhere(self.seen>0)
        extra=[self.cell(x[:2]) for x in (pose,goal) if x is not None]
        if extra:cells=np.concatenate((cells,np.array(extra)),axis=0)
        if len(cells)==0:return [100,100,120,120]
        lo=cells.min(axis=0);hi=cells.max(axis=0);side=int(min(self.n,max(100,int(max(hi-lo))+24)));center=(lo+hi)/2
        x=int(np.clip(center[1]-side/2,0,self.n-side));y=int(np.clip(self.n-1-center[0]-side/2,0,self.n-side))
        return [x,y,side,side]
    def image(self,path=(),pose=None,goal=None):
        free,occ,unknown,safe,_=self.masks();im=np.full((self.n,self.n,3),138,np.uint8);im[free]=[224,234,228];im[occ]=[49,57,49];im[safe]=[230,244,235]
        def px(xy):y,x=self.cell(xy);return x,self.n-1-y
        im=np.flipud(im).copy()
        if len(path)>1:cv2.polylines(im,[np.array([px(p) for p in path],np.int32)],False,(82,152,213),2)
        if goal is not None:cv2.circle(im,px(goal),4,(32,124,223),1)
        if pose is not None:
            a=px(pose[:2]);b=px(pose[:2]+np.array([math.cos(pose[2]),math.sin(pose[2])])*.3);cv2.circle(im,a,3,(125,95,23),-1);cv2.line(im,a,b,(125,95,23),2)
        return im

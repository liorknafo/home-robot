# Living room group: upholstered sofa, piped cushions, woven rug, coffee table,
# TV console and television, woven blanket basket, books, mug and magazines.
import bmesh
from mathutils import Vector,Matrix

def living_hex(c,f):
    # derived shade of a hex colour
    return '#'+''.join('%02x'%max(0,min(255,round(int(c[i:i+2],16)*f))) for i in (1,3,5))

def living_orient(g):
    # flip a closed shell so faces point outward (positive signed volume)
    V,F=g;vol=0
    for f in F:
        a=V[f[0]]
        for i in range(1,len(f)-1):
            b=V[f[i]];c=V[f[i+1]];vol+=a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0])
    return (V,[tuple(reversed(f)) for f in F]) if vol<0 else (V,F)

def living_xf(g,fn):
    for p in g[0]:p[:]=fn(*p)
    return g

def living_move(g,d,yaw=0):
    a=math.radians(yaw);c,s=math.cos(a),math.sin(a)
    return living_xf(g,lambda x,y,z:(d[0]+x*c-y*s,d[1]+x*s+y*c,d[2]+z))

def living_fit(gs,lo,hi,axes=(0,1,2)):
    # scale/translate so the joint bounds equal the baseline envelope
    pts=[p for g in gs for p in g[0]]
    for i in axes:
        a=min(p[i] for p in pts);b=max(p[i] for p in pts)
        for p in pts:p[i]=lo[i]+(p[i]-a)*(hi[i]-lo[i])/(b-a)

def living_put(g,kind=None,color=None,weld=False,flat=(),name='Living part'):
    o=mesh(name,[tuple(p) for p in g[0]],g[1],kind,color,True)
    if weld:
        bm=bmesh.new();bm.from_mesh(o.data);bmesh.ops.remove_doubles(bm,verts=bm.verts,dist=1e-4);bm.to_mesh(o.data);bm.free()
        for p in o.data.polygons:p.use_smooth=True
    for i in flat:o.data.polygons[i].use_smooth=False
    return o

def living_rbox(size,r,k=(2,2,2),n45=2,crown={}):
    # rounded box: flat faces, round edges, sphere corners; crown={(axis,sign):cm} domes a face
    h=[s/2 for s in size];a=[max(1e-3,v-r) for v in h];V=[];F=[]
    S=[[-(a[i]+r*math.tan(math.pi/4*j/n45)) for j in range(n45,0,-1)]+[-a[i]+2*a[i]*j/k[i] for j in range(k[i]+1)]+[a[i]+r*math.tan(math.pi/4*j/n45) for j in range(1,n45+1)] for i in range(3)]
    for ax,u,w in [(0,1,2),(1,2,0),(2,0,1)]:
        for sg in (-1,1):
            b=len(V);nw=len(S[w])
            for pu in S[u]:
                for pw in S[w]:p=[0,0,0];p[ax]=sg*h[ax];p[u]=pu;p[w]=pw;V.append(p)
            for i in range(len(S[u])-1):
                for j in range(nw-1):q=(b+i*nw+j,b+(i+1)*nw+j,b+(i+1)*nw+j+1,b+i*nw+j+1);F.append(q if sg>0 else q[::-1])
    bump=lambda t:max(0,1-t*t)**2
    for p in V:
        c=[max(-a[i],min(a[i],p[i])) for i in range(3)];d=[p[i]-c[i] for i in range(3)];l=math.sqrt(sum(v*v for v in d)) or 1
        for i in range(3):p[i]=c[i]+d[i]/l*r
        for (ax,sg),amt in crown.items():
            if sg*p[ax]>a[ax]:
                o1,o2=[i for i in range(3) if i!=ax];p[ax]+=sg*amt*bump(p[o1]/a[o1])*bump(p[o2]/a[o2])
    return V,F

def living_pipe(pts,r,sides=6,cyclic=False,rb=1.0):
    # tube on rotation-minimising frames; r scalar or per point, rb flattens the section
    P=[Vector(p) for p in pts];n=len(P);R=r if isinstance(r,(list,tuple)) else [r]*n;T=[]
    for i in range(n):
        a=P[(i-1)%n] if cyclic or i>0 else P[i];b=P[(i+1)%n] if cyclic or i<n-1 else P[i];T.append((b-a).normalized())
    up=Vector((0,0,1)) if abs(T[0].z)<.9 else Vector((1,0,0));N=[(up-T[0]*up.dot(T[0])).normalized()]
    for i in range(1,n):N.append((N[-1]-T[i]*N[-1].dot(T[i])).normalized())
    if cyclic:
        m=(N[-1]-T[0]*N[-1].dot(T[0])).normalized();tw=math.atan2(T[0].dot(N[0].cross(m)),N[0].dot(m))
        N=[Matrix.Rotation(-tw*i/n,3,T[i])@N[i] for i in range(n)]
    V=[];F=[]
    for i in range(n):
        B=T[i].cross(N[i])
        for k in range(sides):a=2*math.pi*k/sides;V.append(list(P[i]+(N[i]*math.cos(a)+B*math.sin(a)*rb)*R[i]))
    for i in range(n if cyclic else n-1):
        j=(i+1)%n
        for k in range(sides):k2=(k+1)%sides;F.append((i*sides+k,i*sides+k2,j*sides+k2,j*sides+k))
    if not cyclic:
        # caps get their own vertices so they do not bend the side normals
        for i,sg in ((0,-1),(n-1,1)):
            b=len(V);V+=[list(V[i*sides+k]) for k in range(sides)]+[list(P[i])];c=b+sides
            for k in range(sides):k2=(k+1)%sides;F.append((c,b+k2,b+k) if sg<0 else (c,b+k,b+k2))
    return V,F

def living_loop(ax,c,ha,hb,rr,nc=6):
    # rounded-rectangle loop in the plane normal to axis ax (corner centres +-ha,+-hb)
    u,w=[(1,2),(2,0),(0,1)][ax];out=[]
    for q,(sa,sb) in enumerate([(1,1),(-1,1),(-1,-1),(1,-1)]):
        for j in range(nc+1):
            t=math.pi/2*(q+j/nc);p=[0,0,0];p[ax]=c;p[u]=sa*ha+rr*math.cos(t);p[w]=sb*hb+rr*math.sin(t);out.append(p)
    return out

def living_rrl(rings,nc=4):
    # stacked rounded rectangles (z,hx,hy,rc) closed by flat caps
    V=[];F=[];m=4*(nc+1)
    for zz,hx,hy,rc in rings:
        for q,(sx,sy) in enumerate([(1,1),(-1,1),(-1,-1),(1,-1)]):
            for j in range(nc+1):t=math.pi/2*(q+j/nc);V.append([sx*(hx-rc)+rc*math.cos(t),sy*(hy-rc)+rc*math.sin(t),zz])
    for s in range(len(rings)-1):
        for k in range(m):F.append((s*m+k,s*m+(k+1)%m,(s+1)*m+(k+1)%m,(s+1)*m+k))
    F+=[tuple(range(m-1,-1,-1)),tuple(range((len(rings)-1)*m,len(rings)*m))]
    return living_orient((V,F)),[len(F)-2,len(F)-1]

def living_fillet(poly,radii,nc=6):
    # replace polygon corners by tangent arcs
    out=[];n=len(poly)
    for i in range(n):
        r=radii[i];P=Vector((poly[i][0],poly[i][1],0))
        if r<=0:out.append(tuple(poly[i]));continue
        u1=(Vector((*poly[i-1],0))-P).normalized();u2=(Vector((*poly[(i+1)%n],0))-P).normalized();hf=u1.angle(u2)/2
        C=P+(u1+u2).normalized()*(r/math.sin(hf));T1=P+u1*(r/math.tan(hf));T2=P+u2*(r/math.tan(hf))
        a1=math.atan2(T1.y-C.y,T1.x-C.x);a2=math.atan2(T2.y-C.y,T2.x-C.x);d=(a2-a1+math.pi)%(2*math.pi)-math.pi
        out+=[(C.x+r*math.cos(a1+d*j/nc),C.y+r*math.sin(a1+d*j/nc)) for j in range(nc+1)]
    return out

def living_sweep(prof,c0,c1,r0=0,r1=0,m0=3,m1=3,ax=0,sharp=(),mid=0):
    # extrude a closed profile (a,b) along axis ax with rounded ends (+mid straight rings); returns geometry, flat faces, surface sampler
    n=len(prof);N=[];sgn=1 if sum(prof[i-1][0]*prof[i][1]-prof[i][0]*prof[i-1][1] for i in range(n))>0 else -1
    for i in range(n):
        p0,p1,p2=prof[i-1],prof[i],prof[(i+1)%n];e=[]
        for q0,q1 in ((p0,p1),(p1,p2)):dx,dy=q1[0]-q0[0],q1[1]-q0[1];l=math.hypot(dx,dy) or 1;e.append((sgn*dy/l,-sgn*dx/l))
        nx,ny=e[0][0]+e[1][0],e[0][1]+e[1][1];l=math.hypot(nx,ny) or 1;N.append((nx/l,ny/l))
    def at(k,s,th,off=0):
        r=r1 if s>0 else r0;t=math.radians(th);d=r*(1-math.cos(t))-off*math.cos(t)
        c=(c1-r+(r+off)*math.sin(t)) if s>0 else (c0+r-(r+off)*math.sin(t))
        a,b=prof[k][0]-N[k][0]*d,prof[k][1]-N[k][1]*d
        return [(c,a,b),(a,c,b),(a,b,c)][ax]
    th0=[90*(1-j/m0) for j in range(m0+1)] if r0>0 else [0];th1=[90*j/m1 for j in range(m1+1)] if r1>0 else [0]
    a0,a1=c0+max(r0,0),c1-max(r1,0)
    md=[[(c,p[0],p[1]),(p[0],c,p[1]),(p[0],p[1],c)][ax] for c in [a0+(a1-a0)*j/(mid+1) for j in range(1,mid+1)] for p in prof]
    V=[list(at(k,-1,t)) for t in th0 for k in range(n)]+[list(p) for p in md]+[list(at(k,1,t)) for t in th1 for k in range(n)]
    S=len(V)//n;F=[];flat=[]
    for s in range(S-1):
        for k in range(n):
            if k in sharp:flat.append(len(F))
            F.append((s*n+k,s*n+(k+1)%n,(s+1)*n+(k+1)%n,(s+1)*n+k))
    if r0<=0:flat.append(len(F))
    if r1<=0:flat.append(len(F)+1)
    F+=[tuple(range(n-1,-1,-1)),tuple(range((S-1)*n,S*n))]
    return living_orient((V,F)),flat,at

def living_spline(ctrl,n):
    # uniform Catmull-Rom through control points
    c=[ctrl[0]]+list(ctrl)+[ctrl[-1]];out=[]
    for i in range(n):
        t=i/(n-1)*(len(ctrl)-1);s=min(int(t),len(ctrl)-2);u=t-s;p0,p1,p2,p3=c[s],c[s+1],c[s+2],c[s+3]
        out.append([.5*((2*p1[j])+(-p0[j]+p2[j])*u+(2*p0[j]-5*p1[j]+4*p2[j]-p3[j])*u*u+(-p0[j]+3*p1[j]-3*p2[j]+p3[j])*u**3) for j in range(len(p1))])
    return out

def living_rr_at(hx,hy,rc,s):
    # point and outward normal at perimeter fraction s of a rounded rectangle (CCW from +x mid-side)
    sx,sy=hx-rc,hy-rc;q=math.pi/2*rc
    segs=[('l',(hx,0),(0,1),sy),('a',(sx,sy),0,q),('l',(sx,hy),(-1,0),2*sx),('a',(-sx,sy),90,q),('l',(-hx,sy),(0,-1),2*sy),('a',(-sx,-sy),180,q),('l',(-sx,-hy),(1,0),2*sx),('a',(sx,-sy),270,q),('l',(hx,-sy),(0,1),sy)]
    d=(s%1)*sum(g[3] for g in segs)
    for i,(kd,p,dd,L) in enumerate(segs):
        if d<=L or i==len(segs)-1:
            if kd=='l':return p[0]+dd[0]*d,p[1]+dd[1]*d,dd[1],-dd[0]
            a=math.radians(dd)+d/rc;return p[0]+rc*math.cos(a),p[1]+rc*math.sin(a),math.cos(a),math.sin(a)
        d-=L

# ---------------------------------------------------------------- sofa
C45=math.cos(math.pi/4)

def living_sofa_leg(o,index,size):
    # square collar under the frame, turned taper, brass ferrule
    x,y,z=size;h=z/2;hw=x/2-.01
    g,fl=living_rrl([(h-3.5,hw-.28,hw-.28,.35),(h-3.22,hw,hw,.6),(h,hw,hw,.6)]);living_put(g,'wood',col,flat=fl)
    lathe([(0,-4.7),(1.7,-4.7),(1.74,-4.5),(2.12,1.9),(2.3,2.55),(2.56,3.0),(2.48,3.38),(2.25,3.6),(0,3.6)],(1,1),'wood',col,24)
    lathe([(0,-h),(1.62,-h),(1.79,-h+.2),(1.87,-4.5),(1.8,-4.34),(0,-4.34)],(1,1),'metal','#9c8a63',24)

def living_sofa_base(o,index,size):
    # upholstered deck/plinth, front slightly sprung, welts on the front top edge and bottom
    x,y,z=size;r=3.6;e=(r+.12)*C45;a=[x/2-r,y/2-r,z/2-r]
    g=living_rbox((x,y,z),r,(3,14,2),2,{(0,1):.5})
    w1=living_pipe([(a[0]+e,-a[1],a[2]+e),(a[0]+e,a[1],a[2]+e)],.42,6)
    w2=living_pipe(living_loop(2,-(a[2]+e),a[0],a[1],e,5),.42,6,True)
    living_fit([g,w1,w2],(-x/2,-y/2,-z/2),(x/2,y/2,z/2))
    living_put(g,'fabric',col,True);dk=living_hex(col,.88);living_put(w1,'fabric',dk);living_put(w2,'fabric',dk)

def living_sofa_back(o,index,size):
    # raked inside back, crowned top, welted side panels and top seam
    x,y,z=size;hx,hz=x/2,z/2
    prof=living_fillet([(-hx,-hz),(hx,-hz),(hx,-hz+18),(hx-2.7,hz),(-hx,hz)],[1.5,1.5,10,6.5,6.5],6)
    g,fl,at=living_sweep(prof,-y/2,y/2,4,4,4,4,1)
    n=len(prof);off=.14
    w=[living_pipe([at(k,s,45,off) for k in range(n)],.42,6,True) for s in (-1,1)]
    k=24;w.append(living_pipe([at(k,-1,t,off) for t in (45,22.5,0)]+[at(k,1,t,off) for t in (0,22.5,45)],.42,6))
    living_fit([g]+w,(-hx,-y/2,-hz),(hx,y/2,hz))
    living_put(g,'fabric',col,flat=fl);dk=living_hex(col,.88)
    for p in w:living_put(p,'fabric',dk)

def living_sofa_arm(o,index,size):
    # track arm: crowned top, softly bulged sides, welted front panel
    x,y,z=size;hy,hz=y/2,z/2
    prof=living_fillet([(-hy,-hz),(hy,-hz),(hy+.3,0),(hy,hz),(-hy,hz),(-hy-.3,0)],[3.2,3.2,0,6.6,6.6,0],6)
    g,fl,at=living_sweep(prof,-x/2,x/2,1.0,3.6,2,4,0)
    w=living_pipe([at(k,1,45,.14) for k in range(len(prof))],.42,6,True)
    living_fit([g,w],(-x/2,-hy,-hz),(x/2,hy,hz))
    living_put(g,'fabric',col,flat=fl);living_put(w,'fabric',living_hex(col,.88))

def living_seat_cushion(o,index,size):
    # boxed seat cushion: domed top, sprung boxing, piping on both edges
    x,y,z=size;r=3.0;e=(r+.1)*C45;s=(x-.8,y-.8,z-1.6);a=[v/2-r for v in s]
    g=living_rbox(s,r,(6,6,1),2,{(2,1):1.1,(2,-1):.3,(0,1):.4,(0,-1):.4,(1,1):.4,(1,-1):.4})
    w=[living_pipe(living_loop(2,sg*(a[2]+e),a[0],a[1],e,6),.42,6,True) for sg in (1,-1)]
    living_fit([g]+w,(-x/2,-y/2,-z/2),(x/2,y/2,z/2))
    living_put(g,'fabric',col,True);dk=living_hex(col,.9)
    for p in w:living_put(p,'fabric',dk)

def living_back_cushion(o,index,size):
    # boxed back cushion leaning into the raked back, piped front and rear faces
    lo,hi=[-8.56,-30,-19],[8.56,30,19];r=3.4;e=(r+.1)*C45;s=(14.4,59.2,37.0);a=[v/2-r for v in s]
    g=living_rbox(s,r,(1,6,6),2,{(0,1):1.4,(0,-1):.5,(1,1):.3,(1,-1):.3,(2,1):.5,(2,-1):.2})
    w=[living_pipe(living_loop(0,sg*(a[0]+e),a[1],a[2],e,6),.42,6,True) for sg in (1,-1)]
    for p in [g]+w:living_xf(p,lambda x,y,z:(x-.045*z,y,z))
    living_fit([g]+w,lo,hi)
    living_put(g,'fabric',col,True);dk=living_hex(col,.9)
    for p in w:living_put(p,'fabric',dk)

def living_pillow(o,index,size):
    # knife-edge throw pillow with dog-ear corners and welt, leaning back on the cushions
    lo,hi=[-11.55,-18.03,-16],[11.55,18.03,16];W,H,T=36.5,30.5,13.0;nu,nv=22,20;pin=.07;tl=math.radians(22)
    def P(u,v,s):
        t=T/2*(1-abs(u)**2.6)**.5*(1-abs(v)**2.6)**.5*(1-.07*math.exp(-4*(u*u+v*v)))
        return [s*t,u*W/2*(1-pin*(1-v*v)),v*H/2*(1-pin*(1-u*u))]
    us=[math.sin(math.pi/2*(-1+2*i/nu)) for i in range(nu+1)];vs=[math.sin(math.pi/2*(-1+2*j/nv)) for j in range(nv+1)]
    V=[];F=[]
    for s in (1,-1):
        b=len(V);V+=[P(u,v,s) for u in us for v in vs]
        for i in range(nu):
            for j in range(nv):q=(b+i*(nv+1)+j,b+(i+1)*(nv+1)+j,b+(i+1)*(nv+1)+j+1,b+i*(nv+1)+j+1);F.append(q if s>0 else q[::-1])
    rim=[P(u,-1,0) for u in us[:-1]]+[P(1,v,0) for v in vs[:-1]]+[P(u,1,0) for u in us[::-1][:-1]]+[P(-1,v,0) for v in vs[::-1][:-1]]
    g=(V,F);w=living_pipe(rim,.45,6,True)
    for p in (g,w):living_xf(p,lambda x,y,z:(x*math.cos(tl)-z*math.sin(tl),y,x*math.sin(tl)+z*math.cos(tl)))
    living_fit([g,w],lo,hi)
    living_put(g,'fabric',col,True);living_put(w,'fabric',living_hex(col,.92))

# ---------------------------------------------------------------- rug
def living_rug(o,index,size):
    # flat-woven rug: weft ribs, tight end headers, rounded bound edges, channels under the border bands, knotted fringe
    x,y,z=size;hx,hy,hz=x/2,y/2,z/2;by=hy-1.0;cx,cy=o['center'][:2];rng=random.Random(91+index)
    bands=[(b['center'][0]-cx,b['center'][1]-cy,b['size'][0]/2+.2,b['size'][1]/2+.2) for b in scene['objects'] if b['name']=='Rug border' and abs(b['center'][0]-cx)<hx and abs(b['center'][1]-cy)<hy]
    def ht(px,py):
        h=hz-.06*(.5-.5*math.cos(math.pi*py));e=by-abs(py)
        if e<2.0:h=hz-.035+(h-hz+.035)*max(0,min(1,(e-1.6)/.4))
        for bx,bb,sx,sy in bands:
            d=max(abs(px-bx)-sx,abs(py-bb)-sy)
            if d<.4:h=min(h,.2+(h-.2)*max(0,d)/.4)
        de=min(hx-abs(px),by-abs(py));return -hz+(h+hz)*(.55+.45*math.sqrt(max(0,1-(1-min(1,de/.6))**2)))
    xs=[s*v for s in (-1,1) for v in (hx,hx-.07,hx-.25,hx-.6)]+[-60,-30,0,30,60]
    ys=[s*v for s in (-1,1) for v in (by,by-.07,by-.25,by-.6,by-1.6,by-2.0)]+[-by+j for j in range(int(2*by)+1)]
    for bx,bb,sx,sy in bands:
        if sy>sx:xs+=[bx-sx-.4,bx-sx,bx+sx,bx+sx+.4];ys+=[bb-sy-.4,bb-sy,bb+sy,bb+sy+.4]
        else:ys+=[bb-sy-.4,bb-sy,bb+sy,bb+sy+.4]
    def clean(v,lim):
        out=[]
        for t in sorted(t for t in v if abs(t)<=lim+1e-6):
            if not out or t-out[-1]>.03:out.append(t)
        return out
    xs=clean(xs,hx);ys=clean(ys,by);nx,ny=len(xs),len(ys)
    V=[[px,py,ht(px,py)] for py in ys for px in xs];F=[]
    for j in range(ny-1):
        for i in range(nx-1):F.append((j*nx+i,j*nx+i+1,(j+1)*nx+i+1,(j+1)*nx+i))
    ring=[i for i in range(nx)]+[j*nx+nx-1 for j in range(1,ny)]+[(ny-1)*nx+i for i in range(nx-2,-1,-1)]+[j*nx for j in range(ny-2,0,-1)]
    b0=len(V);V+=[[V[i][0],V[i][1],-hz] for i in ring]
    for k in range(len(ring)):k2=(k+1)%len(ring);F.append((ring[k],b0+k,b0+k2,ring[k2]))
    b1=len(V);V+=[[-hx,-by,-hz],[-hx,by,-hz],[hx,by,-hz],[hx,-by,-hz]];F.append((b1,b1+1,b1+2,b1+3))
    living_put((V,F),'fabric',col)
    # warp fringe: dense, slightly domed yarn ribbons leaving the rug end and lying on the floor
    fv,ff=[],[]
    for s in (-1,1):
        n=int((2*hx-3)/1.3)
        for i in range(n+1):
            xx=-hx+1.5+i*(2*hx-3)/n;dx=rng.uniform(-.5,.5);L=rng.uniform(1.95,2.3);b=len(fv)
            for px,py,pz,w in [(xx,by-.4,-.04,.5),(xx+dx*.15,by+.35,-.13,.55),(xx+dx*.55,by+1.2,-.2,.42),(xx+dx,by+L,-.215,.34)]:
                fv+=[[px-w/2,s*py,pz-.05],[px,s*py,pz+.03],[px+w/2,s*py,pz-.05]]
            for j in range(3):
                q=b+3*j;ff+=[(q,q+1,q+4,q+3),(q+1,q+2,q+5,q+4)] if s>0 else [(q,q+3,q+4,q+1),(q+1,q+4,q+5,q+2)]
    living_put((fv,ff),'fabric',living_hex(col,1.07))

def living_rug_border(o,index,size):
    # woven border band set into a channel of the rug; ends tuck under a crossing band
    x,y,z=size;along=1 if y>x else 0;L=max(x,y)/2;w=min(x,y)/2;bot,top=-.145,z/2;c=o['center']
    others=[b for b in scene['objects'] if b['name']=='Rug border' and b is not o]
    def under(s):
        px=c[0]+(s*L if along==0 else 0);py=c[1]+(s*L if along==1 else 0)
        return any(abs(px-b['center'][0])<b['size'][0]/2 and abs(py-b['center'][1])<b['size'][1]/2 for b in others)
    dip=[under(-1),under(1)];st=[]
    for s in (-1,1):
        st+=[(s*L,.12,dip[s>0]),(s*(L-.12),0,dip[s>0])]
        if dip[s>0]:st+=[(s*(L-3.2),0,True),(s*(L-3.6),0,False)]
    st=sorted(set(st))
    def prof(ins,tz):
        return [(-w+ins,bot),(w-ins,bot),(w-ins,tz-.05),(w-ins-.07,tz-.008),(w-ins-.22,tz),(-w+ins+.22,tz),(-w+ins+.07,tz-.008),(-w+ins,tz-.05)]
    V=[];F=[];n=8
    for sv,ins,dp in st:
        for u,hh in prof(ins,-.035 if dp else top):V.append([u,sv,hh] if along else [sv,u,hh])
    for i in range(len(st)-1):
        for k in range(n):F.append((i*n+k,i*n+(k+1)%n,(i+1)*n+(k+1)%n,(i+1)*n+k))
    F+=[tuple(range(n-1,-1,-1)),tuple(range((len(st)-1)*n,len(st)*n))]
    g=living_orient((V,F));living_put(g,'fabric',col,flat=[len(F)-2,len(F)-1])

# ---------------------------------------------------------------- coffee table
def living_coffee_leg(o,index,size):
    # tapered square leg with softened arrises, rising into the apron frame
    x,y,z=size;h=x/2
    g,fl=living_rrl([(-z/2,h*.715,h*.715,.3),(-z/2+.14,h*.76,h*.76,.38),(z/2+.45,h,h,.45)]);living_put(g,'wood',col,flat=fl)

def living_coffee_top(o,index,size):
    # slab with eased edges on a shallow apron frame that caps the legs; a wooden board under the book stack
    x,y,z=size;hz=z/2;cx,cy=o['center'][:2]
    slab=living_move(living_rbox((x,y,2.25),.45,(4,6,1),2),(0,0,hz-1.125));living_put(slab,'wood',col,True)
    legs=[b for b in scene['objects'] if b['name']=='Coffee table leg' and abs(b['center'][0]-cx)<x/2 and abs(b['center'][1]-cy)<y/2]
    ox=max(abs(b['center'][0]-cx)+b['size'][0]/2 for b in legs)+.05;oy=max(abs(b['center'][1]-cy)+b['size'][1]/2 for b in legs)+.05;t=4.1;zc=-hz-.46+1.125
    for s in (-1,1):
        box((0,s*(oy-t/2),zc),(2*ox,t,2.25),.25,'wood',col);box((s*(ox-t/2),0,zc),(t,2*(oy-t),2.25),.25,'wood',col)
    books=[b for b in scene['objects'] if b['name']=='Book on coffee table' and abs(b['center'][0]-cx)<x/2 and abs(b['center'][1]-cy)<y/2]
    if books:
        b=min(books,key=lambda q:q['center'][2]);bx,by_=b['center'][0]-cx,b['center'][1]-cy;bw,bl=b['size'][0]/2+3,b['size'][1]/2+3
        g,fl=living_rrl([(hz-.02,bw-.12,bl-.12,1.38),(hz+.1,bw,bl,1.5),(hz+.33,bw,bl,1.5),(hz+.45,bw-.12,bl-.12,1.38)],5)
        living_move(g,(bx,by_,0),b['yaw']);living_put(g,'wood',living_hex(col,.7),flat=fl)

def living_book_geo(L,W,T,ct,sr,oh,ne=6):
    # cover wrapped round a rounded spine (at -x) and a page block with a rounded back
    hx,hy,ht=L/2,W/2,T/2;cx=-hx+sr;tp=ht-ct-.03
    def ell(rx,rz,a0,a1):return [(cx+rx*math.cos(math.radians(a0+(a1-a0)*j/ne)),rz*math.sin(math.radians(a0+(a1-a0)*j/ne))) for j in range(ne+1)]
    cov=[(hx,-ht)]+ell(sr,ht,270,90)+[(hx,ht),(hx,ht-ct)]+ell(sr-ct,ht-ct,90,270)+[(hx,-ht+ct)]
    g1,f1,_=living_sweep(cov,-hy,hy,ax=1,sharp={0,ne+1,ne+2,ne+3,2*ne+4,2*ne+5})
    pg=[(hx-oh,-tp)]+ell(sr-ct-.03,tp,270,90)+[(hx-oh,tp)]
    g2,f2,_=living_sweep(pg,-hy+oh,hy-oh,ax=1,sharp={0,ne+1,ne+2})
    return [(g1,f1,'book'),(g2,f2,'paper')]

def living_book(o,index,size):
    # hardcover; the lowest book of a stack is thicker so it rests on the table board
    x,y,z=size;same=[b for b in scene['objects'] if b['name']==o['name'] and b['center'][:2]==o['center'][:2]]
    low=min(b['center'][2] for b in same)==o['center'][2] and len(same)>1;zl=-z/2-.4 if low else -z/2
    parts=living_book_geo(x,y,z/2-zl,.16,.55,.3);living_fit([p[0] for p in parts],(-x/2,-y/2,zl),(x/2,y/2,z/2))
    for g,fl,kd in parts:living_put(g,kd,col if kd=='book' else '#e8e2d2',flat=fl)

def living_mug(o,index,size):
    # straight-walled glazed mug: foot ring, rolled rim, coffee, D handle
    x,y,z=size;R=y/2-.05;zb,zt=-z/2+.45,z/2
    lathe([(0,zb+.06),(3.2,zb),(3.5,zb+.1),(3.72,zb+.38),(3.88,zb+1.0),(R,zb+2.2),(R,zt-.15),(R-.07,zt-.01),(R-.32,zt-.01),(R-.4,zt-.16),(R-.4,zb+1.5),(R-.95,zb+.72),(0,zb+.62)],(1,1),'ceramic',col,40)
    lathe([(R-.38,2.3),(0,2.3)],(1,1),'plastic','#4a3123',40)
    path=living_spline([(R-.2,0,2.95),(5.2,0,3.05),(6.35,0,2.1),(6.45,0,-.4),(5.8,0,-1.9),(R-.2,0,-2.25)],16)
    living_put(living_pipe(path,.38,8,False,1.6),'ceramic',col)

def living_magazines(o,index,size):
    # each layer is a few softcover magazines with slight rotations; the lowest layer reaches toward the floor
    x,y,z=size;same=[b for b in scene['objects'] if b['name']==o['name'] and b['center'][:2]==o['center'][:2]]
    low=min(b['center'][2] for b in same)==o['center'][2];zl=-z/2-.49 if low else -z/2;n=4 if low else 3
    rng=random.Random(17+index);t=(z/2-zl)/n;parts=[]
    for i in range(n):
        f=[1,.92,1.07,.97][i];pp=living_book_geo(x-.3,y-.3,t,.035,t/2,.06,4)
        for g,fl,kd in pp:living_move(g,(rng.uniform(-.12,.12),rng.uniform(-.12,.12),zl+t*(i+.5)),rng.uniform(-1.0,1.0))
        parts+=[(g,fl,kd,living_hex(col,f) if kd=='book' else '#ebe6da') for g,fl,kd in pp]
    living_fit([p[0] for p in parts],(-x/2,-y/2,zl),(x/2,y/2,z/2))
    for g,fl,kd,c in parts:living_put(g,kd,c,flat=fl)

# ---------------------------------------------------------------- TV console and television
def living_console_leg(o,index,size):
    # steel mounting plate, turned tapered leg, brass ferrule
    x,y,z=size;h=x/2-.01
    g,fl=living_rrl([(z/2-.02,h,h,.3),(z/2+.46,h,h,.3)]);living_put(g,'metal',living_hex(col,.75),flat=fl)
    lathe([(0,-4.6),(1.12,-4.6),(1.16,-4.45),(1.6,z/2-.4),(1.66,z/2),(0,z/2)],(1,1),'wood',col,24)
    lathe([(0,-z/2),(1.0,-z/2),(1.14,-z/2+.15),(1.22,-4.5),(1.17,-4.35),(0,-4.35)],(1,1),'metal','#9c8a63',24)

def living_console(o,index,size):
    # panel carcass (front brought forward 0.45 so the overlay drawer fronts sit on it); legs and fronts are separate
    x,y,z=size;hx,hy,hz=x/2,y/2,z/2;zb=-hz-.04;xc,xs=-.225,x+.45;zm,hm=(zb+2+hz-2.4)/2,hz-2.4-zb-2
    box((xc,0,hz-1.2),(xs,y,2.4),.35)
    box((xc,0,zb+1.0),(xs-.2,y-.2,2.0),.3)
    for s in (-1,1):box((xc,s*(hy-1.1),zm),(xs,2.0,hm),.3)
    box((xc+.05,0,zm),(xs-.1,y-4.2,hm),.12)

def living_slabs(boxes,bevel,kind=None,color=None):
    # several cuboids as ONE part (one grain direction across them), bevelled like box()
    bm=bmesh.new()
    for c,s in boxes:
        for v in bmesh.ops.create_cube(bm,size=1)['verts']:v.co=Vector([c[i]+v.co[i]*s[i] for i in range(3)])
    me=bpy.data.meshes.new('Paired panels');bm.to_mesh(me);bm.free();o=bpy.data.objects.new('Paired panels',me);bpy.context.collection.objects.link(o)
    b=o.modifiers.new('Rounded manufactured edges','BEVEL');b.width=bevel;b.segments=3;b.affect='EDGES';b.harden_normals=True
    n=o.modifiers.new('Face normals','WEIGHTED_NORMAL');n.keep_sharp=True;n.weight=40
    return keep(o,kind,color,True)

def living_drawer_front(o,index,size):
    # pair of overlay doors meeting on the centre line (grain runs on across both), slim brass bar pulls, dark reveal
    x,y,z=size;hx,hy,hz=x/2,y/2,z/2;g=.14;w=hy-g
    living_slabs([((0,s*(g+w/2),0),(x,w,z)) for s in (-1,1)],.2,'wood',col)
    box((.42,0,0),(.5,.2,z-.4),.02,'wood',living_hex(col,.32))
    for s in (-1,1):
        yy=s*2.4;bx=-hx-.27
        living_put(living_pipe([(bx,yy,zz) for zz in (1.0,1.12,1.3,10.7,10.88,11.0)],[.05,.15,.2,.2,.15,.05],8,False,1.5),'metal','#a08d62')
        for zz in (2.2,9.8):living_put(living_pipe([(-hx+.06,yy,zz),(bx,yy,zz)],.13,8),'metal','#a08d62')

def living_tv_frame(o,index,size):
    # slim panel, flush black glass border, rear electronics housing and two feet
    x,y,z=size;hx,hy,hz=x/2,y/2,z/2;zb=-hz+1.0
    living_put(living_move(living_rbox((1.9,y,hz-zb),.45,(1,8,6),2),(-hx+.95,0,(hz+zb)/2)),'plastic',col,True)
    scr=[b for b in scene['objects'] if b['name']=='Television screen' and abs(b['center'][1]-o['center'][1])<1]
    sy,sz=(scr[0]['size'][1]/2,scr[0]['size'][2]/2) if scr else (hy-4,hz-4);zc=(scr[0]['center'][2]-o['center'][2]) if scr else 0
    ox,oz0,oz1,iy,iz=hy-.42,zb+.42,hz-.42,sy+.05,sz+.05;xf,xb=-hx-.43,-hx+.05;V=[];F=[]
    for xx in (xf,xb):V+=[[xx,a,b] for a,b in [(ox,oz0),(-ox,oz0),(-ox,oz1),(ox,oz1),(iy,zc-iz),(-iy,zc-iz),(-iy,zc+iz),(iy,zc+iz)]]
    for i in range(4):
        j=(i+1)%4;F+=[(i,j,j+4,i+4),(8+i,8+i+4,8+j+4,8+j),(i,i+8,j+8,j),(i+4,j+4,j+12,i+12)]
    # black glass border in the same glass as the panel, so only the slim rim reads as bezel
    living_put(living_orient((V,F)),'plastic',living_hex(scr[0]['color'] if scr else col,.9),flat=range(len(F)))
    living_put(living_move(living_rbox((3.3,y*.67,48),1.4,(1,6,4),2),(hx-1.65,0,zb+3+24)),'plastic',living_hex(col,1.08),True)
    for s in (-1,1):
        yy=s*(hy-14);V=[[a,yy+b,-hz-.45] for a,b in [(-hx-.4,-.9),(hx+.4,-.9),(hx+.4,.9),(-hx-.4,.9)]]+[[a,yy+b,zb+.2] for a,b in [(-hx+.2,-.55),(-hx+1.7,-.55),(-hx+1.7,.55),(-hx+.2,.55)]]
        F=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
        ft=living_put(living_orient((V,F)),'metal',living_hex(col,1.25));b=ft.modifiers.new('Soft edge','BEVEL');b.width=.12;b.segments=2

# ---------------------------------------------------------------- blanket basket
def living_basket(o,index,size):
    # closed seagrass-style weave (stakes and alternating weavers), wrapped rim, foot band, folded blanket with a fold over the rim
    x,y,z=size;rc=4.0;A=.6;zb,zt=-18.0,11.6;R=14;bl='#ddd0b5'
    HX=lambda zz:16.0+.5*(zz-zb)/(zt-zb);HY=lambda zz:15.4+.5*(zz-zb)/(zt-zb)
    P=4*(HX(zt)-rc)+4*(HY(zt)-rc)+2*math.pi*rc;N=2*round(P/6.0);Ns=4*N;V=[]
    # three rings per weaver row: groove, crest band (bulging over / dipping under alternate stakes), groove
    fs=[(k,f) for k in range(R) for f in (0,1/3,2/3)]+[(R,0)]
    for k,f in fs:
        zz=zb+(zt-zb)*(k+f)/R
        for js in range(Ns):
            s=js/Ns;px,py,nx,ny=living_rr_at(HX(zz),HY(zz),rc,s);c=math.cos(2*math.pi*N*s+math.pi*k)
            d=A*(.05 if f==0 else 1-.5*max(0,-c)**1.5);V.append([px+nx*d,py+ny*d,zz])
    crest=[[],[]];flank=[]
    for i in range(len(fs)-1):
        for j in range(Ns):j2=(j+1)%Ns;(crest[fs[i][0]%2] if fs[i][1]==1/3 else flank).append((i*Ns+j,i*Ns+j2,(i+1)*Ns+j2,(i+1)*Ns+j))
    living_put((V,crest[0]),'fabric',col);living_put((V,crest[1]),'fabric',living_hex(col,.95));living_put((V,flank),'fabric',living_hex(col,.8))
    V=[];F=[];m=64
    for zz in (zb+.5,zt+.2):
        for js in range(m):px,py,nx,ny=living_rr_at(HX(zz)-.9,HY(zz)-.9,rc-.9,js/m);V.append([px,py,zz])
    for j in range(m):F.append((j,m+j,m+(j+1)%m,(j+1)%m))
    living_put((V,F),'fabric',living_hex(col,.8))
    rim=[];rr=[]
    for js in range(120):
        px,py,nx,ny=living_rr_at(HX(zt),HY(zt),rc,js/120);rim.append((px-nx*.45,py-ny*.45,zt+.55));rr.append(.93+.07*math.cos(math.pi*js/2))
    living_put(living_pipe(rim,rr,8,True),'fabric',living_hex(col,.93))
    foot=[]
    for js in range(72):px,py,nx,ny=living_rr_at(HX(zb),HY(zb),rc,js/72);foot.append((px-nx*.25,py-ny*.25,zb-.2))
    living_put(living_pipe(foot,.8,6,True),'fabric',living_hex(col,.88))
    # blanket: bunched fill below, a throw folded six times with rounded folds, its top end draped over the +x rim
    living_put(living_move(living_rbox((2*(HX(zt)-1.3),2*(HY(zt)-1.3),6.0),2.4,(4,4,1),2,{(2,1):.3}),(0,0,9.2)),'fabric',living_hex(bl,.95),True)
    L,W,t,p,n,z0,td=25.0,23.0,.85,1.05,6,13.0,.5;r=p/2;path=[]
    for i in range(n):
        d=1 if (n-1-i)%2==0 else -1;zz=z0+i*p;xs=-d*L/2;xe=L/2-2 if i==n-1 else d*L/2
        path+=[(xs+(xe-xs)*j/6,zz) for j in range(7)]
        if i<n-1:path+=[(d*(L/2+r*math.cos(a)),zz+r+r*math.sin(a)) for a in [math.pi*(-.5+j/8) for j in range(1,8)]]
    # the free end wraps over the +x fold beneath it, then over the rim roll and hangs outside
    zt5=z0+(n-1)*p;w=lambda a:(16.05+1.42*math.cos(a),12.15+1.42*math.sin(a));v=lambda a:(L/2+1.75*math.cos(a),z0+3.5*p+1.75*math.sin(a))
    tail=living_spline([(L/2-2,zt5),v(1.75),v(1.05),v(.52),(15.25,15.5),w(1.2),w(.6),(17.47,12.0),(17.47,9.0),(17.47,3.0)],24)[1:]
    th=[t]*len(path)+[t+(td-t)*min(1,(j+1)/11) for j in range(len(tail))]
    g,fl,_=living_sweep(living_fold_outline(path+[tuple(q) for q in tail],th),-W/2,W/2,.2,.2,2,2,1,mid=10)
    def soft(x,y,z):
        q=(z-z0)/p;i=max(0,min(n-1,round(q)))
        if x<L/2:z+=(.22+.06*i+(.16*math.sin(.3*x+.42*y+1) if i==n-1 else 0))*math.sin(math.pi*max(0,(x+L/2)/L))*math.cos(math.pi*max(-.5,min(.5,y/W)))**.6
        if x<-L/2+2.5:x+=.3*math.sin(2.2*math.pi*y/W+1.7*i)*min(1,(-x-L/2+2.5)/2.5)
        if x>16.9 and z<12:z=12-(12-z)*(1+.07*math.sin(.5*y+1));x-=.1*(.5+.5*math.sin(.9*y))*min(1,(12-z)/9)
        y+=.3*math.sin(1.3*q+.4)+.25*math.sin(.5*x+1.1*q)*(abs(y)/(W/2))**4
        return x,y+.3,z
    living_put(living_xf(g,soft),'fabric',bl,flat=fl)

def living_fold_outline(path,th):
    # closed (x,z) outline of a sheet of thickness th[i] along an open centre path, half-round free ends
    n=len(path);N=[];T=[]
    for i in range(n):
        a=path[max(0,i-1)];b=path[min(n-1,i+1)];dx,dz=b[0]-a[0],b[1]-a[1];l=math.hypot(dx,dz) or 1;T.append((dx/l,dz/l));N.append((-dz/l,dx/l))
    off=lambda i,cn,ct:(path[i][0]+(N[i][0]*cn+T[i][0]*ct)*th[i]/2,path[i][1]+(N[i][1]*cn+T[i][1]*ct)*th[i]/2)
    cap=lambda i,sg:[off(i,sg*math.cos(math.pi*j/4),(1 if i else -1)*math.sin(math.pi*j/4)) for j in range(1,4)]
    return [off(i,1,0) for i in range(n)]+cap(n-1,1)+[off(i,-1,0) for i in range(n-1,-1,-1)]+cap(0,-1)

GROUP_BUILDERS.update({'Sofa leg':living_sofa_leg,'Sofa upholstered base':living_sofa_base,'Sofa back':living_sofa_back,'Sofa arm':living_sofa_arm,
    'Seat cushion':living_seat_cushion,'Back cushion':living_back_cushion,'Terracotta throw pillow':living_pillow,'Cream throw pillow':living_pillow,
    'Woven living rug':living_rug,'Rug border':living_rug_border,'Coffee table leg':living_coffee_leg,'Coffee table top':living_coffee_top,
    'Book on coffee table':living_book,'Coffee mug':living_mug,'TV console leg':living_console_leg,'TV console':living_console,
    'Console drawer front':living_drawer_front,'Television frame':living_tv_frame,'Folded blanket basket':living_basket,'Magazine stack':living_magazines})

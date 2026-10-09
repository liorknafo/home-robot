# Architecture group: moulded trim, windows, blinds, thresholds, potted plants,
# framed prints, slippers and toy blocks. Envelopes match the original boxes.
import bmesh
from mathutils import Matrix

def architecture_solid(name,verts,faces,kind=None,color=None,sharp=40):
    # Closed authored solid: outward normals, smooth curves, crisp arrises.
    o=mesh(name,verts,faces,kind,color,True);bm=bmesh.new();bm.from_mesh(o.data)
    bmesh.ops.recalc_face_normals(bm,faces=bm.faces);bm.to_mesh(o.data);bm.free();o.data.update()
    o.data.set_sharp_from_angle(angle=math.radians(sharp));return o

def architecture_prism(poly,axis,lo,hi,kind=None,color=None,sharp=40,name='Moulded profile'):
    # Closed 2D section (other two axes, index order) extruded along axis.
    a,b=[j for j in range(3) if j!=axis];n=len(poly);v=[]
    for t in (lo,hi):
        for p in poly:q=[0,0,0];q[axis]=t;q[a]=p[0];q[b]=p[1];v.append(tuple(q))
    f=[tuple(range(n)),tuple(range(2*n-1,n-1,-1))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    return architecture_solid(name,v,f,kind,color,sharp)

def architecture_ring(A,B,profile,axis=2,kind=None,color=None,sharp=40,pieces=False,name='Mitred moulding'):
    # Sweep a closed (offset d, height h) profile round an A x B rectangle; corners are true mitres.
    a,b=[j for j in range(3) if j!=axis];n=len(profile);C=[(-1,-1),(1,-1),(1,1),(-1,1)]
    def P(c,d,h):q=[0,0,0];q[a]=c[0]*(A+d);q[b]=c[1]*(B+d);q[axis]=h;return tuple(q)
    if not pieces:
        v=[P(c,d,h) for d,h in profile for c in C]
        f=[(i*4+k,i*4+(k+1)%4,(i+1)%n*4+(k+1)%4,(i+1)%n*4+k) for i in range(n) for k in range(4)]
        return [architecture_solid(name,v,f,kind,color,sharp)]
    out=[];cap=[tuple(range(n)),tuple(range(2*n-1,n-1,-1))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    for k in range(4):out.append(architecture_solid(name,[P(C[k],d,h) for d,h in profile]+[P(C[(k+1)%4],d,h) for d,h in profile],cap,kind,color,sharp))
    return out

def architecture_lathe(profile,kind=None,color=None,seg=48,sharp=50,name='Turned profile'):
    # Closed (r,z) profile; r=0 points become single poles (no degenerate fans).
    v=[];rows=[];f=[]
    for r,zz in profile:
        if r<1e-6:rows.append([len(v)]*seg);v.append((0,0,zz))
        else:rows.append(list(range(len(v),len(v)+seg)));v+=[(r*math.cos(2*math.pi*i/seg),r*math.sin(2*math.pi*i/seg),zz) for i in range(seg)]
    for A,B in zip(rows,rows[1:]):
        for i in range(seg):
            j=(i+1)%seg;q=[A[i],A[j],B[j],B[i]];q=[p for k,p in enumerate(q) if p not in q[:k]]
            if len(q)>2:f.append(tuple(q))
    return architecture_solid(name,v,f,kind,color,sharp)

def architecture_arc(cx,cy,r,a0,a1,n=4):
    return [(cx+r*math.cos(math.radians(a0+(a1-a0)*i/n)),cy+r*math.sin(math.radians(a0+(a1-a0)*i/n))) for i in range(n+1)]

def architecture_mirror(half):
    # Right half (x>=0) of a section, listed bottom to top -> closed loop symmetric in x.
    return half+[(-px,py) for px,py in reversed(half)]

def architecture_shade(c,f):
    # Darken (f<1) or lighten (f>1) a hex colour.
    v=[int(c[i:i+2],16) for i in (1,3,5)];v=[min(255,int(q*f)) if f<=1 else int(q+(255-q)*(f-1)) for q in v]
    return '#%02x%02x%02x'%tuple(v)

def architecture_nearest(o,name):
    return min((p for p in scene['objects'] if p['name']==name),key=lambda p:(p['center'][0]-o['center'][0])**2+(p['center'][1]-o['center'][1])**2)

def architecture_back(o):
    # Thin axis and the side (+-1) that is buried in a wall (wall-hung decor).
    ax=min(range(3),key=lambda j:o['size'][j])
    for s in (-1,1):
        p=list(o['center']);p[ax]+=s*(o['size'][ax]/2+.8)  # just behind the back face
        for w in scene['objects']:
            if w['group']=='wall' and all(abs(p[j]-w['center'][j])<=w['size'][j]/2 for j in range(3)):return ax,s
    return ax,-1

# ---------------------------------------------------------------- skirting
def architecture_skirting(o,index,size):
    x,y,z=size;A,B=(x-1.2)/2,(y-1.2)/2;h0=-z/2
    # 6 mm proud board: flat face, shadow quirk, ogee top; wraps wall ends with mitred returns.
    og=[(.5-.38*(1-math.cos(math.pi*t/6))/2,h0+6.7+2.85*t/6) for t in range(7)]
    prof=[(-.3,h0),(.6,h0),(.6,h0+6.4),(.5,h0+6.5)]+og+[(.12,h0+9.72),(.09,h0+9.9),(.03,h0+10),(-.3,h0+10)]
    architecture_ring(A,B,prof,2,'paint',col,35,name='Ogee skirting board')

# ---------------------------------------------------------------- door trim
def architecture_casing_section(L):
    # Architrave face across its width s (0 = outer edge at the wall, L = reveal at the opening): (s, proud).
    return [(0,5),(0,6.0),(.03,6.14),(.1,6.26),(.2,6.34),(.32,6.4),(.75,6.4),(.82,6.36),(.86,6.3),(.9,6.25),(L-.12,6.25),(L-.05,6.22),(L,6.15),(L,5)]

def architecture_door_casing(o,index,size):
    x,y,z=size;W=0 if x>y else 1;T=1-W
    hd=architecture_nearest(o,'Door casing header');s=1 if hd['center'][T]>o['center'][T] else -1
    # Jamb lining (2 cm) with a narrow moulded architrave each side; reveal towards the opening.
    sec=architecture_casing_section(1.79);right=[(5,-1)]+[(p,-.94+q) for q,p in sec[:-1]]+[(5,.85),(5,.93),(4.97,.98),(4.92,1)]
    poly=[(p,s*t) if W==0 else (s*t,p) for p,t in architecture_mirror(right)]
    architecture_prism(poly,2,-z/2,z/2-.35,'paint',col,35,'Moulded door architrave')

def architecture_door_header(o,index,size):
    x,y,z=size;L=0 if x>y else 1
    # Head lining with butt-jointed head architrave, slightly prouder than the side casings.
    right=[(4.92,-1.5),(4.97,-1.48),(5,-1.43),(5,-1.35),(6.42,-1.35),(6.48,-1.31),(6.5,-1.25),(6.5,.08),(6.45,.14),(6.4,.24),(6.4,.36),(6.45,.46),(6.5,.52),(6.5,.95)]+architecture_arc(5.95,.95,.55,0,90,4)[1:]+[(5,1.5)]
    architecture_prism(architecture_mirror(right),L,-size[L]/2,size[L]/2,'paint',col,35,'Moulded head architrave')

# ---------------------------------------------------------------- windows
def architecture_room_side(o):
    return -1 if o['center'][1]>0 else 1

def architecture_window_upright(o,index,size):
    x,y,z=size;r=architecture_room_side(o);hx,hy=x/2,y/2
    # Ovolo-moulded room face, square sides, outside glazing rebate and chamfered exterior arrises.
    half=[(hx-.65,-hy),(hx-.45,-hy+.2),(hx-.45,-.6),(hx,-.6),(hx,hy-.8)]+architecture_arc(hx-.8,hy-.8,.8,0,90,4)[1:]
    poly=[(px,r*py) for px,py in architecture_mirror(half)]
    architecture_prism(poly,2,-z/2+.1,z/2-.1,'paint',col,35,'Rebated window mullion')

def architecture_window_frame(o,index,size):
    x,y,z=size;r=architecture_room_side(o);hy,hz=y/2,z/2;head=o['center'][2]>150
    # Sill rail: ovolo room edge, glazing rebate, outside weathering. Head rail mirrors it.
    sec=[(-hy,-hz),(hy,-hz),(hy,hz-.8)]+architecture_arc(hy-.8,hz-.8,.8,0,90,4)[1:]+[(-.6,hz),(-.6,hz-.7),(-hy+2,hz-.7)]
    sec+=[(-hy+.2,hz-1.5),(-hy,hz-1.62)] if not head else [(-hy,hz-.7)]
    poly=[(r*p,-q if head else q) for p,q in sec]
    architecture_prism(poly,0,-x/2,x/2,'paint',col,35,'Rebated window rail')

def architecture_window_sill(o,index,size):
    x,y,z=size;r=architecture_room_side(o);hy,hz=y/2,z/2
    # Bullnosed window board through the wall, weathered outside top and a drip groove underneath.
    sec=[(-hy+.4,-hz),(-hy+1.3,-hz),(-hy+1.3,-hz+.5),(-hy+1.85,-hz+.5),(-hy+1.85,-hz),(hy-.6,-hz)]
    sec+=architecture_arc(hy-.6,-hz+.6,.6,-90,0,3)[1:]+architecture_arc(hy-.6,hz-.6,.6,0,90,3)
    sec+=[(-hy/2,hz),(-hy+.45,hz-.62)]+architecture_arc(-hy+.45,hz-1.07,.45,90,180,3)[1:]+[(-hy,-hz+.25),(-hy+.15,-hz+.05)]
    poly=[(r*p,q) for p,q in sec]
    architecture_prism(poly,0,-x/2,x/2,'paint',col,35,'Window board with drip')

def architecture_roller_blind(o,index,size):
    x,y,z=size;r=architecture_room_side(o);hz=z/2;trim='#e7e4dc'
    ups=sorted(p['center'][0]-o['center'][0] for p in scene['objects'] if p['name']=='Window upright' and abs(p['center'][1]-o['center'][1])<8 and abs(p['center'][0]-o['center'][0])<x/2+4)
    bays=[(a+2,b-2) for a,b in zip(ups,ups[1:])] or [(-x/2,x/2)]
    def Y(sec):return [(r*p,q) for p,q in sec]
    cas=[(-.86,hz),(.82,hz),(.82,hz-2.6)]+architecture_arc(-.15,hz-2.6,.97,0,-90,4)[1:]+[(-.86,hz-3.57)]
    brk=[(-.88,hz),(.88,hz),(.88,hz-2.6)]+architecture_arc(-.15,hz-2.6,1.03,0,-90,4)[1:]+[(-.88,hz-3.63)]
    bz=-hz-.4;bar=[(-.42,bz+.28),(-.3,bz+.1),(-.12,bz),(.12,bz),(.3,bz+.1),(.42,bz+.28),(.42,bz+1.6),(.3,bz+1.8),(-.3,bz+1.8),(-.42,bz+1.6)]
    for a,b in bays:
        # Cassette per sash bay, end brackets, taut fabric and a weighted bottom bar with a grip.
        architecture_prism(Y(cas),0,a+.3,b-.3,'paint',trim,35,'Blind cassette')
        for e,d in [(a,1),(b,-1)]:
            architecture_prism(Y(brk),0,min(e-.25*d,e+.35*d),max(e-.25*d,e+.35*d),'paint',architecture_shade(trim,.93),35,'Cassette end bracket')
            if abs(e)>x/2-4:  # hidden fixing tongue into the jamb
                architecture_prism(Y([(-.8,hz-1.0),(-.4,hz-1.0),(-.4,hz-2.4),(-.8,hz-2.4)]),0,min(e,d*-(x/2)),max(e,d*-(x/2)),'paint',trim,35,'Bracket fixing')
        architecture_prism(Y([(-.36,hz-3.2),(-.24,hz-3.2),(-.24,bz+1.2),(-.36,bz+1.2)]),0,a+.45,b-.45,'fabric',col,30,'Roller blind fabric')
        architecture_prism(Y(bar),0,a+.4,b-.4,'paint',trim,35,'Blind bottom bar')
        m=(a+b)/2;architecture_prism(Y([(.3,bz+.35),(.75,bz+.5),(.8,bz+.75),(.8,bz+1.25),(.75,bz+1.45),(.3,bz+1.55)]),0,m-1.6,m+1.6,'plastic',architecture_shade(trim,.9),35,'Blind pull grip')

# ---------------------------------------------------------------- thresholds
def architecture_threshold(o,index,size):
    x,y,z=size;L=0 if x>y else 1;W=1-L;hw,hz=size[W]/2,z/2;stone='bathroom' in o['name']
    # Exact thickness kept; bevelled reducer edges (hardwood ramp, stone saddle round-over).
    half=[(hw,-hz),(hw,-hz+.12),(hw-.06,-hz+.17),(1.25,hz-.08),(1.1,hz-.02),(.95,hz)] if not stone else [(hw,-hz),(hw,-hz+.22),(hw-.2,hz-.18),(hw-.42,hz-.05),(hw-.65,hz)]
    architecture_prism(architecture_mirror(half),L,-size[L]/2,size[L]/2,'ceramic' if stone else 'wood',col,30,'Bevelled threshold')

# ---------------------------------------------------------------- plants
def architecture_pot(o,index,size):
    x,y,z=size;R=x/2;hz=z/2
    # Unglazed flower pot: rolled collar, tapered body, sitting in a matching drainage saucer.
    body=[(0,-hz+.75),(8.2,-hz+.75),(8.55,-hz+.9),(8.75,-hz+1.2),(11.45,hz-4.4),(11.6,hz-4.27),(12.2,hz-4.1),(12.42,hz-3.85),(R,hz-3.4),(R,hz-.6),(R-.08,hz-.2),(R-.3,hz),(11.2,hz),(10.95,hz-.2),(10.8,hz-1.0),(10.7,hz-1.8),(0,hz-1.8)]
    architecture_lathe(body,'paint',col,48,40,'Terracotta flower pot')
    sau=[(0,-hz),(10.3,-hz),(10.6,-hz+.1),(12.15,-hz+2.0),(12.38,-hz+2.25),(12.42,-hz+2.42),(12.3,-hz+2.5),(12.05,-hz+2.36),(10.75,-hz+.75),(10.6,-hz+.68),(0,-hz+.68)]
    architecture_lathe(sau,'paint',architecture_shade(col,.93),48,40,'Drainage saucer')

def architecture_soil(o,index,size):
    x,y,z=size;R=x/2-.05;rnd=random.Random(index);seg=40;rings=7
    # Potting mix: slight mound at the stem, lumpy relief, edge tucked under the pot rim.
    waves=[(rnd.uniform(1,4),rnd.randint(2,6),rnd.uniform(0,6.3),rnd.uniform(.03,.07)) for _ in range(5)]
    def h(rr,a):
        t=rr/R;return -.45+.7*(1-t*t)**1.5*.96+sum(amp*math.sin(k*a+ph+fr*rr)*min(1,4*(1-t)+.2) for fr,k,ph,amp in waves)
    v=[(0,0,h(0,0))];f=[]
    for i in range(1,rings+1):
        rr=R*(i/rings)**.8
        for j in range(seg):a=2*math.pi*j/seg;v.append((rr*math.cos(a),rr*math.sin(a),h(rr,a) if i<rings else -.45))
    for j in range(seg):f.append((0,1+j,1+(j+1)%seg))
    for i in range(rings-1):
        for j in range(seg):a=1+i*seg+j;b=1+i*seg+(j+1)%seg;f.append((a,a+seg,b+seg,b))
    base=len(v);v+=[(R*math.cos(2*math.pi*j/seg),R*math.sin(2*math.pi*j/seg),-.8) for j in range(seg)]
    top=1+(rings-1)*seg
    for j in range(seg):f.append((top+j,base+j,base+(j+1)%seg,top+(j+1)%seg))
    f.append(tuple(range(base+seg-1,base-1,-1)))
    # Base layer is matte (the soil image does not tile across the pot centre); crumbs carry the soil texture.
    architecture_solid('Potting soil surface',v,f,'paint',architecture_shade(col,.6),70)
    # Soil crumbs, bark chips and a few perlite grains half-sunk into the surface.
    for kind,c,count,sz,npt in [('soil',col,70,(.6,1.4),8),('soil',architecture_shade(col,.85),55,(.55,1.3),8),('soil',architecture_shade(col,1.15),45,(.45,1.1),8),('wood','#4e3a2b',14,(.5,1.1),9),('wood','#5e4734',10,(.45,1.0),9),('soil','#b5ad9f',10,(.22,.4),7)]:
        cv=[];cf=[]
        for _ in range(count):
            rr=R*.95*math.sqrt(rnd.random());a=rnd.uniform(0,2*math.pi)
            px,py=rr*math.cos(a),rr*math.sin(a);sc_=rnd.uniform(*sz);fl=.3 if kind=='wood' else .8 if npt==7 else .62
            M=Matrix.Rotation(rnd.uniform(0,6.3),3,'Z')@Matrix.Rotation(rnd.uniform(-.25,.25),3,'X')@Matrix.Rotation(rnd.uniform(-.25,.25),3,'Y')
            pts=[M@Vector((rnd.uniform(-1,1)*sc_/2,rnd.uniform(-.65,.65)*sc_/2,rnd.uniform(-1,1)*sc_*fl/2)) for _ in range(npt)]
            bm=bmesh.new();vs=[bm.verts.new(p) for p in pts];bmesh.ops.convex_hull(bm,input=vs)
            for q in [q for q in bm.verts if not q.link_faces]:bm.verts.remove(q)
            base=len(cv);idx={q:i for i,q in enumerate(bm.verts)};zz=h(math.hypot(px,py),math.atan2(py,px))+sc_*fl*.2
            cv+=[(px+q.co.x,py+q.co.y,zz+q.co.z) for q in bm.verts];cf+=[tuple(base+idx[q] for q in fc.verts) for fc in bm.faces];bm.free()
        architecture_solid({'wood':'Bark mulch chips','soil':'Soil crumbs' if npt==8 else 'Perlite grains'}[kind],cv,cf,kind,c,80 if npt==8 else 50)

def architecture_stem(o,index,size):
    x,y,z=size;hz=z/2
    nodes=[p['center'][2]-o['center'][2] for p in scene['objects'] if p['name']=='Plant leaf' and math.hypot(p['center'][0]-o['center'][0],p['center'][1]-o['center'][1])<15]
    # Woody tapered stem with gentle sway, swollen leaf nodes and a sheathed terminal bud.
    zs=[-hz+.3,-hz+10,-hz+20]+[-hz+22+k*1.5 for k in range(int((z-25)/1.5))]+[hz-2.6]
    ring=8;v=[];f=[]
    for k,zz in enumerate(zs):
        t=(zz+hz)/z;rr=.92-.32*t+sum(.16*math.exp(-((zz-nz)/.7)**2) for nz in nodes)
        cx=.12*math.sin(t*5.1+index);cy=.1*math.sin(t*3.7+1.3*index)
        v+=[(cx+rr*math.cos(2*math.pi*j/ring),cy+rr*math.sin(2*math.pi*j/ring),zz) for j in range(ring)]
    for k in range(len(zs)-1):
        for j in range(ring):a=k*ring+j;b=k*ring+(j+1)%ring;f.append((a,b,b+ring,a+ring))
    v.append((cx,cy,hz-1.6));top=len(v)-1;last=(len(zs)-1)*ring
    for j in range(ring):f.append((last+j,last+(j+1)%ring,top))
    f.append(tuple(range(ring-1,-1,-1)))
    architecture_solid('Woody plant stem',v,f,'wood',col,60)
    # Terminal leaf sheath (bud) on top of the stem.
    bud=architecture_lathe([(0,hz-3.2),(.62,hz-3.0),(.55,hz-1.8),(.32,hz-.7),(0,hz)],'leaf','#8a6b55',8,70,'Terminal leaf sheath')
    for p in bud.data.vertices:p.co.x+=cx;p.co.y+=cy

def architecture_blade(L,W,arch,droop,n,m,th,bend,ph,rib):
    # Ovate leathery blade, acuminate tip, gull-wing section, rippled margin. Upper and under
    # surfaces are separate open shells meeting at the margin (lens section, no rim faces).
    def P(t,s,side):
        hw=W/2*max(0.,math.sin(math.pi*t**.82))**.7*(1-.35*max(0.,(t-.68)/.32)**1.6)
        z=L*(arch*math.sin(math.pi*t)-droop*t*t)+hw*(.22*abs(s)-.16*s*s+.06*s*s*math.sin(9*t+ph*(1 if s>0 else 1.7)))
        return Vector((t*L,s*hw+bend*L*t*t,z+side*th/2*max(0.,1-s*s)**.35*(1-.55*t))),hw
    ts=[i/n for i in range(1,n)];ss=[-1+2*j/(m-1) for j in range(m)];B=len(ts)*m;g=lambda i,j:i*m+j
    up=[P(t,s,1)[0] for t in ts for s in ss]+[Vector((0,0,0)),P(1,0,0)[0]]
    un=[P(t,s,-1)[0] for t in ts for s in ss]+[Vector((0,0,0)),P(1,0,0)[0]]
    # Explicit triangles with the same diagonals on both shells, so the under shell never pokes through.
    f=[q for i in range(n-2) for j in range(m-1) for q in ((g(i,j),g(i+1,j),g(i+1,j+1)),(g(i,j),g(i+1,j+1),g(i,j+1)))]
    f+=[(B,g(0,j),g(0,j+1)) for j in range(m-1)]+[(g(n-2,j),B+1,g(n-2,j+1)) for j in range(m-1)]
    rv=[];rf=[]
    if rib:  # pale midrib: flush stripe on top, raised keel underneath (feet sunk into the blade)
        k=len(ts)-1
        for side,h0,h1,wf in [(1,.055,.02,1),(-1,-.1,-.04,1.5)]:
            o=len(rv)
            for i,t in enumerate(ts[:k]):
                hw=P(t,0,1)[1];w=min(.45*hw,(.05+.13*(1-t))*wf);e=Vector((0,0,-.012*side))
                h=Vector((0,0,(h0*(1-t)+h1*t) if i<k-1 else -.012*side))
                rv+=[P(t,-w/hw,side)[0]+e,P(t,0,side)[0]+h,P(t,w/hw,side)[0]+e]
            q=[(o+3*i+j,o+3*i+3+j,o+3*i+4+j,o+3*i+1+j) for i in range(k-1) for j in range(2)]
            rf+=q if side>0 else [tuple(reversed(a)) for a in q]
    return up,f,un,[tuple(reversed(a)) for a in f],rv,rf,Vector((1,0,arch*math.pi)).normalized()

def architecture_petiole(p0,p1,d1,r0,r1,k=6,rings=5):
    # Leaf stalk: quadratic curve from the stem node, arriving along the blade's base tangent (capped tube).
    p0,p1=Vector(p0),Vector(p1);c=p1-d1*(p1-p0).length*.45;v=[];f=[]
    pts=[(1-t)**2*p0+2*(1-t)*t*c+t*t*p1 for t in [i/(rings-1) for i in range(rings)]]
    for i,p in enumerate(pts):
        d=(pts[min(i+1,rings-1)]-pts[max(i-1,0)]).normalized();u=d.cross(Vector((0,0,1)));u=u.normalized() if u.length>1e-3 else Vector((1,0,0));w=d.cross(u)
        rr=r0+(r1-r0)*i/(rings-1);v+=[tuple(p+rr*(math.cos(2*math.pi*j/k)*u+math.sin(2*math.pi*j/k)*w)) for j in range(k)]
    f=[(i*k+j,i*k+(j+1)%k,(i+1)*k+(j+1)%k,(i+1)*k+j) for i in range(rings-1) for j in range(k)]
    return v,f+[tuple(range(k-1,-1,-1)),tuple(range((rings-1)*k,rings*k))]

def architecture_plant_leaf(o,index,size):
    x,y,z=size;rnd=random.Random(index*7+3)
    st=architecture_nearest(o,'Plant stem');yw=math.radians(o['yaw']);dx,dy=st['center'][0]-o['center'][0],st['center'][1]-o['center'][1]
    S=Vector((dx*math.cos(yw)+dy*math.sin(yw),-dx*math.sin(yw)+dy*math.cos(yw),0))
    top=o['center'][2]>st['center'][2]+st['size'][2]/2-2;N=S+Vector((0,0,-1.4 if top else -.2))
    J=lambda a,f=.07:a*(1+rnd.uniform(-f,f))
    # Node cluster: long leaf reaching out, two shorter side leaves, a young leaf unfolding behind the stem.
    # (yaw, blade pitch, roll, length, width, arch, droop, stalk length, stalk elevation, sideways bend, rib)
    spec=[(J(3,1),J(14),rnd.uniform(-8,8),J(19.0),J(7.6),.10,.36,3.0,30,rnd.uniform(-.03,.03),1),
          (J(-22),J(26),rnd.uniform(-16,-6),J(11.5),J(5.0),.11,.30,2.4,38,-.03,1),
          (J(21),J(-2),rnd.uniform(6,16),J(12.0),J(5.2),.08,.32,2.2,12,.03,1),
          (J(172,.02),J(60),rnd.uniform(-8,8),J(5.9),J(2.4),.05,.08,.9,62,0,0)]
    parts=[]
    for ya,pt,ro,L,W,ar,dr,pl,pe,bd,rib in spec:
        R=Matrix.Rotation(math.radians(ya),3,'Z');Rb=R@Matrix.Rotation(math.radians(-pt),3,'Y')@Matrix.Rotation(math.radians(ro),3,'X')
        b=N+R@Vector((pl*math.cos(math.radians(pe)),0,pl*math.sin(math.radians(pe))))
        up,f,un,fu,rv,rf,d=architecture_blade(L,W,ar,dr,10 if L>8 else 7,7 if L>8 else 5,.09 if L>8 else .06,bd,rnd.uniform(0,6.3),rib)
        parts.append(([b+Rb@p for p in up],f,[b+Rb@p for p in un],fu,[b+Rb@p for p in rv],rf,b,Rb@d,L))
    # Fit the cluster to the original leaf envelope with one affine scale per axis (no kinks).
    tlo=Vector((-.5510*x,-.5030*y,-.3045*z));thi=Vector((.5*x,.5030*y,.4*z))
    allv=[p for q in parts for p in q[0]+q[2]];lo=Vector([min(p[j] for p in allv) for j in range(3)]);hi=Vector([max(p[j] for p in allv) for j in range(3)])
    k=Vector([(thi[j]-tlo[j])/(hi[j]-lo[j]) for j in range(3)]);fit=lambda p:tlo+(p-lo)*k
    acc={key:([],[]) for key in ('up','un','rib','stalk')}
    def add(key,v,f):V,F=acc[key];F+=[tuple(i+len(V) for i in q) for q in f];V+=[tuple(p) for p in v]
    for up,f,un,fu,rv,rf,b,d,L in parts:
        add('up',[fit(p) for p in up],f);add('un',[fit(p) for p in un],fu)
        if rv:add('rib',[fit(p) for p in rv],rf)
        d=Vector([d[j]*k[j] for j in range(3)]).normalized();b=fit(b)
        add('stalk',*architecture_petiole(N,b+d*.3,d,.19 if L>8 else .12,.12 if L>8 else .07))
    for key,name,c in [('up','Leaf blades upper surface',col),('un','Leaf blades under surface',architecture_shade(col,1.16)),
                       ('rib','Leaf midribs',architecture_shade(col,1.26)),('stalk','Leaf petioles',architecture_shade(col,1.22))]:
        V,F=acc[key];p=mesh(name,V,F,'leaf',c,True);p.data.set_sharp_from_angle(angle=math.radians(70))

# ---------------------------------------------------------------- framed prints
def architecture_picture_frame(o,index,size):
    ax,back=architecture_back(o);a,b=[j for j in range(3) if j!=ax];A,B=size[a]/2,size[b]/2;f=-back
    # Mitred moulding: rounded back band, cove, inner bead and a rebate lip over the mount.
    prof=[(0,-1.5),(0,1.05),(.06,1.25),(.18,1.4),(.35,1.48),(.5,1.5),(.9,1.5),(1.0,1.45),(1.05,1.36),(1.2,1.18),(1.45,1.06),(1.75,1.02),(2.05,1.06),(2.3,1.16),(2.5,1.3),(2.65,1.39),(2.8,1.42),(2.95,1.39),(3.05,1.34),(3.15,1.33),(3.85,1.33),(3.9,1.3),(3.85,1.26),(3.35,1.26),(3.35,-1.5)]
    prof=[(-u,f*h) for u,h in prof]
    architecture_ring(A,B,prof,ax,'wood',col,40,True,'Mitred frame moulding')

def architecture_picture_print(o,index,size):
    ax,back=architecture_back(o);a,b=[j for j in range(3) if j!=ax];A,B=size[a]/2,size[b]/2;f=-back
    # Bevel-cut cream mount board over the print, recessed under the frame lip.
    prof=[(0,-1.3),(0,1.2),(-5.5,1.2),(-5.68,1.0),(-5.68,-1.3)]
    architecture_ring(A,B,[(d,f*h) for d,h in prof],ax,'paper','#e8e2d3',30,name='Bevel-cut mount')
    lo,hi=sorted((f*-1.3,f*1.02));poly=[(-(A-5.62),-(B-5.62)),(A-5.62,-(B-5.62)),(A-5.62,B-5.62),(-(A-5.62),B-5.62)]
    architecture_prism(poly,ax,lo,hi,'art','#ffffff',30,'Printed artwork')

# ---------------------------------------------------------------- slipper
def architecture_slipper(o,index,size):
    x,y,z=size;seg=64;b0=-z/2
    def sole_w(xx):
        e=max(0.,1-(xx/(x/2))**2)**.5;g=5.55 if xx>-3 else 5.55-(-3-xx)*1.25/(x/2-3);return max(.03,e**.8*g)
    outline=[(x/2*math.cos(2*math.pi*i/seg),math.copysign(sole_w(x/2*math.cos(2*math.pi*i/seg)),math.sin(2*math.pi*i/seg)) if i%(seg//2) else 0) for i in range(seg)]
    # Foot-shaped rubber outsole (narrower heel, broad ball) with a pale foxing band.
    v=[(a,c,h) for h in [b0,b0+.9] for a,c in outline];f=[tuple(range(seg-1,-1,-1)),tuple(range(seg,2*seg))]
    for i in range(seg):j=(i+1)%seg;f.append((i,j,j+seg,i+seg))
    sole=mesh('Shaped rubber outsole',v,f,'rubber','#554b43');bv=sole.modifiers.new('Soft sole edge','BEVEL');bv.width=.18;bv.segments=3
    tube([(a*.998,c*.998,b0+.58) for a,c in outline],.08,'rubber','#d2c2a9',True)
    # Dished footbed with a heel cup and raised rim.
    fv=[(0,0,b0+1.0)];ff=[];K=5;fs=48
    for k in range(1,K+1):
        s=k/K
        for i in range(fs):
            ang=2*math.pi*i/fs;px=x/2*math.cos(ang)*.93*s;py=math.copysign(sole_w(x/2*math.cos(ang)),math.sin(ang))*.9*s if i%(fs//2) else 0
            fv.append((px,py,b0+1.0+.32*s**5+.08*s**2*max(0,-px/(x/2))))
    for i in range(fs):ff.append((0,1+i,1+(i+1)%fs))
    for k in range(K):  # last band drops the insole edge into the sole
        if k==K-1:fv+=[(px,py,b0+.85) for px,py,_ in fv[1+k*fs:1+(k+1)*fs]]
        for i in range(fs):a=1+k*fs+i;c=1+k*fs+(i+1)%fs;ff.append((a,a+fs,c+fs,c))
    ff.append(tuple(range(len(fv)-1,len(fv)-fs-1,-1)))
    architecture_solid('Dished footbed',fv,ff,'fabric','#bfad98',45)
    # Open-backed upper: original vamp shape (unchanged where the robot camera sees it), toe closed over the sole.
    vv=[];ff=[];n,m=24,24;rows=[]
    for i in range(n+1):
        t=i/n;xx=-x*.10+t*(x*.60-.06);factor=math.sqrt(max(.0001,1-(xx/(x*.51))**2))
        w=min(y*.49*factor,sole_w(xx)-.1);height=(z-1.15)*(1-.15*t)*factor*(w/(y*.49*factor))*min(1,max(0,(x/2-xx)/1.4))**.5
        height=max(height,.12);rows.append((xx,w,height))
        for j in range(m+1):a=math.pi*j/m;vv.append((xx,math.cos(a)*w,b0+.85+math.sin(a)*height))
    for i in range(n):
        for j in range(m):a=i*(m+1)+j;ff.append((a,a+m+1,a+m+2,a+1))
    tipc=len(vv);vv.append((x/2-.02,0,b0+.85))
    for j in range(m):a=n*(m+1)+j;ff.append((a,tipc,a+1))
    upper=mesh('Open-backed fabric upper',vv,ff,'fabric',col);s=upper.modifiers.new('Fabric thickness','SOLIDIFY');s.thickness=.22
    tube(vv[:m+1],.17,'fabric','#cfbfa8')
    # Lock stitching along the throat binding and round the vamp base.
    sv=[];sf=[]
    def stitch(p,q,nrm):
        p,q,nrm=Vector(p),Vector(q),Vector(nrm).normalized();d=(q-p);side=d.cross(nrm).normalized()*.035;up=nrm*.03;k=len(sv)
        for e in (p,q):
            for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]:sv.append(tuple(e+side*sx+up*sy))
        sf.extend([(k,k+1,k+2,k+3),(k+7,k+6,k+5,k+4),(k,k+4,k+5,k+1),(k+1,k+5,k+6,k+2),(k+2,k+6,k+7,k+3),(k+3,k+7,k+4,k)])
    def on_upper(i_f,a,off=.25):
        i=min(n-1,int(i_f));fr=i_f-i;xx,w,hh=[rows[i][q]*(1-fr)+rows[i+1][q]*fr for q in range(3)]
        nrm=Vector((0,math.cos(a)/max(w,.1),math.sin(a)/max(hh,.1))).normalized();return Vector((xx,math.cos(a)*w,b0+.85+math.sin(a)*hh))+nrm*off,nrm
    for k in range(34):  # throat row
        a0=.12+(math.pi-.24)*k/34;a1=a0+(math.pi-.24)/34*.6;p,nr=on_upper(.9,a0);q,_=on_upper(.9,a1);stitch(p,q,nr)
    for a in (.16,math.pi-.16):  # vamp base rows, both sides
        for k in range(40):
            i0=.3+k*(n-3.2)/40;i1=i0+(n-3.2)/40*.6;p,nr=on_upper(i0,a);q,_=on_upper(i1,a);stitch(p,q,nr)
    architecture_solid('Stitching thread',sv,sf,'fabric','#d8cbb6',40)

# ---------------------------------------------------------------- toy blocks
def architecture_glyph(ch,size,z0,z1,M):
    # Built-in font glyph extruded between z0..z1, centred, transformed by 4x4 M; returns (verts, faces).
    cu=bpy.data.curves.new('Glyph','FONT');cu.body=ch;cu.size=size;cu.resolution_u=2;cu.extrude=(z1-z0)/2
    ob=bpy.data.objects.new('Glyph',cu);bpy.context.collection.objects.link(ob);dg=bpy.context.evaluated_depsgraph_get()
    me=bpy.data.meshes.new_from_object(ob.evaluated_get(dg));bpy.data.objects.remove(ob);bpy.data.curves.remove(cu)
    bm=bmesh.new();bm.from_mesh(me);bmesh.ops.remove_doubles(bm,verts=bm.verts,dist=1e-5);bm.to_mesh(me);bm.free()  # weld caps to sides
    c=[(min(p.co[i] for p in me.vertices)+max(p.co[i] for p in me.vertices))/2 for i in range(2)]
    v=[tuple(M@Vector((p.co.x-c[0],p.co.y-c[1],p.co.z+(z0+z1)/2))) for p in me.vertices];f=[tuple(p.vertices) for p in me.polygons]
    bpy.data.meshes.remove(me);return v,f

def architecture_toy_block(o,index,size):
    x,y,z=size;k=index%3;letters=['ABCDE','FGHKM','NPRST'][k];ink=['#8c4a3c','#3f5f73','#5f7344'][k]
    # Sanded wooden cube with well-rounded arrises; shallow engraved, ink-filled letters on five faces.
    blk=box((0,0,0),(x,y,z),0,'toy',col);bv=blk.modifiers.new('Sanded edges','BEVEL');bv.width=.5;bv.segments=4;bv.affect='EDGES'
    dg=bpy.context.evaluated_depsgraph_get();me=bpy.data.meshes.new_from_object(blk.evaluated_get(dg));blk.modifiers.clear();blk.data=me
    faces=[((0,0,1),(0,1,0)),((1,0,0),(0,0,1)),((-1,0,0),(0,0,1)),((0,1,0),(0,0,1)),((0,-1,0),(0,0,1))]
    cv=[];cf=[];fv=[];ff=[];depth=.12
    for ch,(nrm,up) in zip(letters,faces):
        nrm,up=Vector(nrm),Vector(up);right=up.cross(nrm);h=abs(nrm.dot(Vector((x/2,y/2,z/2))))
        M=Matrix((right,up,nrm)).transposed().to_4x4();M.translation=nrm*h
        v,f=architecture_glyph(ch,3.5,-depth,.4,M);cf+=[tuple(i+len(cv) for i in q) for q in f];cv+=v
        v,f=architecture_glyph(ch,3.5,-depth,-depth+.05,M);ff+=[tuple(i+len(fv) for i in q) for q in f];fv+=v
    cme=bpy.data.meshes.new('Engraving cutter');cme.from_pydata(cv,[],cf);cut=bpy.data.objects.new('Engraving cutter',cme);bpy.context.collection.objects.link(cut)
    bo=blk.modifiers.new('Engraved letters','BOOLEAN');bo.operation='DIFFERENCE';bo.object=cut;bo.solver='EXACT'
    dg=bpy.context.evaluated_depsgraph_get();me=bpy.data.meshes.new_from_object(blk.evaluated_get(dg));blk.modifiers.clear();blk.data=me
    bpy.data.objects.remove(cut);bpy.data.meshes.remove(cme)
    for p in blk.data.polygons:p.use_smooth=True
    blk.data.set_sharp_from_angle(angle=math.radians(35))
    architecture_solid('Engraved ink fill',fv,ff,'paint',ink,30)

GROUP_BUILDERS.update({
    'Plaster partition skirting':architecture_skirting,'West exterior skirting':architecture_skirting,'East exterior skirting':architecture_skirting,
    'Door casing':architecture_door_casing,'Door casing header':architecture_door_header,
    'Window upright':architecture_window_upright,'Window frame':architecture_window_frame,'Deep window sill':architecture_window_sill,
    'Pale roller blind':architecture_roller_blind,
    '10 mm hardwood threshold':architecture_threshold,'8 mm bathroom threshold':architecture_threshold,
    'Terracotta pot':architecture_pot,'Soil':architecture_soil,'Plant stem':architecture_stem,'Plant leaf':architecture_plant_leaf,
    'Picture frame':architecture_picture_frame,'Picture print':architecture_picture_print,
    'Slipper':architecture_slipper,'Toy block':architecture_toy_block})

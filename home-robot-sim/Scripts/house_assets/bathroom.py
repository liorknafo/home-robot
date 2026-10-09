# Bathroom group: shaker vanity, stone top with inset basin + low mixer, rebated mirror frame,
# close-coupled toilet (pedestal flowing into the pan, seat + lid, cistern), cast shower tray, square-tube
# shower frame with rings over curtain eyelets, gathered curtain with rounded irregular folds, pile bath mat.
# Envelopes match the baseline. (The fixed layout leaves the curtain top 3 cm below the rail rings.)
import bmesh

def bathroom_hex(c,f):
    # Derived shade: f<1 darkens, f>1 blends towards white.
    v=[int(c[i:i+2],16) for i in (1,3,5)]
    v=[round(a*f) if f<=1 else round(a+(255-a)*(f-1)) for a in v]
    return '#%02x%02x%02x'%tuple(max(0,min(255,a)) for a in v)

def bathroom_se(n):
    # Unit superellipse outline (counter-clockwise).
    return lambda t:(math.copysign(abs(math.cos(t))**(2/n),math.cos(t)),math.copysign(abs(math.sin(t))**(2/n),math.sin(t)))

def bathroom_egg(e=.12):
    # Toilet-pan plan: wider towards +y (the cistern), normalised to unit half-width.
    m=max(math.cos(t)*(1+e*math.sin(t)) for t in [i*math.pi/1000 for i in range(-500,501)])
    return lambda t:(math.cos(t)*(1+e*math.sin(t))/m,math.sin(t))

def bathroom_ring(fn,n,ax,by,cx,cy,z):
    return [(cx+ax*u,cy+by*v,z) for u,v in (fn(2*math.pi*i/n) for i in range(n))]

def bathroom_rrect(hx,hy,r,z,cx=0,cy=0,m=4,kx=6,ky=6):
    # Rounded-rectangle ring with a fixed vertex layout so rings of any size loft cleanly.
    r=max(.03,min(r,hx-.01,hy-.01));p=[]
    sides=[((hx,-hy+r),(hx,hy-r),ky),((hx-r,hy),(-hx+r,hy),kx),((-hx,hy-r),(-hx,-hy+r),ky),((-hx+r,-hy),(hx-r,-hy),kx)]
    corners=[(hx-r,hy-r,0),(-hx+r,hy-r,90),(-hx+r,-hy+r,180),(hx-r,-hy+r,270)]
    for (a,b,k),(ox,oy,a0) in zip(sides,corners):
        p+=[(a[0]+(b[0]-a[0])*i/k,a[1]+(b[1]-a[1])*i/k) for i in range(k)]
        p+=[(ox+r*math.cos(math.radians(a0+90*i/m)),oy+r*math.sin(math.radians(a0+90*i/m))) for i in range(m)]
    return [(cx+px,cy+py,z) for px,py in p]

def bathroom_loft(rings,cap0=False,cap1=False,kind=None,color=None,cyclic=False,sharp=40,name='Lofted ceramic surface'):
    # Rings follow the section counter-clockwise in (radius,z): outward normals; cap0 faces down, cap1 up.
    n=len(rings[0]);L=len(rings);v=[p for r in rings for p in r];f=[]
    for k in range(L if cyclic else L-1):
        k2=(k+1)%L;f+=[(k*n+i,k*n+(i+1)%n,k2*n+(i+1)%n,k2*n+i) for i in range(n)]
    if cap0:c=len(v);v.append(tuple(sum(p[q] for p in rings[0])/n for q in range(3)));f+=[(c,(i+1)%n,i) for i in range(n)]
    if cap1:c=len(v);b=(L-1)*n;v.append(tuple(sum(p[q] for p in rings[-1])/n for q in range(3)));f+=[(c,b+i,b+(i+1)%n) for i in range(n)]
    o=mesh(name,v,f,kind,color,True)
    if sharp:o.data.set_sharp_from_angle(angle=math.radians(sharp))
    return o

def bathroom_round(pts,r,seg=4):
    # Replace polyline corners by short quadratic bends.
    out=[pts[0]]
    for a,b,c in zip(pts,pts[1:],pts[2:]):
        A,B,C=Vector(a),Vector(b),Vector(c);rr=min(r,(B-A).length/2,(C-B).length/2)
        p0=B+(A-B).normalized()*rr;p2=B+(C-B).normalized()*rr
        out+=[tuple((1-t)**2*p0+2*(1-t)*t*B+t*t*p2) for t in [i/seg for i in range(seg+1)]]
    return out+[pts[-1]]

def bathroom_tube(points,radius,kind=None,color=None,res=2):
    # Capped tube (handles, spout, lever, rail); ends closed and creased.
    cu=bpy.data.curves.new('Capped tube','CURVE');cu.dimensions='3D';cu.bevel_depth=radius;cu.bevel_resolution=res;cu.use_fill_caps=True
    s=cu.splines.new('POLY');s.points.add(len(points)-1)
    for p,q in zip(s.points,points):p.co=(*q,1)
    o=bpy.data.objects.new('Capped tube',cu);bpy.context.collection.objects.link(o)
    bpy.context.view_layer.objects.active=o;o.select_set(True);bpy.ops.object.convert(target='MESH');o=bpy.context.object;o.select_set(False)
    o.data.set_sharp_from_angle(angle=math.radians(50));return keep(o,kind,color,True)

def bathroom_fix_normals(o):
    bm=bmesh.new();bm.from_mesh(o.data);bmesh.ops.recalc_face_normals(bm,faces=bm.faces);bm.to_mesh(o.data);bm.free()

def bathroom_scene_color(name):
    return next(s['color'] for s in scene['objects'] if s['name']==name)

def bathroom_vanity(o,index,size):
    x,y,z=size;X,Y,h=x/2,y/2,z/2
    box((3.15,0,-h+4.85),(48.7,y-3,9.7),.2,'paint',bathroom_hex(col,.62))   # toe-kick plinth, 5 cm recess
    box((2.2,0,4.725),(52.6,y,68.45),.35)                                       # carcass behind overlay doors
    xf,xb=-26.2,-24.1;xc=(xf+xb)/2;t=xb-xf;z0,z1=-29.2,38.55;H=z1-z0;zc=(z0+z1)/2;st=6.5
    for s in [-1,1]:
        y0,y1=sorted((s*.15,s*(Y-.3)));W=y1-y0;yc=(y0+y1)/2
        for yy in [y0+st/2,y1-st/2]:box((xc,yy,zc),(t,st,H),.25)               # shaker stiles
        for zz in [z0+st/2,z1-st/2]:box((xc,yc,zz),(t,W-2*st+.2,st),.25)       # rails
        box((xc+.75,yc,zc),(.8,W-2*st+.4,H-2*st+.4),.2)                         # recessed centre panel
        yh=s*(.15+st/2)                                                           # bar pull on the meeting stile
        bathroom_tube(bathroom_round([(xf+.5,yh,19.5),(-28.3,yh,19.5),(-28.3,yh,34.5),(xf+.5,yh,34.5)],1.0),.55,'metal','#aeb1ac')

def bathroom_countertop(o,index,size):
    x,y,z=size;X,Y,h=x/2,y/2,z/2;cx=-3.5;ha,hb=13.8,23.8;se=bathroom_se(2.4)
    def rr(i,zz):return bathroom_rrect(X-i,Y-i,.8-i*.6,zz,m=4,kx=8,ky=12)
    def hole(pts,e,zz):
        out=[]
        for px,py,_ in pts:u,w=se(math.atan2(py/(hb+e),(px-cx)/(ha+e)));out.append((cx+(ha+e)*u,(hb+e)*w,zz))
        return out
    top=rr(.35,h);mid=[((a[0]+b[0])/2,(a[1]+b[1])/2,h) for a,b in zip(top,hole(top,.3,h))]
    # Stone slab with eased edges and a cut-out under the basin rim (closed section, cyclic loft).
    bathroom_loft([hole(top,0,-h),rr(.35,-h),rr(.1,-h+.1),rr(0,-h+.35),rr(0,h-.35),rr(.1,h-.1),top,mid,hole(top,.3,h),hole(top,.08,h-.08),hole(top,0,h-.3)],kind='ceramic',cyclic=True,name='Stone vanity top')
    # Lower half of the basin bowl continues inside the slab cut-out down to the waste.
    bz=[(11.2,21.2,1.5),(10.2,19.6,.6),(8.9,17.2,-.4),(7.0,13.4,-1.4),(4.8,8.6,-2.05),(3.0,4.6,-2.3),(2.0,2.0,-2.38),(2.0,2.0,-2.75)]
    bathroom_loft([bathroom_ring(se,64,a,b,cx,0,zz) for a,b,zz in bz],cap1=True,kind='ceramic',color=bathroom_scene_color('Vanity basin'),name='Basin bowl')
    dr=[(1.7,-2.78),(1.75,-2.5),(1.8,-2.42),(1.6,-2.33),(1.0,-2.3)]
    bathroom_loft([bathroom_ring(bathroom_se(2),24,a,a,cx,0,zz) for a,zz in dr],True,True,'metal','#a9ada9',name='Waste cap')

def bathroom_basin(o,index,size):
    se=bathroom_se(2.4);cx=-3.5;A,B=16,26;mc='#b9bdb9';tx=17.1
    # Inset rim resting on the stone, rolling over into the bowl (meets the top's lower bowl at z -2.5).
    prof=[(2.4,-.95),(0,-.95),(0,-.5),(.2,-.15),(.7,.05),(1.6,.12),(2.5,.02),(2.95,-.2),(3.15,-.55),(3.4,-1.3),(4.0,-2.0),(4.8,-2.5)]
    bathroom_loft([bathroom_ring(se,64,A-w,B-w,cx,0,zz) for w,zz in prof],kind='ceramic',name='Inset basin rim')
    # Low deck mixer behind the bowl: flanged body, short spout over the rim, side lever.
    body=[(2.55,-.98),(2.7,-.9),(2.7,-.78),(2.45,-.7),(2.15,-.68),(2.15,2.35),(2.0,2.58),(1.6,2.66)]
    bathroom_loft([bathroom_ring(bathroom_se(2),32,r,r,tx,0,zz) for r,zz in body],True,True,'metal',mc,name='Mixer body')
    bathroom_tube(bathroom_round([(16.2,0,1.95),(10.2,0,1.95),(8.8,0,1.6),(8.8,0,1.25)],.9),.55,'metal',mc)
    bathroom_tube([(tx,1.9,1.15),(tx,3.0,1.15)],.62,'metal',mc)
    bathroom_tube(bathroom_round([(tx,3.05,1.15),(15.0,3.05,1.45),(13.9,3.05,1.55)],.5),.26,'metal',mc)

def bathroom_frame(o,index,size):
    x,y,z=size;Y,Z=y/2,z/2;f0=-x/2-.45   # face 0.45 cm proud so the mirror sits in a rebate
    prof=[(0,3.45),(0,1.0),(.12,.5),(.4,.17),(.8,.03),(1.6,0),(2.6,.07),(3.4,.1),(3.7,.15),(3.8,.25),(3.8,3.45)]
    sides=[lambda w,s:(-Y+w+(2*Y-2*w)*s,Z-w),lambda w,s:(-Y+w+(2*Y-2*w)*s,-Z+w),lambda w,s:(Y-w,-Z+w+(2*Z-2*w)*s),lambda w,s:(-Y+w,-Z+w+(2*Z-2*w)*s)]
    n=len(prof)
    for P in sides:
        v=[(f0+d,*P(w,s)) for w,d in prof for s in (0,1)]
        f=[(2*i,2*((i+1)%n),2*((i+1)%n)+1,2*i+1) for i in range(n)]+[tuple(2*i for i in range(n)),tuple(2*i+1 for i in range(n-1,-1,-1))]
        m=mesh('Mitred frame moulding',v,f);bathroom_fix_normals(m);m.data.set_sharp_from_angle(angle=math.radians(35))

def bathroom_mirror(o,index,size):
    box((.4,0,0),size,.05,'mirror')   # set back 0.4 cm into the frame rebate

def bathroom_toilet_base(o,index,size):
    x,y,z=size;h=z/2;se=bathroom_se(2);egg=bathroom_egg(.12)
    # Waisted trap foot (inside the old robot-band outline, z 3.5-27), then flaring into the pan's own
    # egg-shaped plan so it meets the pan underside in a level seam; the top is hidden inside the pan.
    prof=[(10.6,15.3,.2,-h,0),(11.1,15.85,.2,-h+.4,0),(11.2,15.95,.2,-h+3,0),(10.8,15.5,.1,-h+6.5,0),(10.4,15.0,0,-h+11,0),(10.6,15.3,0,-h+18,0),
          (11.0,15.8,0,-h+24,0),(11.3,16.05,0,-h+27,0),(11.9,16.9,0,9.5,.4),(12.8,18.1,0,12,.8),(13.7,19.35,0,14.2,1),(14.75,20.8,0,16.1,1),(15.5,21.9,-.3,17.2,1),(16.2,23.1,-.1,h,1)]
    def mix(a,b,c,zz,w):return [tuple(p*(1-w)+q*w for p,q in zip(P,Q)) for P,Q in zip(bathroom_ring(se,64,a,b,0,c,zz),bathroom_ring(egg,64,a,b,0,c,zz))]
    bathroom_loft([mix(*p) for p in prof],True,True,'ceramic')
    for s in [-1,1]:   # floor bolt caps
        bathroom_loft([bathroom_ring(se,16,r,r,s*11.3,4,zz) for r,zz in [(1.25,-h),(1.2,-h+.45),(.9,-h+1.05),(.45,-h+1.35)]],True,True,'plastic',bathroom_hex(col,1.3),name='Bolt cap')

def bathroom_toilet_bowl(o,index,size):
    x,y,z=size;egg=bathroom_egg(.12);n=64;sc=bathroom_hex(col,1.35)
    # Pan: outer shell from inside the pedestal up to the rim, then the glazed bowl down to the water.
    shell=[(14.6,20.6,4.0,-6.0),(16.9,23.9,3.2,-3.5),(18.7,26.4,1.2,-1.0),(19.4,27.3,.2,1.2),(19.5,27.5,0,1.9),(19.35,27.35,0,2.3),(19.0,27.0,0,2.5),(17.2,25.0,-.3,2.55),(16.1,23.7,-.6,2.45),(15.7,23.1,-.8,2.15),(14.2,20.6,-1.2,.6),(11.2,16.0,-1.8,-1.4),(8.2,11.2,-2.2,-2.7),(6.6,8.6,-2.4,-3.0)]
    bathroom_loft([bathroom_ring(egg,n,a,b,0,c,zz) for a,b,c,zz in shell],True,True,'ceramic',name='Toilet pan')
    def clip(p,k=.9):return (p[0],-k*math.log(math.exp(-p[1]/k)+math.exp(-17/k)),p[2])   # straight hinge edge
    O=[clip(p) for p in bathroom_ring(egg,n,18.9,26.9,0,0,0)];I=bathroom_ring(egg,n,12.0,16.0,0,-3.5,0)
    sect=[(1.0,.05),(.85,0),(.12,0),(0,.25),(0,1.05),(.06,1.55),(.25,1.82),(.55,1.85),(.85,1.6),(.97,1.15),(1.0,.6)]
    bathroom_loft([[(a[0]*(1-u)+b[0]*u,a[1]*(1-u)+b[1]*u,2.6+zz) for a,b in zip(O,I)] for u,zz in sect],kind='plastic',color=sc,cyclic=True,name='Toilet seat')
    C=[sum(p[q] for p in O)/n for q in range(2)]
    lid=[(.9,0),(.985,.08),(1,.4),(1,.9),(.99,1.35),(.96,1.62),(.85,1.8),(.55,1.86)]
    bathroom_loft([[(C[0]+(p[0]-C[0])*s,C[1]+(p[1]-C[1])*s,4.5+zz) for p in O] for s,zz in lid],True,True,'plastic',sc,name='Toilet lid')
    for s in [-1,1]:box((s*7,17.9,3.975),(3.2,3.0,2.85),.6,'plastic',sc)   # hinge housings

def bathroom_toilet_tank(o,index,size):
    x,y,z=size;X,Y,h=x/2,y/2,z/2;se=bathroom_se(2)
    def rr(d,r,zz):return bathroom_rrect(X-d,Y-d,r,zz,m=5,kx=6,ky=3)
    bathroom_loft([rr(1.5,1.2,-h),rr(.45,2.1,-h+.35),rr(.2,2.4,-h+1.2),rr(.2,2.4,h-3.3),rr(.4,2.2,h-2.3),rr(.9,1.7,h-2.0)],True,True,'ceramic',name='Cistern body')
    bathroom_loft([rr(.35,2.45,h-1.95),rr(.05,2.75,h-1.7),rr(0,2.8,h-1.2),rr(.12,2.68,h-.35),rr(.6,2.2,h-.05),rr(1.5,1.4,h)],True,True,'ceramic',name='Cistern lid')
    def disc(prof,kind,c,cap0=True):bathroom_loft([bathroom_ring(se,32,r,r,0,0,h+zz) for r,zz in prof],cap0,True,kind,c,name='Flush button')
    disc([(2.4,-.15),(2.5,.05),(2.47,.2),(2.32,.25),(2.24,.14)],'metal','#b5b9b5')     # bezel
    disc([(2.05,.1),(2.05,.27),(1.95,.32),(1.05,.32),(1.05,.25)],'metal','#c2c5c1')    # large flush ring
    disc([(.9,.1),(.9,.27),(.82,.31)],'metal','#c2c5c1')                                # small flush

def bathroom_tray(o,index,size):
    x,y,z=size;X,Y,h=x/2,y/2,z/2
    # Front face pulled back 0.45 cm so the frame posts only touch the tray's foot, not its rim.
    def rr(w,zz,r):return bathroom_rrect(X-w,Y-w-.225,r,zz,cy=.225,m=6,kx=16,ky=8)
    # Vertical sides, rolled outer edge, flat rim, sloped inner wall, floor falling 0.65 cm to the waste.
    prof=[(.3,-h,1.2),(0,-h+.3,1.5),(0,1.6,1.5),(.15,2.35,1.6),(.55,2.8,1.8),(1.1,h,2.0),(3.6,h,3.0),(4.3,2.85,3.5),(4.7,2.45,3.9),(5.4,1.4,4.6),(6.0,1.0,5.2),(6.8,.9,6.0)]
    rings=[rr(*p) for p in prof];F=rings[-1]
    def circ(r,zz):return [(r*math.cos(math.atan2(py,px)),r*math.sin(math.atan2(py,px)),zz) for px,py,_ in F]
    C=circ(5.2,.25)
    rings+=[[(a[0]*(1-s)+b[0]*s,a[1]*(1-s)+b[1]*s,.9-.65*s) for a,b in zip(F,C)] for s in [.3,.55,.75,.9]]
    rings+=[C,circ(5.0,.12),circ(4.95,-.7)]
    bathroom_loft(rings,True,True,'ceramic',name='Low cast shower tray')
    dc=[(4.5,-.75),(4.62,.02),(4.55,.13),(4.2,.19),(2.0,.22)]   # waste cover with a shadow gap
    bathroom_loft([bathroom_ring(bathroom_se(2),48,r,r,0,0,zz) for r,zz in dc],True,True,'metal','#9a9f9b',name='Waste cover')

def bathroom_upright(o,index,size):
    x,y,z=size;h=z/2
    box((0,-.43,-h+.25),(2.8,1.95,.5),.15)                  # floor plate, stopping at the tray foot
    box((0,0,-1.45),(x,y,z-3.5),.3)                         # square tube
    box((0,0,h-1.7),(2.8,2.8,3.4),.45)                      # corner connector taking the rail

def bathroom_curtain_layout():
    # Shared fold layout (curtain-local cm): K half-waves between K+1 fold turns; even turns face the rail.
    R=random.Random(370);K=9
    xt=[-3.75+8.9*k/K for k in range(K+1)]                                            # tight pleats under the rings
    xb=[-5.36+10.68*k/K+(R.uniform(-.32,.32) if 0<k<K else 0) for k in range(K+1)]   # relaxed spread at the hem
    amp=[.42 if k in (0,K) else R.uniform(.58,1.0) for k in range(K+1)]
    ph=[R.uniform(0,6.3) for k in range(K+1)];fr=[R.uniform(3.5,8) for k in range(K+1)];cc=[R.uniform(.72,.95) for k in range(K)]
    return K,xt,xb,amp,ph,fr,cc

def bathroom_curtain_plan(L,q,M):
    # Plan-view cloth path at height fraction q (0 hem, 1 top): rounded fold turns joined by steep walls.
    K,xt,xb,amp,ph,fr,cc=L;loose=(1-q)**1.15;flare=max(0,1-q/.07);X=[];Y=[]
    ymid=.06+.09*(loose-.5)
    for k in range(K+1):
        d=.26*math.sin(q*fr[k]+ph[k])*loose**.5 if 0<k<K else 0
        X.append((xt[k]+(xb[k]-xt[k])*loose+d)*(1+.03*flare))
        a=amp[k]+((.95 if 0<k<K else .8)-amp[k])*q**3+(.07*math.sin(q*fr[k]*1.3+ph[k]) if 0<k<K else 0)
        Y.append(ymid+(-1 if k%2==0 else 1)*1.74*a*(1-.1*flare))
    for k in range(1,K+1):X[k]=max(X[k],X[k-1]+.55)
    pts=[]
    for k in range(K):
        dx=X[k+1]-X[k];c=cc[k];P=[(X[k],Y[k]),(X[k]+c*dx,Y[k]),(X[k+1]-c*dx,Y[k+1]),(X[k+1],Y[k+1])]
        for m in range(M):
            t=m/M;w=[(1-t)**3,3*(1-t)**2*t,3*(1-t)*t*t,t**3];pts.append(tuple(sum(w[i]*P[i][j] for i in range(4)) for j in range(2)))
    return pts+[(X[K],Y[K])]

def bathroom_rail(o,index,size):
    x,y,z=size;L=x/2
    bathroom_tube([(-L,0,0),(L,0,0)],.8)
    cur=next(s for s in scene['objects'] if s['name']=='Shower folded curtain');R=random.Random(372)
    for ex,ey,ez,du,at in bathroom_curtain_eyelets():   # one ring above each curtain eyelet, slightly swung
        xx=cur['center'][0]+ex-o['center'][0];yaw=math.radians(R.uniform(-9,9))
        tube([(xx-1.05*math.cos(t)*math.sin(yaw),1.05*math.cos(t)*math.cos(yaw),-.12+1.13*math.sin(t)) for t in [2*math.pi*i/14 for i in range(14)]],.19,'metal',bathroom_hex(col,1.15),True)

def bathroom_curtain_eyelets():
    # Eyelet centres on the header at the rail-side fold turns: (x, y, z, du, path sampler).
    lay=bathroom_curtain_layout();K=lay[0];z0,H=-83.35,166.8;zc=83.45-1.35;fine=64
    P=bathroom_curtain_plan(lay,(zc-z0)/H,fine)
    def at(u):
        s_=min(len(P)-1.001,max(0,u*fine));i=int(s_);t=s_-i;return (P[i][0]*(1-t)+P[i+1][0]*t,P[i][1]*(1-t)+P[i+1][1]*t)
    def span(u):return .36/(math.dist(at(u-.02),at(u+.02))/.04)   # parameter step for a 0.36 cm radius
    out=[]
    for k in range(0,K,2):
        uc=span(.3)+.06 if k==0 else k   # the edge eyelet sits just inside the side hem
        du=span(uc);out.append((*at(uc),zc,du,lambda a,uc=uc,du=du:at(uc+du*math.cos(a))))
    return out

def bathroom_curtain(o,index,size):
    x,y,z=size;lay=bathroom_curtain_layout();K=lay[0];M=8;N=K*M;R=random.Random(371)
    z0,z1=-83.35,83.45;H=z1-z0;hem,head=1.9/H,2.9/H
    qs=[0,.005,hem-.0012,hem,.022,.04,.065,.1,.15,.21,.28,.36,.44,.52,.6,.67,.73,.79,.84,.88,.915,.945,.965,1-head-.0012,1-head,.989,.995,1]
    sw=[R.uniform(0,6.3) for i in range(4)];v=[];f=[];wt=[]
    for q in qs:
        P=bathroom_curtain_plan(lay,q,M);zz=z0+H*q
        for i,(px,py) in enumerate(P):
            u=i/M
            sag=.42*math.sin(math.pi*u/2)**2*min(1,max(0,(q-.97)/.03))**2   # top edge droops between eyelets
            wob=.22*(.5+.5*math.sin(u*2.3+sw[0])*math.sin(u*.9+sw[1])) if q==0 else 0   # uneven hem line
            ripple=.05*math.sin(zz*.21+u*1.7+sw[2])*(1-q)   # soft lengthwise undulation
            v.append((px,py+ripple,zz-sag+wob));wt.append(1.0 if q<=hem+1e-6 or q>=1-head-1e-6 else .5)
    for j in range(len(qs)-1):
        f+=[(j*(N+1)+i,j*(N+1)+i+1,(j+1)*(N+1)+i+1,(j+1)*(N+1)+i) for i in range(N)]
    c=mesh('Gathered shower curtain',v,f,'fabric');g=c.vertex_groups.new(name='Hem and header')
    for i,w in enumerate(wt):g.add([i],w,'REPLACE')
    s=c.modifiers.new('Cloth thickness','SOLIDIFY');s.thickness=.2;s.offset=0;s.vertex_group=g.name;s.thickness_vertex_group=0
    # Metal eyelets punched through the header at the rail-side fold turns (they follow the cloth curvature).
    for ex,ey,ez,du,at in bathroom_curtain_eyelets():
        tube([(*at(a),ez+.36*math.sin(a)) for a in [2*math.pi*i/12 for i in range(12)]],.15,'metal','#b4b8b3',True)

def bathroom_mat(o,index,size):
    x,y,z=size;X,Y,h=x/2,y/2+.7,z/2;rc=3.0;R=random.Random(371)
    def axis(L):
        e=[0,.25,.6,1.0,1.45,1.9,2.4];n=max(1,round((2*L-4.8)/1.3))
        return [-L+a for a in e]+[-L+2.4+(2*L-4.8)*i/n for i in range(1,n)]+[L-a for a in reversed(e)]
    xs,ys=axis(X),axis(Y);nx,ny=len(xs),len(ys)
    def rho(ax_,ay_):   # ray scale from the rectangle to the rounded rectangle
        t=min(X/ax_ if ax_>1e-9 else 1e9,Y/ay_ if ay_>1e-9 else 1e9)
        if ax_*t>X-rc and ay_*t>Y-rc:
            cx_,cy_=X-rc,Y-rc;a=ax_*ax_+ay_*ay_;b=-2*(ax_*cx_+ay_*cy_);c=cx_*cx_+cy_*cy_-rc*rc
            return (-b+math.sqrt(max(0,b*b-4*a*c)))/(2*a)/t
        return 1
    def edge(px,py):
        ax_,ay_=abs(px),abs(py)
        if ax_>X-rc and ay_>Y-rc:return max(0,rc-math.hypot(ax_-X+rc,ay_-Y+rc))
        return max(0,min(X-ax_,Y-ay_))
    p0,p1=R.uniform(0,6),R.uniform(0,6);pts=[];raw=[]
    for py in ys:
        for px in xs:
            w=max(abs(px)/X,abs(py)/Y)**6;s=1-(1-(rho(abs(px),abs(py)) if (px or py) else 1))*w
            qx,qy=px*s,py*s;e=edge(qx,qy)
            if e<.8:zz=.15*math.sin(min(1,e/.8)*math.pi/2)        # bound edge
            elif e<1.6:zz=.15
            elif e<2.1:zz=.15-.06*math.sin((e-1.6)/.5*math.pi)    # stitch line
            else:zz=None;raw.append(.75*R.random()+.25*(.5+.5*math.sin(qx*1.9+p0)*math.sin(qy*1.6+p1)))
            pts.append([qx,qy,zz])
    lo,hi=min(raw),max(raw);it=iter(raw)
    for p in pts:
        if p[2] is None:p[2]=.13+.22*(next(it)-lo)/(hi-lo)   # pile tufts, top exactly at +0.35
    v=[tuple(p) for p in pts];f=[(j*nx+i,j*nx+i+1,(j+1)*nx+i+1,(j+1)*nx+i) for j in range(ny-1) for i in range(nx-1)]
    bnd=[i for i in range(nx-1)]+[j*nx+nx-1 for j in range(ny-1)]+[(ny-1)*nx+i for i in range(nx-1,0,-1)]+[j*nx for j in range(ny-1,0,-1)]
    m=len(bnd);s1=len(v);v+=[(v[b][0],v[b][1],-.27) for b in bnd];s2=len(v);v+=[(v[b][0]*.996,v[b][1]*.997,-h) for b in bnd]
    f+=[(s2+i,s2+(i+1)%m,s1+(i+1)%m,s1+i) for i in range(m)]+[(s1+i,s1+(i+1)%m,bnd[(i+1)%m],bnd[i]) for i in range(m)]
    c=len(v);v.append((0,0,-h));f+=[(c,s2+(i+1)%m,s2+i) for i in range(m)]
    mt=mesh('Pile bath mat',v,f,'fabric');mt.data.set_sharp_from_angle(angle=math.radians(55))

GROUP_BUILDERS.update({'Vanity':bathroom_vanity,'Vanity countertop':bathroom_countertop,'Vanity basin':bathroom_basin,
    'Vanity mirror frame':bathroom_frame,'Mirror opaque placeholder':bathroom_mirror,'Toilet base':bathroom_toilet_base,
    'Toilet bowl':bathroom_toilet_bowl,'Toilet tank':bathroom_toilet_tank,'Shower tray':bathroom_tray,'Shower upright':bathroom_upright,
    'Shower rail':bathroom_rail,'Shower folded curtain':bathroom_curtain,'Bathroom mat':bathroom_mat})

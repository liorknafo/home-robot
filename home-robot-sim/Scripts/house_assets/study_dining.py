# Study + dining group: tables with aprons, chairs, monitor, keyboard, bookcase,
# books, moving box, desk cable, fruit bowl and fruit. Footprints/heights unchanged;
# geometry inside the robot band (3.5-27 cm) keeps the previous silhouettes.
import bmesh

def study_dining_tone(c,f):
    # Derived shade of a hex colour.
    return '#'+''.join(f'{min(255,max(0,round(int(c[i:i+2],16)*f))):02x}' for i in (1,3,5))

def study_dining_smooth(o,deg=35):
    # Smooth shading split at creases sharper than deg (exported as corner normals).
    bm=bmesh.new();bm.from_mesh(o.data);a=math.radians(deg)
    for f in bm.faces:f.smooth=True
    for e in bm.edges:e.smooth=len(e.link_faces)==2 and e.calc_face_angle(9)<=a
    for f in bm.faces:
        if len(f.verts)>4:
            for e in f.edges:e.smooth=False  # flat n-gon caps keep crisp, even shading
    bm.to_mesh(o.data);bm.free();return o

def study_dining_rr(a,b,r,n=5,k=1):
    # Rounded rectangle outline (half sizes a,b), counter-clockwise; k splits each straight edge.
    r=max(.03,min(r,a-.02,b-.02));c=[]
    for q,(cx,cy) in enumerate([(a-r,b-r),(r-a,b-r),(r-a,r-b),(a-r,r-b)]):
        c.append([(cx+r*math.cos(math.pi/2*(q+i/n)),cy+r*math.sin(math.pi/2*(q+i/n))) for i in range(n+1)])
    p=[]
    for q in range(4):
        e,s=c[q][-1],c[(q+1)%4][0];p+=c[q]+[(e[0]+(s[0]-e[0])*j/k,e[1]+(s[1]-e[1])*j/k) for j in range(1,k)]
    return p

def study_dining_loft(rings,kind=None,color=None,caps=(1,1),deg=35,name='Lofted part'):
    # Quad skin through equal-length CCW rings (viewed from the travel direction), capped.
    m=len(rings[0]);v=[tuple(p) for r in rings for p in r]
    f=[(j*m+i,j*m+(i+1)%m,(j+1)*m+(i+1)%m,(j+1)*m+i) for j in range(len(rings)-1) for i in range(m)]
    if caps[0]:f.append(tuple(range(m-1,-1,-1)))
    if caps[1]:f.append(tuple(range(len(v)-m,len(v))))
    return study_dining_smooth(mesh(name,v,f,kind,color),deg)

STUDY_DINING_ROT={'z':lambda u,v,h:(u,v,h),'y':lambda u,v,h:(u,h,-v),'-y':lambda u,v,h:(u,-h,v),'x':lambda u,v,h:(h,v,-u)}
def study_dining_slab(prof,a,b,r,kind=None,color=None,n=5,rot='z',off=(0,0,0),warp=None,caps=(1,1),deg=35,k=1):
    # Rounded-rectangle rings (inset, height[, radius]) stacked along an axis.
    T=STUDY_DINING_ROT[rot];rings=[]
    for q in prof:
        d,h=q[:2];rad=q[2] if len(q)>2 else r-d
        ring=[tuple(c+o for c,o in zip(T(u,v,h),off)) for u,v in study_dining_rr(a-d,b-d,rad,n,k)]
        rings.append([warp(p) for p in ring] if warp else ring)
    return study_dining_loft(rings,kind,color,caps,deg,'Rounded slab')

def study_dining_sweep(pts,tans,rad,kind=None,color=None,sides=10,caps=(1,1)):
    # Round tube along points with given tangents (floor-level cables).
    rings=[]
    for p,t,r in zip(pts,tans,rad if isinstance(rad,list) else [rad]*len(pts)):
        t=Vector(t).normalized();nn=t.cross(Vector((0,0,1))).normalized();u=t.cross(nn)
        rings.append([tuple(Vector(p)+r*(math.cos(2*math.pi*k/sides)*nn+math.sin(2*math.pi*k/sides)*u)) for k in range(sides)])
    return study_dining_loft(rings,kind,color,caps,60,'Swept cable')

def study_dining_local(o,q):
    # Centre of scene object q in o's local frame (cm).
    a=-math.radians(o['yaw']);dx=q['center'][0]-o['center'][0];dy=q['center'][1]-o['center'][1]
    return (dx*math.cos(a)-dy*math.sin(a),dx*math.sin(a)+dy*math.cos(a),q['center'][2]-o['center'][2])

def study_dining_near(o,name,reach):
    return [(study_dining_local(o,q),q['size']) for q in scene['objects'] if q['name']==name and math.dist(q['center'][:2],o['center'][:2])<reach]

# ---------------------------------------------------------------- legs, tables
def study_dining_leg(o,index,size):
    x,y,z=size;g=.3;s=lambda h:.76+.24*(h+z/2)/z
    # Same 76 % taper as before (robot-band silhouette unchanged), eased edges, felt glide.
    v=[(a*x/2*s(h),b*y/2*s(h),h) for h in [-z/2+g,z/2] for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]]
    leg=mesh('Tapered solid-wood leg',v,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],'wood',col)
    b=leg.modifiers.new('Eased edges','BEVEL');b.width=min(.25,x*.07);b.segments=3;b.harden_normals=True
    n=leg.modifiers.new('Face normals','WEIGHTED_NORMAL');n.keep_sharp=True;n.weight=40
    lathe([(0,-z/2),(.9,-z/2),(1,-z/2+g*.6),(.95,-z/2+g+.03),(0,-z/2+g+.03)],(x/2*.62,y/2*.62),'fabric','#62594f',16)

def study_dining_top(o,index,size):
    x,y,z=size;lift=.48 if 'desk' in o['name'].lower() else 0
    # Solid board with softened edge profile (desk raised 4.5 mm so the keyboard feet land).
    t=z/2+lift
    study_dining_slab([(.45,-.15),(.12,.05),(0,.4),(0,t-.45),(.1,t-.12),(.35,t)],x/2,y/2,1.2,'wood',col,6)
    # Apron rails + corner blocks tie the leg tops together under the board.
    legs=study_dining_near(o,o['name'].replace(' top',' leg'),math.hypot(x,y)/2+1)
    lx=max(abs(p[0]) for p,s in legs);ly=max(abs(p[1]) for p,s in legs);h=legs[0][1][0]/2;rt=1.8;za,zb=-z/2-.45,-.1
    for s in [-1,1]:
        box((0,s*(ly+h-.4-rt/2),(za+zb)/2),(2*lx,rt,zb-za),.12,'wood',col)
        box((s*(lx+h-.4-rt/2),0,(za+zb)/2),(rt,2*ly,zb-za),.12,'wood',col)
    for p,sz in legs:box((p[0],p[1],(-z/2+zb-.06)/2),(sz[0],sz[1],zb-.06+z/2),.22,'wood',col)  # top clear of the rails'

# ---------------------------------------------------------------- chairs
def study_dining_seat(o,index,size):
    x,y,z=size;wood='#9a7350'
    legs=study_dining_near(o,'Dining chair leg',32);posts=study_dining_near(o,'Chair back post',32)
    lx=max(abs(p[0]) for p,s in legs);ly=max(abs(p[1]) for p,s in legs);h=legs[0][1][0]/2;rt=1.6
    ox,oy=lx+h-.15,ly+h-.15
    # Seat frame: aprons over the leg tops, corner blocks (taking the back-post feet), thin deck.
    for s in [-1,1]:
        box((0,s*(oy-rt/2),-z/2+.85),(2*ox,rt,1.7),.15,'wood',wood)
        box((s*(ox-rt/2),0,-z/2+.85),(rt,2*oy-2*rt+.3,1.7),.15,'wood',wood)
    for p,sz in legs:
        lo=[p[0]-h,p[1]-h];hi=[p[0]+h,p[1]+h]
        for q,qs in posts:
            if math.dist(q[:2],p[:2])<4:lo=[min(lo[i],q[i]-qs[i]/2) for i in (0,1)];hi=[max(hi[i],q[i]+qs[i]/2) for i in (0,1)]
        box(((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,-z/2+.79),(hi[0]-lo[0],hi[1]-lo[1],1.7),.2,'wood',wood)  # 0.6 mm proud of the aprons
    box((0,0,-.49),(2*ox-.3,2*oy-.3,.3),.06,'wood',study_dining_tone(wood,.9))
    # Upholstered pad: rounded overhang, crowned top, piped seam at the widest line.
    study_dining_slab([(.9,-.42),(.3,-.3),(.05,-.02),(0,.32),(.1,.75),(.42,1.25),(1.1,1.62),(2.4,1.86,2.2),(5,1.97,1.6),(9,2,1)],x/2,y/2,3.2,'fabric',col,6)
    tube([(px,py,.32) for px,py in study_dining_rr(x/2-.02,y/2-.02,3.18,6)],.12,'fabric',study_dining_tone(col,.9),True)

def study_dining_post(o,index,size):
    x,y,z=size
    # Back post: slims and rakes back (+y) towards a rounded top.
    v=[(a*x/2*s,b*y/2*s+dy,h) for h,s,dy in [(-z/2,1,0),(z/2,.8,.45)] for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]]
    p=mesh('Raked back post',v,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],'wood',col)
    b=p.modifiers.new('Rounded post','BEVEL');b.width=.32;b.segments=3;b.harden_normals=True
    n=p.modifiers.new('Face normals','WEIGHTED_NORMAL');n.keep_sharp=True;n.weight=40

def study_dining_back(o,index,size):
    x,y,z=size;w=x/2;bow=2.0;y0=-1.0;yc=lambda u:y0+bow*(1-(u/w)**2)
    # Curved upholstered back rest, concave towards the sitter (-y).
    ends=[(1.0,-w),(.4,-w+.25),(.1,-w+.8),(0,-w+1.5)]
    mid=[(0,-w+2.5+i*(2*w-5)/12) for i in range(13)]
    prof=ends+mid+[(d,-h) for d,h in reversed(ends)]
    # Front face crowns ~3 mm towards the sitter (padding), fading to the seams and ends.
    crown=lambda p:.32*max(0,1-(p[0]/w)**4)*max(0,1-(2*p[2]/z)**2)*min(0,p[1])
    study_dining_slab(prof,z/2,1.0,.8,'fabric',col,4,'x',warp=lambda p:(p[0],p[1]+yc(p[0])+crown(p),p[2]),k=6)

# ---------------------------------------------------------------- monitor + keyboard
def study_dining_monitor_base(o,index,size):
    x,y,z=size;b=-z/2+.48
    # Low plate with a sloped top edge, resting on the raised desk surface.
    study_dining_slab([(.25,b),(0,b+.15),(0,b+.55),(.9,z/2-.12),(1.4,z/2,3)],x/2,y/2,4,'metal',col,6)

def study_dining_monitor_stem(o,index,size):
    x,y,z=size
    # Flat neck rising from the base's back edge and leaning into the screen housing.
    study_dining_slab([(0,-z/2),(0,z/2-.6),(.35,z/2-.15),(.8,z/2)],x/2-.1,1.0,.75,'metal',col,4,
        warp=lambda p:(p[0],p[1]-1.5+3*(p[2]+z/2)/z,p[2]))

def study_dining_monitor_screen(o,index,size):
    x,y,z=size;dark=study_dining_tone(col,.62)
    # Thin panel (front at -y), narrow bezel + chin, inset display, rear housing hump.
    study_dining_slab([(.25,1.25),(0,1.42),(0,1.92),(.08,2.0)],x/2,z/2,.6,'plastic',dark,4,'-y')
    hh=(z-.75-1.9)/2
    study_dining_slab([(0,1.95),(0,2.06)],x/2-.75,hh,.25,'plastic',col,3,'-y',(0,0,-z/2+1.9+hh))
    study_dining_slab([(0,-1.3),(.35,0),(1.6,1.2),(3.3,1.85),(4.4,2.0)],20.5,12.5,4.5,'plastic',dark,6,'y',(0,0,-1.5))

def study_dining_keyboard(o,index,size):
    x,y,z=size;body=study_dining_tone(col,.78)
    top=lambda yy:.75-.8*(y/2-yy)/y
    # Wedge body (low front edge), rubber feet down to the desk.
    study_dining_slab([(.2,-.98),(0,-.85),(0,.6),(.18,.75)],x/2,y/2,.8,'plastic',body,4,
        warp=lambda p:(p[0],p[1],p[2] if p[2]<0 else p[2]-.8*(y/2-p[1])/y))
    for a in [-1,1]:
        for yy in [-y/2+1.2,y/2-1.2]:box((a*(x/2-2.5),yy,-1.205),(2.4,1.0,.48),.12,'rubber','#2f322e')
    # Keycaps: compact full-size ANSI layout (1.9 cm pitch) + numpad, one sculpted mesh.
    u=1.9;rows=[[1,-1,1,1,1,1,-.5,1,1,1,1,-.5,1,1,1,1],[1]*13+[2],[1.5]+[1]*12+[1.5],[1.75]+[1]*11+[2.25],[2.25]+[1]*10+[2.75],[1.25]*3+[6.25]+[1.25]*4]
    x0=-x/2+1.65;v=[];f=[]
    def cap(cx,cy,w,d):
        z0=top(cy)-.12;i=len(v)
        for dx,dy,zz in [(w/2,d/2,z0),(w/2-.1,d/2-.12,z0+.42),(w/2-.24,d/2-.26,z0+.52)]:
            v.extend([(cx-dx,cy-dy,zz),(cx+dx,cy-dy,zz),(cx+dx,cy+dy,zz),(cx-dx,cy+dy,zz)])
        for r0 in (0,4):
            for k in range(4):f.append((i+r0+k,i+r0+(k+1)%4,i+r0+4+(k+1)%4,i+r0+4+k))
        f.append((i+8,i+9,i+10,i+11))
    for r,row in enumerate(rows):
        cy=y/2-1.75-r*u-(.2 if r else 0);cx=x0
        for w in row:
            if w>0:cap(cx+w*u/2,cy,w*u-.18,u-.18)
            cx+=abs(w)*u
    nx=x0+15*u+.6
    for c,r,w,d in [(0,1,1,1),(1,1,1,1),(2,1,1,1),(3,1,1,1),(0,2,1,1),(1,2,1,1),(2,2,1,1),(3,2,1,2),(0,3,1,1),(1,3,1,1),(2,3,1,1),
                    (0,4,1,1),(1,4,1,1),(2,4,1,1),(3,4,1,2),(0,5,2,1),(2,5,1,1)]:
        cap(nx+(c+w/2)*u,y/2-1.75-.2-(r+d/2-.5)*u,w*u-.18,d*u-.18)
    study_dining_smooth(mesh('Sculpted keycaps',v,f,'plastic',col),30)

# ---------------------------------------------------------------- bookcase + books
def study_dining_case_for(q):
    for c in scene['objects']:
        if c['name']=='Low bookcase' and abs(q['center'][0]-c['center'][0])<c['size'][0]/2+6 and abs(q['center'][1]-c['center'][1])<c['size'][1]/2:return c

def study_dining_shelves(case):
    # Shelf boards (world z0,z1) under each book row; none may enter the robot band.
    rows={}
    for q in scene['objects']:
        if q['name']=='Book spine' and study_dining_case_for(q) is case:rows.setdefault(q['center'][2],[]).append(q['center'][2]-q['size'][2]/2)
    return [(max(b)-.45-2.15,max(b)-.45) for zc,b in sorted(rows.items()) if max(b)-.45-2.15>27.3]

def study_dining_bookcase(o,index,size):
    x,y,z=size;cz=o['center'][2];sd=2.4;iy=y/2-sd+.9;B=-z/2
    # Carcass: full sides (as before), top across the sides, recessed plinth.
    for s in [-1,1]:box((0,s*(y/2-sd/2),-1.2),(x,sd,z-2.4),.3,'wood',col)
    box((0,0,z/2-1.2),(x,y,2.4),.35,'wood',col)
    box((-1.025,0,B+2.2),(x-.3-2.4+.05,2*iy,2.0),.25,'wood',col)
    box((-x/2+2.4,0,B+.62),(1.8,2*iy,1.24),.15,'wood',study_dining_tone(col,.8))
    # Back panel (band part unchanged) + a lining above the band carrying the dark inset boards.
    box((x/2-1.26,0,(B+z/2-2.4)/2),(2.4,2*iy,z-2.4+.05),.2,'wood',col)  # rear face 0.6 mm inside the sides'
    box((x/2-3.05,0,(B+27.3+z/2-2.4)/2),(1.4,2*(y/2-sd)+.1,z-2.4-27.3+.05),.08,'wood',study_dining_tone(col,.93))
    for z0,z1 in study_dining_shelves(o):
        box((-1.7,0,(z0+z1)/2-cz),(x-.3-3.75+.05,2*iy,z1-z0),.3,'wood',col)

def study_dining_shadow(o,index,size):
    x,y,z=size;cz=o['center'][2];lo,hi=-z/2-.45,z/2+.45
    case=study_dining_case_for(o)
    for z0,z1 in (study_dining_shelves(case) if case else []):
        if z0<cz-z/2+1<z1:lo=z0+.05-cz
    # Dark-stained inset back board, flush on the bookcase back.
    box((39,0,(lo+hi)/2),(x,y+.9,hi-lo),.08,'wood',col)

def study_dining_book(o,index,size):
    t,w,h=size;D=26;cz=o['center'][2];zb,zt=-h/2,h/2;hard=index%3!=1
    case=study_dining_case_for(o)
    for z0,z1 in (study_dining_shelves(case) if case else []):
        if z0-1<cz-h/2<z1+1:zb=min(max(z1-cz,-h/2-.45),-h/2+.45)
    cb,sq,xc,sg,x0=(.22,.3,.1,.45,.32) if hard else (.07,.08,-.15,.2,.14)
    ry=w/2-(.07 if hard else .06)
    # Page block with a slightly concave fore-edge.
    py=w/2-cb-.02;fe=D-sq;poly=[(.9,-py)]+[(fe-(.32 if hard else .05)*math.cos(math.pi*(i/8-.5)),py*(i/4-1)) for i in range(9)]+[(.9,py)]
    study_dining_loft([[(px,pyy,zz) for px,pyy in poly] for zz in (zb+sq,zt-sq)],'paper','#e4dccb',deg=40,name='Page block')
    # Boards (covers) and a rounded spine with hinge grooves.
    for s in [-1,1]:box(((x0+D)/2,s*(w/2-cb/2),(zb+zt)/2),(D-x0,cb,zt-zb),.06,'book',col)
    Rs=(ry*ry+sg*sg)/(2*sg);ax=lambda yy:xc-(math.sqrt(Rs*Rs-yy*yy)-(Rs-sg))  # gentle circular spine arc
    arc=[(ax(yy),yy) for yy in [ry-2*ry*i/12 for i in range(13)]]+[(1.2,-ry),(1.2,ry)]
    rings=[]  # short quads along the height keep highlights even
    for zz,k in [(zb,.9)]+[(zb+.07+(zt-zb-.14)*i/8,1) for i in range(9)]+[(zt,.9)]:
        rings.append([(xc+.3+(px-xc-.3)*(1 if k==1 else .97),pyy*k,zz) for px,pyy in arc])
    study_dining_loft(rings,'book',col,deg=40,name='Rounded spine')
    if hard:
        # Two low raised bands across the spine (bookbinding ribs), a shade darker.
        ys=[.9*ry-1.8*ry*i/8 for i in range(9)];band=[(ax(yy)-.06,yy) for yy in ys]+[(ax(yy)+.3,yy) for yy in ys[::-1]]
        for zc in (zt-(zt-zb)*.13,zb+(zt-zb)*.13):
            study_dining_loft([[(px,pyy,zz) for px,pyy in band] for zz in (zc-.16,zc+.16)],'book',study_dining_tone(col,.86),deg=50,name='Spine band')

# ---------------------------------------------------------------- box, tape, cable
def study_dining_box(o,index,size):
    x,y,z=size;fl=.45;tape=next((q for q in scene['objects'] if q['name']=='Box tape' and math.dist(q['center'][:2],o['center'][:2])<5),None)
    # Corrugated carton: body, two closing flaps with a seam gap, glued manufacturer's joint.
    box((0,0,-fl/2),(x,y,z-fl),.3,'cardboard',col)
    for s in [-1,1]:box((s*(x/4+.09),0,z/2-fl/2-.02),(x/2-.18,y,fl),.14,'cardboard',study_dining_tone(col,1.02))
    box((x/2-1.6,-y/2-.05,-fl/2),(3.0,.1,z-fl-.6),.03,'cardboard',study_dining_tone(col,.95))
    if tape:
        # Tape continues down both end faces under the lid strip.
        tw=tape['size'][0];tc=tape['color']
        for s in [-1,1]:box((0,s*(y/2+.075),z/2-4.4),(tw,.05,8.2),.02,'plastic',tc)
    # Die-cut hand-hold flaps on the narrow ends: a thin cut line, flap still closed.
    for s in [-1,1]:
        out=[];inn=[]
        for i in range(32):
            t=2*math.pi*i/32;cy=(3.2 if math.cos(t)>0 else -3.2)
            for r,L in [(1.3,out),(1.12,inn)]:L.append((s*(x/2+.05),s*(cy+r*math.cos(t)),z/2-7.5+r*math.sin(t)))
        v=out+inn;f=[(i,(i+1)%32,32+(i+1)%32,32+i) for i in range(32)]
        mesh('Hand-hold cut',v,f,'cardboard',study_dining_tone(col,.6),False)

def study_dining_tape(o,index,size):
    x,y,z=size
    # Thin packing tape over the seam, folding over both lid edges.
    Y=y/2+.18;t=-.135
    pts=[(0,-Y,-.65),(0,-Y,-.32),(0,-Y+.06,t-.045),(0,-y/2+.02,t),(0,y/2-.02,t),(0,Y-.06,t-.045),(0,Y,-.32),(0,Y,-.65)]
    v=[];f=[]
    for i,(px,py,pz) in enumerate(pts):
        v+= [(-x/2,py,pz),(x/2,py,pz)]
    for i in range(len(pts)-1):f.append((2*i,2*i+1,2*i+3,2*i+2))
    me=mesh('Packing tape',v,f,'plastic',col);s=me.modifiers.new('Tape film','SOLIDIFY');s.thickness=.03;s.offset=-1

def study_dining_cable(o,index,size):
    x,y,z=size;chain=[i for i,q in enumerate(scene['objects']) if q['name']==o['name']];k=chain.index(index)
    P=[Vector(scene['objects'][i]['center'][:2]) for i in chain];L=y/2
    C=[P[0]-Vector((0,L))]+P+[P[-1]+Vector((0,L))];C=[2*C[0]-C[1]]+C+[2*C[-1]-C[-2]]
    def cr(j,t,d=False):
        p0,p1,p2,p3=C[j-1],C[j],C[j+1],C[j+2]
        if d:return .5*((p2-p0)+2*(2*p0-5*p1+4*p2-p3)*t+3*(-p0+3*p1-3*p2+p3)*t*t)
        return .5*(2*p1+(p2-p0)*t+(2*p0-5*p1+4*p2-p3)*t*t+(-p0+3*p1-3*p2+p3)*t**3)
    # Catmull-Rom path through every section centre; neighbours share joint points + tangents.
    j=k+2;seg=[(j-1,.5+.125*i) for i in range(5)]+[(j,.125*i) for i in range(1,5)]
    if k==0:seg=[(1,.25*i) for i in range(4)]+[(2,.125*i) for i in range(5)]
    if k==len(chain)-1:seg=[(j-1,.5+.125*i) for i in range(5)]+[(j,.25*i) for i in range(1,5)]
    c=Vector(o['center'][:2]);pts=[(*(cr(a,t)-c),0) for a,t in seg];tans=[(*cr(a,t,True),0) for a,t in seg]
    study_dining_sweep(pts,tans,x/2,'rubber',col,10,(k==0,k==len(chain)-1))
    plug=study_dining_tone(col,.75)
    if k==0:
        study_dining_slab([(.2,-L-.45),(0,-L-.3),(0,-1.25),(.12,-1.0)],.75,.48,.35,'plastic',plug,4,'y',(0,0,.13))
        study_dining_sweep([(0,-1.05,.12),(0,-.4,.03)],[(0,1,0)]*2,[.47,.38],'plastic',plug,10,(0,1))
    if k==len(chain)-1:
        study_dining_slab([(.12,1.0),(0,1.25),(0,L+.3),(.2,L+.45)],.8,.5,.3,'plastic',plug,4,'y',(0,0,.15))
        study_dining_sweep([(0,.4,.03),(0,1.05,.12)],[(0,1,0)]*2,[.38,.47],'plastic',plug,10,(1,0))

# ---------------------------------------------------------------- dining table dressing
def study_dining_runner(o,index,size):
    x,y,z=size;bw=1.8;edge=study_dining_tone(col,.86)
    # Woven runner: plain field, tonal borders and hemmed ends (same 2 mm thickness).
    for s in [-1,1]:
        box((s*(x/2-bw/2),0,0),(bw,y,z),.03,'fabric',edge)
        box((0,s*(y/2-.6),0),(x-2*bw,1.2,z),.03,'fabric',edge)
    box((0,0,0),(x-2*bw,y-2.4,z),.03,'fabric',col)

# Footed bowl profile (r, h in cm for a 23 x 8 bowl); the foot stands 2 mm up on the runner.
STUDY_DINING_BOWL=[(0,-4),(4.6,-4),(5.3,-3.85),(5.2,-3.45),(3.8,-3.05),(3.25,-2.55),(3.6,-2.0),(6.2,-1.35),(9.2,-.2),(10.9,1.6),(11.5,3.6),(11.45,3.95),(11.2,4),
    (10.95,3.95),(10.8,3.5),(9.9,2.2),(8.3,1.15),(6.9,.75),(3.5,.62),(0,.6)]
def study_dining_bowl_h(h):return -3.8+(h+4)*7.8/8

def study_dining_bowl(o,index,size):
    x,y,z=size;R=x/2
    # Footed fruit bowl: turned foot, short stem, shallow dish with a raised flat floor.
    lathe([(r/R,study_dining_bowl_h(h)*z/8) for r,h in STUDY_DINING_BOWL],(R,y/2),'ceramic',col,48)

def study_dining_bowl_floor(q,b):
    # Height of the bowl's inner surface (bowl-local) under fruit q.
    r=math.dist(q['center'][:2],b['center'][:2]);pts=STUDY_DINING_BOWL[-1:-7:-1]
    for (r0,h0),(r1,h1) in zip(pts,pts[1:]):
        if r<=r1:return study_dining_bowl_h(h0+(h1-h0)*(r-r0)/(r1-r0))
    return study_dining_bowl_h(3.5)

def study_dining_cr_sub(prof):
    # Insert Catmull-Rom midpoints into a 2D profile (smoother lathed silhouettes).
    q=[prof[0]]+prof+[prof[-1]];out=[]
    for i in range(1,len(q)-2):
        p0,p1,p2,p3=q[i-1],q[i],q[i+1],q[i+2];out.append(p1)
        out.append(tuple(.5*(2*p1[k]+.5*(p2[k]-p0[k])+.25*(2*p0[k]-5*p1[k]+4*p2[k]-p3[k])+.125*(-p0[k]+3*p1[k]-3*p2[k]+p3[k])) for k in (0,1)))
    return out+[prof[-1]]

def study_dining_fruit(o,index,size):
    x,y,z=size;bowl=next((q for q in scene['objects'] if q['name']=='Fruit bowl' and math.dist(q['center'][:2],o['center'][:2])<12),None)
    pear=index%2==1;rng2=random.Random(index)
    bottom=-3.3 if not pear else -3.4
    if bowl:bottom=min(max(bowl['center'][2]+study_dining_bowl_floor(o,bowl)-o['center'][2],-3.55),-2.65)
    if pear:prof=[(0,.25),(.55,.1),(1.35,0),(2.15,.2),(2.8,.85),(3.08,1.85),(2.95,2.85),(2.55,3.85),(1.95,4.75),(1.55,5.55),(1.25,6.25),(.85,6.85),(.4,7.1),(0,7.05)];prof=[(r,h*.95) for r,h in prof]
    else:prof=[(0,.42),(.45,.3),(.95,.12),(1.55,0),(2.25,.22),(2.85,.95),(3.22,2.05),(3.3,3.25),(3.2,4.4),(2.85,5.4),(2.25,6.2),(1.5,6.55),(.8,6.42),(.35,6.12),(0,5.95)]
    prof=study_dining_cr_sub(prof)
    # Lathed fruit body with five soft lobes, resting on the bowl floor.
    seg=28;tilt=rng2.uniform(0,2*math.pi);v=[];f=[]
    for r,h in prof:
        for i in range(seg):
            a=2*math.pi*i/seg;k=1+(.016 if not pear else .01)*math.cos(5*a+tilt)*min(1,r/1.5)
            v.append((r*k*math.cos(a),r*k*math.sin(a),bottom+h))
    for j in range(len(prof)-1):
        for i in range(seg):f.append((j*seg+i,j*seg+(i+1)%seg,(j+1)*seg+(i+1)%seg,(j+1)*seg+i))
    study_dining_smooth(mesh('Lathed fruit',v,f,'fruit','#b9a552' if pear else '#ba6751'),50)
    top=bottom+prof[-1][1];lean=rng2.uniform(-.25,.25);L=.55 if pear else .8
    tube([(0,0,top-.15),(lean*.4,.05,top+L*.5),(lean,.12,top+L)],.11,'wood','#5a4a2c')

GROUP_BUILDERS.update({
    'Dining table leg':study_dining_leg,'Writing desk leg':study_dining_leg,'Dining chair leg':study_dining_leg,
    'Dining table top':study_dining_top,'Writing desk top':study_dining_top,
    'Dining chair seat':study_dining_seat,'Chair back post':study_dining_post,'Chair back':study_dining_back,
    'Monitor base':study_dining_monitor_base,'Monitor stem':study_dining_monitor_stem,'Monitor screen':study_dining_monitor_screen,
    'Keyboard':study_dining_keyboard,'Low bookcase':study_dining_bookcase,'Shelf shadow':study_dining_shadow,'Book spine':study_dining_book,
    'Cardboard moving box':study_dining_box,'Box tape':study_dining_tape,'Desk power cable':study_dining_cable,
    'Table runner':study_dining_runner,'Fruit bowl':study_dining_bowl,'Fruit':study_dining_fruit})

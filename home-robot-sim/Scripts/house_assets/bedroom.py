# Bedroom group: slatted bed, bedding, nightstand + lamp, wardrobe and the two open interior doors.
# Envelopes match the baseline; details stay inside each object's existing bounds.
import bmesh as bedroom_bmesh
from mathutils import Matrix as bedroom_Matrix

BEDROOM_MR=4.0  # mattress edge radius, shared by mattress, duvet and pillows
BEDROOM_OBJ={}
for _o in scene['objects']:BEDROOM_OBJ.setdefault(_o['name'],_o)
BEDROOM_PILLOWS=[_o for _o in scene['objects'] if _o['name']=='Pillow']

def bedroom_shade(c,f):
    return '#'+''.join(f'{max(0,min(255,round(int(c[i:i+2],16)*f))):02x}' for i in (1,3,5))

def bedroom_smooth(a,b,t):
    t=min(1,max(0,(t-a)/(b-a)));return t*t*(3-2*t)

def bedroom_round(d,r):
    # 0 at a soft edge, 1 once d>=r, with a vertical tangent at the edge (rolled fabric)
    t=min(1,max(0,d)/r);return math.sqrt(1-(1-t)**2)

def bedroom_close(o):
    # Weld shared seams, then point every face outward.
    bm=bedroom_bmesh.new();bm.from_mesh(o.data);bedroom_bmesh.ops.remove_doubles(bm,verts=bm.verts,dist=1e-4)
    bedroom_bmesh.ops.recalc_face_normals(bm,faces=bm.faces)
    # recalc can flip thin pinched shells (pillows) inward: enforce a positive enclosed volume
    if sum(f.verts[0].co.dot(f.verts[i].co.cross(f.verts[i+1].co)) for f in bm.faces for i in range(1,len(f.verts)-1))<0:bedroom_bmesh.ops.reverse_faces(bm,faces=bm.faces[:])
    bm.to_mesh(o.data);bm.free()
    for p in o.data.polygons:p.use_smooth=True
    o.data.update();return o

def bedroom_rod(p0,p1,r,kind=None,color=None,seg=16,end=.3):
    # Closed turned rod between two points, softly chamfered ends.
    p0,p1=Vector(p0),Vector(p1);d=p1-p0;L=d.length;e=min(end*r,L/3)
    o=lathe([(0,0),(r*.72,0),(r,e),(r,L-e),(r*.72,L),(0,L)],(1,1),kind,color,seg)
    o.data.transform(bedroom_Matrix.Translation(p0)@Vector((0,0,1)).rotation_difference(d).to_matrix().to_4x4());o.data.update();return o

def bedroom_rrect(hx,hy,r,z,k=6):
    pts=[]
    for cx,cy,a0 in [(hx-r,hy-r,0),(-hx+r,hy-r,90),(-hx+r,-hy+r,180),(hx-r,-hy+r,270)]:
        pts+=[(cx+r*math.cos(math.radians(a0+90*i/k)),cy+r*math.sin(math.radians(a0+90*i/k)),z) for i in range(k+1)]
    return pts

def bedroom_blob(size,pos,e=(.55,.12),n=(20,12),kind=None,color=None):
    # Superellipsoid padding (long axis z) with single pole vertices.
    sx,sy,sz=size;nu,nv=n;sq=lambda a,p:math.copysign(abs(a)**p,a)
    v=[(pos[0],pos[1],pos[2]-sz/2)];f=[]
    for j in range(1,nv):
        phi=-math.pi/2+math.pi*j/nv;c=abs(math.cos(phi))**e[1]
        v+=[(pos[0]+sx/2*c*sq(math.cos(t),e[0]),pos[1]+sy/2*c*sq(math.sin(t),e[0]),pos[2]+sz/2*sq(math.sin(phi),e[1])) for t in [2*math.pi*i/nu for i in range(nu)]]
    v.append((pos[0],pos[1],pos[2]+sz/2));top=len(v)-1
    for i in range(nu):f.append((0,1+(i+1)%nu,1+i));f.append((top,1+(nv-2)*nu+i,1+(nv-2)*nu+(i+1)%nu))
    for j in range(nv-2):
        for i in range(nu):a=1+j*nu;f.append((a+i,a+(i+1)%nu,a+nu+(i+1)%nu,a+nu+i))
    return bedroom_close(mesh('Padded channel',v,f,kind,color))

def bedroom_sheet(name,rows,kind=None,color=None):
    # rows[j][i]=(x,y,top,bottom); top and bottom grids meet along the border.
    m,n=len(rows),len(rows[0]);v=[(p[0],p[1],p[2]) for r in rows for p in r]+[(p[0],p[1],p[3]) for r in rows for p in r];f=[]
    for s in (0,n*m):
        for j in range(m-1):
            for i in range(n-1):a=s+j*n+i;q=(a,a+1,a+n+1,a+n);f.append(q if s==0 else q[::-1])
    return bedroom_close(mesh(name,v,f,kind,color))

def bedroom_axis(a,b,step,edge=(0,.15,.5,1.0,1.7,2.6,3.6)):
    # samples dense at both soft edges, even spacing inside
    k=max(1,round((b-a-2*edge[-1])/step));inner=[a+edge[-1]+(b-a-2*edge[-1])*i/k for i in range(1,k)]
    return [round(a+e,3) for e in edge]+[round(q,3) for q in inner]+[round(b-e,3) for e in edge[::-1]]

def bedroom_mattress_top(xw,yw):
    m=BEDROOM_OBJ['Mattress'];r=BEDROOM_MR
    dx=max(0,abs(xw-m['center'][0])-(m['size'][0]/2-r));dy=max(0,abs(yw-m['center'][1])-(m['size'][1]/2-r));d=min(r,math.hypot(dx,dy))
    return m['center'][2]+m['size'][2]/2-(r-math.sqrt(r*r-d*d))

def bedroom_duvet_ext(o):
    return o['size'][0]/2+.1,-o['size'][1]/2-.15,o['size'][1]/2+.1

def bedroom_duvet_t(xl,yl,o):
    """Duvet loft above the mattress (cm), duvet-local coordinates."""
    X,Y0,Y1=bedroom_duvet_ext(o)
    w=(.32*math.sin(.085*xl+.04*yl+1.3)*math.cos(.06*yl-.4)+.2*math.sin(.19*xl-.11*yl+.7)+.15*math.cos(.05*xl+.23*yl)
       +.16*math.cos(2*math.pi*xl/27)*math.cos(2*math.pi*yl/27)                          # baffles showing through the cover
       -1.0*math.exp(-((.62*xl+.78*yl+18)/4.2)**2)-.8*math.exp(-((.7*xl-.71*yl-30)/4.6)**2)   # soft creases
       -.7*math.exp(-((.95*xl-.3*yl+42)/5)**2)*bedroom_smooth(10,-40,yl)+.45*math.exp(-((xl-.15*yl-8)/9)**2))  # pulled fold
    w+=.8*math.sin(2*math.pi*yl/17+.8+.03*xl)*math.exp(-(X-abs(xl))/6)                      # ripples rolling over the sides
    yf=Y1-24;t=3.4+w+(1.3*bedroom_round(yl-yf,1.4) if yl>yf else 0)                         # turned-down top band
    t*=bedroom_round(X-abs(xl),3.4)*bedroom_round(yl-Y0,3.4)*bedroom_round(Y1-yl,2.8)
    xw,yw=o['center'][0]+xl,o['center'][1]+yl
    for p in BEDROOM_PILLOWS:  # pillows press the head band flatter
        hx,hy=p['size'][0]/2,p['size'][1]/2
        t*=1-.5*bedroom_smooth(hx+4,hx-2,abs(xw-p['center'][0]))*bedroom_smooth(hy+4,hy-2,abs(yw-p['center'][1]))
    return max(0,t)

def bedroom_duvet_top(xw,yw):
    d=BEDROOM_OBJ['Duvet'];X,Y0,Y1=bedroom_duvet_ext(d);xl,yl=xw-d['center'][0],yw-d['center'][1]
    if abs(xl)>X or not Y0<=yl<=Y1:return None
    return bedroom_mattress_top(xw,yw)+.04+bedroom_duvet_t(xl,yl,d)

# ---------------------------------------------------------------- bed
def bedroom_bed_frame(o,index,size):
    x,y,z=size;t=2.6;pine=bedroom_shade(col,1.12);cx,cy,cz=o['center']
    for s in (-1,1):
        box((s*(x/2-t/2),0,0),(t,y,z),.6)                         # side rails, full length
        box((0,s*(y/2-t/2),0),(x-2*t+.2,t,z),.6)                  # end rails between them
        box((s*(x/2-t-.95),0,5.4),(2,y-2*t+.1,2),0,'wood',pine)    # slat ledgers
    ms=BEDROOM_OBJ['Mattress'];top=ms['center'][2]-ms['size'][2]/2-cz-.05
    for i in range(14):box((0,-y/2+t+5.6+i*(y-2*t-11.2)/13,top-.8),(x-2*t-.1,7,1.6),0,'wood',pine)   # slats
    box((0,0,(top-1.6-z/2+3)/2+.05),(4,y-2*t+.1,top-1.6+z/2-3+.1),0,'wood',pine)                    # centre beam
    for lg in [p for p in scene['objects'] if p['name']=='Bed frame leg']:   # leg mounting blocks
        lx,ly=lg['center'][0]-cx,lg['center'][1]-cy;sx,sy=math.copysign(1,lx),math.copysign(1,ly)
        x0,x1=sorted((lx-sx*4.5,sx*(x/2-t+.05)));y0,y1=sorted((ly-sy*4.5,sy*(y/2-t+.05)))
        box(((x0+x1)/2,(y0+y1)/2,-z/2+3.775),(x1-x0,y1-y0,7.45),0,'wood',pine)

def bedroom_bed_leg(o,index,size):
    x,y,z=size;h=z/2;f=BEDROOM_OBJ['Bed frame'];tl=f['center'][2]-f['size'][2]/2-o['center'][2]   # frame underside
    lathe([(0,-h),(1.8,-h),(1.9,-h+.1),(1.92,-h+1.6),(1.82,-h+1.72),(0,-h+1.72)],(1,1),'metal','#8e7d58',24)   # brass ferrule
    lathe([(0,-h+1.66),(1.77,-h+1.66),(1.85,-h+1.8),(2.75,tl),(2.75,tl+.4),(0,tl+.4)],(1,1),'wood',col,24)      # turned taper
    box((0,0,(tl+.2+h)/2),(x,y,h-tl-.2),.2)                       # square mounting block inside the frame

def bedroom_mattress(o,index,size):
    x,y,z=size;r=BEDROOM_MR;b=box((0,0,0),(x,y,z),r,'fabric');b.modifiers[0].segments=5
    # tape edges where the rounded top and bottom panels meet the vertical border (clear of the duvet edge)
    for s in (-1,1):tube(bedroom_rrect(x/2-.05,y/2-.05,r-.05,s*(z/2-r+.15),7),.3,'fabric',bedroom_shade(col,.84),True)

def bedroom_duvet(o,index,size):
    X,Y0,Y1=bedroom_duvet_ext(o);cx,cy,cz=o['center'];yf=Y1-24
    xs=bedroom_axis(-X,X,3.4);ys=sorted(set(bedroom_axis(Y0,Y1,3.4))|{round(yf+d,3) for d in (-.5,0,.15,.4,.7,1.05,1.4,2.1)})
    sup=lambda xl,yl:bedroom_mattress_top(cx+xl,cy+yl)+.04-cz
    bedroom_sheet('Lofted duvet',[[(xl,yl,sup(xl,yl)+bedroom_duvet_t(xl,yl,o),sup(xl,yl)) for xl in xs] for yl in ys],'fabric')
    yy=yf+.3;tube([(xl,yy,sup(xl,yy)+bedroom_duvet_t(xl,yy,o)-.05) for xl in [-X+2.6+(2*X-5.2)*i/44 for i in range(45)]],.26,'fabric','#b7b9ac')

def bedroom_pillow(o,index,size):
    x,y,z=size;X,Y=x/2,y/2-.02;cx,cy,cz=o['center'];rnd=random.Random(index);n,m=36,24
    seam=-1.6;Ht=z/2-seam;Hb=6.5;dx,dy=rnd.uniform(-7,7),rnd.uniform(-2,5)
    rows=[];edge=[]
    for j in range(m+1):
        v=-math.cos(math.pi*j/m);row=[]
        for i in range(n+1):
            u=-math.cos(math.pi*i/n);px,py=u*(X-2.3*(1-v*v)),v*(Y-1.7*(1-u*u))   # stuffing draws the edges in
            fz=((1-u*u)*(1-v*v))**.62
            cr=.32*math.sin(.42*px+.31*py+dx)*math.sin(.37*py-.22*px+dy)*math.sqrt(fz)*(1-fz)*4   # fabric creases on the slopes
            top=seam+Ht*fz*(1-.04*math.exp(-((px-dx)**2+(py-dy)**2)/140))+cr;bot=seam-Hb*fz
            s=bedroom_mattress_top(cx+px,cy+py)+.1-cz;d=bedroom_duvet_top(cx+px,cy+py)
            if d is not None:s=max(s,d+.3-cz)
            if bot<s:bot=s;top=max(top,s+1.3*math.sqrt(fz))               # soft underside settles on what supports it
            row.append((px,py,top,bot))
        rows.append(row)
    bedroom_sheet('Filled pillow',rows,'fabric')
    ring=[rows[0][i] for i in range(n)]+[rows[j][n] for j in range(m)]+[rows[m][i] for i in range(n,0,-1)]+[rows[j][0] for j in range(m,0,-1)]
    tube([(p[0],p[1],p[2]) for p in ring[::2]],.3,'fabric',bedroom_shade(col,.9),True)   # piped seam

def bedroom_headboard(o,index,size):
    x,y,z=size;n=8;w=x/n
    box((0,y/2-1.4,0),(x,2.8,z),.5,'wood',bedroom_shade(col,.74))                # backing board
    for i in range(n):bedroom_blob((w,5.4,z-.4),(-x/2+w*(i+.5),-y/2+2.7,0),(.55,.12),(20,12),'fabric',col)   # channel tufting

# ---------------------------------------------------------------- floor runner
def bedroom_runner(o,index,size):
    x,y,z=size;b=5.5;g=.03;bev=min(.14,z*.2);edge=bedroom_shade(col,.88)
    box((0,0,0),(x-2*b-2*g,y-2*b-2*g,z),bev,'fabric')             # woven field
    for s in (-1,1):
        box((s*(x/2-b/2),0,0),(b,y,z),bev,'fabric',edge)          # bound border, exact rug thickness
        box((0,s*(y/2-b/2),0),(x-2*b-2*g,b,z),bev,'fabric',edge)
        for j in range(max(2,int(x/2))):tube([(-x/2+1+j*2,s*(y/2-.5),z*.1),(-x/2+1+j*2,s*(y/2+1),z*.03)],.055,'fabric',col)

# ---------------------------------------------------------------- nightstand and lamp
def bedroom_bedside_table(o,index,size):
    x,y,z=size;h=z/2;fy,by=-y/2+.4,y/2-.3;zb,zt=-h+3.8,h-2.45;cw=x/2-.4;dark=bedroom_shade(col,.55);hw='#8a8273'
    box((0,0,h-1.25),(x,y,2.5),.6)                                 # top, slight overhang all round
    for s in (-1,1):box((s*(cw-.8),(fy+by)/2,(zb+zt)/2+.03),(1.6,by-fy,zt-zb+.06),.25)   # sides
    iw=2*cw-3.2+.2
    box((0,by-.8,(zb+zt)/2),(iw,1.6,zt-zb),.2)                     # back
    box((0,(fy+by)/2,zb+.8),(iw,by-fy,1.6),.25)                     # bottom
    box((0,fy+5,5.6),(iw,10,1.8),.25)                               # rail between drawers
    box((0,(fy+by)/2+1.8,(zb+zt)/2),(iw-.4,by-fy-4,zt-zb-1),.1,None,dark)   # shadowed interior behind the gaps
    for z0,z1 in [(6.5,zt),(zb+1.6,4.7)]:                          # drawer fronts set 1 cm back, 3 mm gaps
        zc=(z0+z1)/2;box((0,fy+1.9,zc),(2*cw-3.2-.6,1.8,z1-z0-.6),.35)
        bedroom_rod((-6.6,fy-.48,zc),(6.6,fy-.48,zc),.36,'metal',hw,16,.6)
        for s in (-1,1):bedroom_rod((s*5.5,fy-.48,zc),(s*5.5,fy+1.05,zc),.3,'metal',hw,12)
    for sx in (-1,1):
        for sy in (-1,1):lathe([(0,-h),(1.1,-h),(1.2,-h+.12),(1.5,zb+.05),(0,zb+.05)],(1,1),'wood',col,16).data.transform(bedroom_Matrix.Translation((sx*(cw-2.6),sy*(y/2-3.4),0)))
    st=BEDROOM_OBJ['Bedside lamp stem']                           # weighted steel base plate of the lamp
    p=lathe([(0,h-.05),(6.2,h-.05),(6.42,h+.06),(6.48,h+.2),(6.35,h+.34),(6.05,h+.4),(0,h+.4)],(1,1),'metal',st['color'],40)
    p.data.transform(bedroom_Matrix.Translation((st['center'][0]-o['center'][0],st['center'][1]-o['center'][1],0)))

def bedroom_lamp_stem(o,index,size):
    x,y,z=size;h=z/2;R=x/2+.45
    lathe([(0,-h),(R-.1,-h),(R,-h+.1),(R,-h+2.5),(R-.12,-h+2.85),(R-.55,-h+3.2),(1.0,-h+3.55),(.72,-h+4.1),(.72,-4.2),(.92,-4.05),(.92,-2.75),
           (.72,-2.6),(.72,h-.95),(1.05,h-.8),(1.05,h-.06),(.96,h),(0,h)],(1,1),'metal',col,24)

def bedroom_lamp_shade(o,index,size):
    x,y,z=size;R=x/2-.25;H=z/2-.25;cz=o['center'][2];hw='#6f6553';st=BEDROOM_OBJ['Bedside lamp stem']
    s0=st['center'][2]+st['size'][2]/2-cz                           # stem top inside the shade
    lathe([(R-.15,-H),(R,-H),(R,H),(R-.15,H),(R-.15,-H)],(1,1),'fabric',col,64)   # drum wall
    for zz in (-H,H):tube([((R-.07)*math.cos(a),(R-.07)*math.sin(a),zz) for a in [2*math.pi*i/48 for i in range(48)]],.25,'fabric',bedroom_shade(col,.9),True)
    lathe([(0,s0-.2),(1.25,s0-.2),(1.3,s0-.1),(1.3,s0+3.2),(1.15,s0+3.4),(0,s0+3.4)],(1,1),'metal',hw,24)   # socket
    lathe([(0,s0+3.0),(1.1,s0+3.0),(1.15,s0+4.0),(1.6,s0+5.0),(2.6,s0+6.4),(2.95,s0+7.8),(2.8,s0+9.0),(2.1,s0+10.1),(1.1,s0+10.7),(0,s0+10.85)],(1,1),'plastic','#f3efe4',32)
    zt=s0+12.4;leg=[(3.6,s0+.4),(3.65,s0+6),(3.4,s0+8.6),(2.4,s0+10.9),(1.0,zt-.2)]   # harp loop with saddle
    tube([(a,0,b) for a,b in leg]+[(0,0,zt)]+[(-a,0,b) for a,b in leg[::-1]],.12,'metal',hw,True)
    for k in range(3):a=2*math.pi*k/3+.5;tube([(.45*math.cos(a),.45*math.sin(a),zt+.05),((R-.1)*math.cos(a),(R-.1)*math.sin(a),H-.1)],.09,'metal',hw)   # spider
    lathe([(0,zt-.05),(.75,zt-.05),(.8,zt+.1),(.55,zt+.3),(.35,zt+.5),(.6,zt+1.0),(.62,zt+1.3),(.4,zt+1.7),(0,zt+1.9)],(1,1),'metal',hw,16)   # finial

# ---------------------------------------------------------------- wardrobe
def bedroom_wardrobe(o,index,size):
    x,y,z=size;X,Y,H=x/2,y/2,z/2;t=1.8;zb=-H+8;zc=H-3.8;iy=2*(Y-t)+.1
    for s in (-1,1):box((0,s*(Y-t/2),(-H+zc)/2),(x,t,H+zc),.3)    # side panels down to the floor
    box((-X+1,0,(zb+zc)/2+.02),(2,iy,zc-zb+.04),.25)               # face frame behind the doors
    box((0,0,zb-.9),(x,iy,1.8),.25)                                # bottom
    box((X-.5,0,(-H+zc)/2),(1,iy,zc+H),.1)                         # back, down to the floor
    box((-X+3.9,0,(-H+zb-1.75)/2),(1.8,iy,zb-1.75+H),.2,None,bedroom_shade(col,.86))   # recessed plinth
    box((-.1,0,zc+.9),(x+.2,y+.4,1.8),.25)                       # cornice fascia
    box((-.225,0,zc+1.8+(H-zc-1.8)/2),(x+.45,y+.9,H-zc-1.8),.4)    # cornice cap

def bedroom_wardrobe_door(o,index,size):
    x,y,z=size;W,H=y/2,z/2;f,b=-x/2,x/2;st,rt,rb=14.5,14.5,16;th=b-f;pf=f+.4   # 1 cm slab keeps the robot-band outline
    for s in (-1,1):box(((f+b)/2,s*(W-st/2),0),(th,st,z),.3)       # stiles
    for a,c in [(-H,-H+rb),(H-rt,H)]:box(((f+b)/2,0,(a+c)/2),(th,y-2*st+.2,c-a),.3)   # rails
    box(((pf+b)/2,0,(rb-rt)/2),(b-pf,y-2*st+.2,z-rb-rt+.2),.12)   # recessed panel
    tube([(pf,-W+st,-H+rb),(pf,W-st,-H+rb),(pf,W-st,H-rt),(pf,-W+st,H-rt)],.17,None,None,True)   # cove bead

def bedroom_wardrobe_handle(o,index,size):
    x,y,z=size;d=min((p for p in scene['objects'] if p['name']=='Wardrobe door'),key=lambda p:abs(p['center'][1]-o['center'][1]))
    front=d['center'][0]-d['size'][0]/2-o['center'][0];bx=-.35
    bedroom_rod((bx,0,-z/2),(bx,0,z/2),.6,'metal',col,20,.5)       # long bar
    for s in (-1,1):bedroom_rod((bx,0,s*(z/2-3)),(front+.05,0,s*(z/2-3)),.42,'metal',col,16,.2)   # standoffs

# ---------------------------------------------------------------- open interior door leaves
def bedroom_stadium(cy,cz,r,a,k=12):
    # CCW (y,z) outline of a vertical stadium as (cos,sin,arc sign); point = centre+(0,sign*a)+rr*(cos,sin)
    return [(math.cos(t),math.sin(t),1) for t in [math.pi*i/k for i in range(k+1)]]+[(math.cos(t),math.sin(t),-1) for t in [math.pi*(1+i/k) for i in range(k+1)]]

def bedroom_flush_pull(stile,y0,z0,X0,hw,r=1.7,a=4.0):
    # Back-to-back recessed flush pulls in a through-pocket of the lock stile (a lever cannot fit the leaf envelope).
    ring=bedroom_stadium(y0,z0,r,a);n=len(ring)
    cv=[(h,y0+r*c,z0+g*a+r*s) for h in (-X0-1,X0+1) for c,s,g in ring]
    me=bpy.data.meshes.new('pocket cutter');me.from_pydata(cv,[],[tuple(range(n))[::-1],tuple(range(n,2*n))]+[(i,(i+1)%n,n+(i+1)%n,n+i) for i in range(n)])
    cut=bpy.data.objects.new('pocket cutter',me);bpy.context.collection.objects.link(cut)
    bpy.context.view_layer.objects.active=stile;bpy.ops.object.modifier_apply(modifier=stile.modifiers[0].name)   # round the stile first; pocket rim hides under the flange
    b=stile.modifiers.new('Flush pull pocket','BOOLEAN');b.operation='DIFFERENCE';b.solver='FAST';b.object=cut   # FAST: no T-vertex slivers
    bpy.ops.object.modifier_apply(modifier=b.name);bpy.data.objects.remove(cut,do_unlink=True)
    # profile (inset from pocket wall, x): cup floor -> cup wall -> lip -> flange (sunk into the face) -> sleeve
    p=[(.6,X0-1.2),(.38,X0-1.15),(.24,X0-1.0),(.2,X0-.75),(.2,X0-.1),(.17,X0+.06),(.08,X0+.14),(-.25,X0+.15),(-.38,X0+.11),(-.43,X0+.03),(-.43,X0-.06),(.03,X0-.2)]
    p+=[(d,-h) for d,h in p[::-1]];m=len(p)
    v=[(h,y0+(r-d)*c,z0+g*a+(r-d)*s) for d,h in p for c,s,g in ring]
    f=[(j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i) for j in range(m-1) for i in range(n)]+[tuple(range(n)),tuple(range((m-1)*n,m*n))]
    o=bedroom_close(mesh('Recessed flush pull',v,f,'metal',hw))
    for q in o.data.polygons:q.use_smooth=len(q.vertices)==4     # flat cup floors
    return o

def bedroom_room_door(o,index,size):
    x,y,z=size;W=y/2;zb,zt=-z/2+.45,z/2;st=11;hw='#a9a599';pt=x/2-.6
    rails=[(zb,zb+21.5),(-12.5,3.5),(zt-11,zt)]                    # bottom, lock and top rails
    stiles=[box((0,s*(W-st/2),(zb+zt)/2),(x,st,zt-zb),.35) for s in (-1,1)]   # stiles
    for a,c in rails:box((0,0,(a+c)/2),(x,y-2*st+.2,c-a),.35)
    for a,c in [(rails[0][1],rails[1][0]),(rails[1][1],rails[2][0])]:
        box((0,0,(a+c)/2),(2*pt,y-2*st+.2,c-a+.2),.15)             # recessed panels
        for s in (-1,1):tube([(s*pt,-W+st,a),(s*pt,W-st,a),(s*pt,W-st,c),(s*pt,-W+st,c)],.3,None,None,True)
    bedroom_flush_pull(stiles[1],W-5.8,-2.5,x/2,hw)                # pull at ~1 m, lock stile
    box((0,W+.04,-2.5),(2.2,.12,22),.03,'metal',hw)                # latch face plate
    hy=-W+3.75                                                     # hinge side sits in the wall return
    for zz in (zb+24.5,0,zt-18):
        bedroom_rod((x/2-.1,hy,zz-5),(x/2-.1,hy,zz+5),.55,'metal',hw,16,.5)
        box((x/2+.03,hy+1.2,zz),(.15,2.4,9.6),.04,'metal',hw)

GROUP_BUILDERS.update({'Bed frame':bedroom_bed_frame,'Bed frame leg':bedroom_bed_leg,'Mattress':bedroom_mattress,'Duvet':bedroom_duvet,
    'Pillow':bedroom_pillow,'Headboard':bedroom_headboard,'Bedroom runner':bedroom_runner,'Bedside table':bedroom_bedside_table,
    'Bedside lamp stem':bedroom_lamp_stem,'Bedside lamp shade':bedroom_lamp_shade,'Wardrobe':bedroom_wardrobe,'Wardrobe door':bedroom_wardrobe_door,
    'Wardrobe handle':bedroom_wardrobe_handle,'Bedroom door open':bedroom_room_door,'Study door open':bedroom_room_door})

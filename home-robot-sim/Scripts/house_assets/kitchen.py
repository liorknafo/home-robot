# Kitchen group: fitted shaker kitchen on a recessed plinth, built-under oven, inset sink,
# induction hob and fridge-freezer. Object-local cm; every unit here faces +y (into the room).
import bmesh

def kitchen_hex(c,f):
    # Derived shade of a hex colour (f<1 darker, f>1 lighter).
    return '#'+''.join(f'{max(0,min(255,round(int(c[i:i+2],16)*f))):02x}' for i in (1,3,5))

def kitchen_rrect(hx,hy,r,n=6,cx=0,cy=0):
    # Rounded-rectangle outline, counterclockwise, n segments per corner.
    r=max(.02,min(r,hx-.01,hy-.01));p=[]
    for qx,qy,a in [(1,1,0),(-1,1,90),(-1,-1,180),(1,-1,270)]:
        for i in range(n+1):
            t=math.radians(a+90*i/n);p.append((cx+qx*(hx-r)+r*math.cos(t),cy+qy*(hy-r)+r*math.sin(t)))
    return p

def kitchen_fix(o,angle=35):
    # Outward normals and crisp creases on generated (closed) meshes.
    bm=bmesh.new();bm.from_mesh(o.data);bmesh.ops.recalc_face_normals(bm,faces=bm.faces);bm.to_mesh(o.data);bm.free()
    o.data.set_sharp_from_angle(angle=math.radians(angle));return o

def kitchen_loft(loops,kind=None,color=None,caps=(False,False),closed=False,angle=35):
    # Quads between equal-length cyclic loops, optional flat end caps.
    n=len(loops[0]);L=len(loops);v=[tuple(p) for l in loops for p in l]
    f=[(j*n+i,j*n+(i+1)%n,(j+1)%L*n+(i+1)%n,(j+1)%L*n+i) for j in range(L if closed else L-1) for i in range(n)]
    if caps[0]:f.append(tuple(range(n-1,-1,-1)))
    if caps[1]:f.append(tuple(range((L-1)*n,L*n)))
    return kitchen_fix(mesh('Kitchen lofted part',v,f,kind,color),angle)

def kitchen_prism(outline,a0,a1,axis=2,kind=None,color=None,cut=False):
    # Outline extruded along an axis; cut=True returns a bare boolean cutter (never exported).
    P=(lambda u,w,a:(a,u,w)) if axis==0 else (lambda u,w,a:(u,a,w)) if axis==1 else (lambda u,w,a:(u,w,a))
    loops=[[P(u,w,a) for u,w in outline] for a in (a0,a1)]
    if not cut:return kitchen_loft(loops,kind,color,(True,True))
    n=len(outline);me=bpy.data.meshes.new('Kitchen cutter')
    me.from_pydata([p for l in loops for p in l],[],[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]);me.update()
    o=bpy.data.objects.new('Kitchen cutter',me);bpy.context.collection.objects.link(o);return kitchen_fix(o)

def kitchen_round(o,w,seg=3):
    # Same rounded-edge finish as box(): angle-limited bevel plus weighted normals.
    b=o.modifiers.new('Rounded manufactured edges','BEVEL');b.width=w;b.segments=seg;b.harden_normals=True
    n=o.modifiers.new('Face normals','WEIGHTED_NORMAL');n.keep_sharp=True;n.weight=40;return o

def kitchen_block(pos,size,bevel,cuts=(),kind=None,color=None):
    # Box with boolean cut-outs applied first, then the rounded-edge finish.
    bpy.ops.mesh.primitive_cube_add(size=1,location=pos);o=bpy.context.object;o.scale=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    for c in cuts:
        b=o.modifiers.new('Kitchen cut','BOOLEAN');b.operation='DIFFERENCE';b.object=c
        bpy.context.view_layer.objects.active=o;bpy.ops.object.modifier_apply(modifier=b.name);bpy.data.objects.remove(c,do_unlink=True)
    if bevel:kitchen_round(o,bevel)
    return keep(o,kind,color,True)

def kitchen_sweep(path,radius,kind=None,color=None,n=14):
    # Capped round tube along a polyline (parallel-transport frames), radius per point or constant.
    P=[Vector(p) for p in path];R=list(radius) if isinstance(radius,(list,tuple)) else [radius]*len(P)
    T=[(P[min(i+1,len(P)-1)]-P[max(i-1,0)]).normalized() for i in range(len(P))]
    N=T[0].cross(Vector((0,0,1)) if abs(T[0].z)<.9 else Vector((1,0,0))).normalized();loops=[]
    for i,p in enumerate(P):
        N=(N-T[i]*N.dot(T[i])).normalized();B=T[i].cross(N)
        loops.append([p+R[i]*(math.cos(2*math.pi*k/n)*N+math.sin(2*math.pi*k/n)*B) for k in range(n)])
    return kitchen_loft(loops,kind,color,(True,True),angle=55)

def kitchen_fillet(pts,r,n=6):
    # Replace interior polyline corners with smooth arcs.
    P=[Vector(p) for p in pts];out=[P[0]]
    for i in range(1,len(P)-1):
        a=P[i-1]-P[i];b=P[i+1]-P[i];d=min(r,a.length*.5,b.length*.5);p0=P[i]+a.normalized()*d;p1=P[i]+b.normalized()*d
        out+=[(1-t)**2*p0+2*(1-t)*t*P[i]+t*t*p1 for t in [k/n for k in range(n+1)]]
    return out+[P[-1]]

def kitchen_dome(e,R,side=1,steps=(0,.03,.1,.22,.4,.62,1)):
    # Axial stations and radii for a rounded bar end of radius R at e (side=-1 for the left end).
    return [(e-side*R*s,max(.08,R*math.sqrt(max(0,1-(1-s)**2)))) for s in steps]

def kitchen_annulus(r0,r1,z0,z1,n=64,kind=None,color=None,cx=0,cy=0):
    # Closed flat ring (printed marking or rosette).
    C=lambda r,z:[(cx+r*math.cos(2*math.pi*k/n),cy+r*math.sin(2*math.pi*k/n),z) for k in range(n)]
    return kitchen_loft([C(r1,z0),C(r1,z1),C(r0,z1),C(r0,z0)],kind,color,closed=True,angle=50)

def kitchen_lathe(profile,pos=(0,0,0),kind=None,color=None,segments=32):
    # lathe() with crisp profile corners, moved to pos; profile runs up the outside.
    o=lathe(profile,(1,1),kind,color,segments);o.location=pos;o.data.set_sharp_from_angle(angle=math.radians(40));return o

def kitchen_shaker(w,h,t,stile,rail,panel_y,cz=0,kind=None,color=None):
    # Five-piece shaker front: full-height stiles, rails between, recessed flat centre panel.
    for s in (-1,1):box((s*(w/2-stile/2),0,cz),(stile,t,h),.3,kind,color)
    for s in (-1,1):box((0,0,cz+s*(h/2-rail/2)),(w-2*stile,t,rail),.3,kind,color)
    box((0,(panel_y-t/2+.15)/2,cz),(w-2*stile+.5,panel_y+t/2-.15,h-2*rail+.5),.12,kind,color)

def kitchen_cabinet(o,index,size):
    x,y,z=size;cx,cy=o['center'][:2]
    # Carcass stands on the plinth (world 10 cm) and meets the worktop underside; fronts sit 0.5 mm proud.
    # A sink base has no top panel under the bowl: open pocket for the hanging basin.
    cuts=[kitchen_prism(kitchen_rrect(19.6,14.6,4,6,s['center'][0]-cx,s['center'][1]-cy),z/2-7.5,z/2+5,2,cut=True)
          for s in scene['objects'] if s['name']=='Inset sink' and abs(s['center'][0]-cx)<x/2 and abs(s['center'][1]-cy)<y/2]
    kitchen_block((0,.2,(-z/2+6+z/2+.47)/2),(x+.9,y+.4,z-6+.47),.3,cuts)
    box((0,-2,-z/2+3.025),(x-4,y-10,6.05),0)  # levelling-leg rail, enclosed by the plinth block

def kitchen_plinth(o,index,size):
    x,y,z=size
    box((0,-.35,-.02),(x+.7,y-.7,z-.04),.15)                       # plinth core
    box((0,y/2-.9,.25),(x+.86,1.8,z-.5),.25)                        # recessed kick board, eased top edge
    box((0,y/2-.7,-z/2+.4),(x+.76,1.6,.8),.3,'rubber',kitchen_hex(col,.55))  # floor seal strip

def kitchen_drawer(o,index,size):
    x,y,z=size;kitchen_shaker(x,z,y,6,3.5,.12)

def kitchen_door(o,index,size):
    x,y,z=size
    oven=[s for s in scene['objects'] if s['name']=='Oven fascia' and abs(s['center'][0]-o['center'][0])<x/2 and abs(s['center'][1]-o['center'][1])<5]
    if not oven:kitchen_shaker(x,z,y,6.5,6.5,.12);return
    # Built-under oven housing: dark oven chassis behind the oven door, shallow drawer below it.
    f=oven[0];fb=f['center'][2]-f['size'][2]/2-o['center'][2]
    box((0,-.125,(fb+z/2)/2),(x,1.25,z/2-fb),.2,'metal',kitchen_hex(f['color'],.72))
    h=fb-.4+z/2;kitchen_shaker(x,h,y,6,2.8,.12,-z/2+h/2)

def kitchen_pull(o,index,size):
    x,y,z=size;r=.6;yc=y/2-r
    # Round bar with domed ends on two standoff posts (160 mm centres) with rosettes.
    st=kitchen_dome(-x/2,r,-1)+kitchen_dome(x/2,r)[::-1]
    kitchen_sweep([(a,yc,0) for a,_ in st],[q for _,q in st],n=16)
    for s in (-1,1):kitchen_sweep([(s*8,-y/2-.47,0),(s*8,-y/2-.3,0),(s*8,-y/2-.22,0),(s*8,yc,0)],[.62,.62,.4,.4],n=12)

def kitchen_worktop(o,index,size):
    x,y,z=size;lo=-z/2-.47
    # Quartz-look slab resting on the carcasses, eased edges, radiused cut-out under the sink rim.
    cuts=[kitchen_prism(kitchen_rrect(20,15,4,6,s['center'][0]-o['center'][0],s['center'][1]-o['center'][1]),-9,9,2,cut=True)
          for s in scene['objects'] if s['name']=='Inset sink' and abs(s['center'][0]-o['center'][0])<x/2]
    kitchen_block((0,0,(lo+z/2)/2),(x,y,z/2-lo),.45,cuts,'ceramic',col)

def kitchen_tile(o,index,size):
    x,y,z=size;f=1+random.Random(index).uniform(-.025,.02)
    box((0,0,0),(x,y,z),.22,'ceramic',kitchen_hex(col,f))            # glazed tile, cushion edges
    box((0,-.33,0),(x+.3,.74,z+.3),0,'paint',kitchen_hex(col,.88))   # grout bed showing in the joints

def kitchen_sink_rim(o,index,size):
    x,y,z=size;hx,hy=18.6,13.6;sy=(y/2-hy)/(x/2-hx)
    # Pressed stainless rim: rolled inner lip over the bowl flange, flat deck, rolled edge onto the worktop,
    # short skirt hidden inside the cut-out. Profile (outward offset, z).
    prof=[(0,.72),(.07,.85),(.28,.95),(3.2,.95),(3.85,.83),(4.25,.5),(4.4,.08),(4.4,-.08),(1.2,-.08),(1.2,-.6),(1,-.6),(1,-.08),(.4,.73),(.05,.73)]
    kitchen_loft([[(a,b,zz) for a,b in kitchen_rrect(hx+d,hy+d*sy,3+d*.5,6)] for d,zz in prof],'metal',col,closed=True,angle=40)

def kitchen_basin(o,index,size):
    # Stainless bowl: flange tucked under the rim lip, drafted walls, coved floor, strainer waste.
    In=[(16.9,11.9,1.3,-11.35),(17.55,12.55,1.95,-10.95),(17.9,12.9,2.3,-9.8),(18.25,13.25,2.65,-.35),(18.33,13.33,2.73,-.13),(18.5,13.5,2.9,-.03),(18.85,13.85,3.25,-.03)]
    Out=[(18.85,13.85,3.25,-.15),(18.45,13.45,2.85,-.15),(18.37,13.37,2.77,-.4),(18.02,13.02,2.42,-9.85),(17.65,12.65,2.05,-11.05),(16.9,11.9,1.3,-11.47)]
    kitchen_loft([[(a,b,zz) for a,b in kitchen_rrect(hx,hy,r,6)] for hx,hy,r,zz in In+Out],'metal',col,(True,True),angle=30)
    zb=-11.35
    kitchen_lathe([(3.8,zb-.02),(3.75,zb+.03),(3.45,zb+.12),(2.55,zb+.12),(2.4,zb+.07),(2.4,zb-.02)],(0,0,0),'metal',kitchen_hex(col,1.1),40)
    kitchen_lathe([(2.5,zb-.02),(2.5,zb+.06),(0,zb+.06)],(0,0,0),'metal',kitchen_hex(col,.55),40)

def kitchen_tap_body(o,index,size):
    x,y,z=size;h=z/2
    # Deck escutcheon, slim body with a fluted control collar, domed head carrying the spout.
    flutes=[(1.47 if k%2==0 else 1.42,6.15+k*.2) for k in range(12)]
    prof=[(1.62,-h+.05),(1.7,-h+.6),(1.7,-11.4),(1.6,-11.12),(1.42,-11.0),(1.3,-10.95),(1.3,5.8),(1.44,5.88)]+flutes+[(1.44,8.55),(1.3,8.62),(1.3,12.2),(1.22,12.65),(1.0,12.9),(.6,13.0),(0,13.0)]
    kitchen_lathe(prof,(0,0,0),'metal',col,32)

def kitchen_spout(o,index,size):
    x,y,z=size
    # Swept spout leaving the tap head, bending down to a threaded aerator.
    arc=[(0,5.6+1.35*math.sin(t),-1.3+1.35*math.cos(t)) for t in [math.pi/2*k/7 for k in range(1,8)]]
    path=[(0,-7.1,-.1),(0,-6.4,-.1),(0,-4,-.04),(0,2,.02),(0,5.6,.05)]+arc+[(0,6.95,-1.5)]
    kitchen_sweep(path,[.4,.8]+[.78]*(len(path)-2),n=16)
    kitchen_lathe([(.62,-2.2),(.92,-2.2),(1.0,-2.12),(1.0,-1.3),(.85,-1.15),(.7,-1.15),(.62,-1.3),(.62,-2.2)],(0,6.95,0),'metal',col,24)
    kitchen_lathe([(0,-2.06),(.66,-2.06),(.66,-2.0),(0,-2.0)],(0,6.95,0),'metal',kitchen_hex(col,.5),20)  # mesh insert

def kitchen_hob(o,index,size):
    x,y,z=size
    # Frameless glass-ceramic plate with ground bevel; low-contrast printed touch controls at the front.
    kitchen_round(kitchen_prism(kitchen_rrect(x/2,y/2,.9,4),-z/2,z/2,2,'ceramic',col),.22)
    lc=kitchen_hex(col,1.7)
    kitchen_prism(kitchen_rrect(4.6,.28,.26,4,0,y/2-1.9),z/2-.02,z/2+.06,2,'ceramic',lc)
    for cx in (-6.4,6.4):kitchen_annulus(.5,.66,z/2-.02,z/2+.06,24,'ceramic',lc,cx,y/2-1.9)

def kitchen_hob_ring(o,index,size):
    x,y,z=size;r=x*.465
    # Cooking-zone marking printed on the glass (glass top is the ring's local bottom).
    kitchen_annulus(r-.16,r,-z/2-.02,-z/2+.06,64,'ceramic',col)
    kitchen_annulus(r*.55-.09,r*.55,-z/2-.02,-z/2+.06,40,'ceramic',col)

def kitchen_oven(o,index,size):
    x,y,z=size
    # Oven door: anthracite steel frame with a radiused window, recessed dark glass.
    win=kitchen_prism(kitchen_rrect(20.5,15.5,2.2,6,0,-2),-2,2,1,cut=True)
    kitchen_block((0,0,0),(x,y,z),.3,[win],'metal',col)
    box((0,-.1,-2),(42,.5,32),.12,'mirror',kitchen_hex(col,.45))

def kitchen_oven_handle(o,index,size):
    x,y,z=size;r=.9;yc=y/2-r-.05
    st=kitchen_dome(-x/2,r,-1)+kitchen_dome(x/2,r)[::-1]
    kitchen_sweep([(a,yc,0) for a,_ in st],[q for _,q in st],n=18)
    for s in (-1,1):box((s*(x/2-2.6),(yc-y/2-.45)/2,0),(1.5,yc+y/2+.45,1.5),.4)  # brackets into the door

def kitchen_fridge(o,index,size):
    x,y,z=size;h=z/2;fb,ff=27.0,y/2+.45
    # Cabinet with radiused vertical corners standing on levelling feet.
    kitchen_round(kitchen_prism(kitchen_rrect(x/2-.05,(fb+y/2-.05)/2,1.6,4,0,(fb-y/2+.05)/2),-h+1.5,h,2),.6)
    kitchen_prism(kitchen_rrect(x/2-.6,.35,.3,2,0,fb+.15),-h+5.6,h-.5,2,'rubber','#2c302d')  # door gasket line
    # Freezer and fridge doors, split at the mullion seam, with rounded edges.
    box((0,(fb+.3+ff)/2,(-h+5.5-h+63.6)/2),(x-.1,ff-fb-.3,58.1),.9)
    box((0,(fb+.3+ff)/2,(-h+64.4+h-.4)/2),(x-.1,ff-fb-.3,119.2),.9)
    box((x/2-3.2,30.1,h-.2),(6.2,5,.38),.12,'plastic',kitchen_hex(col,.93))  # top hinge cover
    # Recessed kick grille with horizontal slats under the freezer door.
    box((0,28.6,-h+3.45),(x-4,3.2,3.7),.1,'plastic','#202422')
    for k in range(4):box((0,30.55,-h+2.05+k*.95),(x-4.6,.8,.45),.15,'plastic',kitchen_hex(col,.32))
    for sx in (-1,1):
        for fy in (24,-28):kitchen_lathe([(0,-h),(1.25,-h),(1.25,-h+1.6),(0,-h+1.6)],(sx*26,fy,0),'rubber','#2d302e',20)

def kitchen_fridge_seam(o,index,size):
    x,y,z=size
    box((0,-.485,0),(x+.9,.53,1.6),.2,'plastic',col)  # mullion strip filling the split between the rounded door edges

def kitchen_fridge_handle(o,index,size):
    x,y,z=size;r=.92;yb=-y/2-.49;yf=y/2-r-.03;zz=z/2-r-.03
    # D handle: posts and bar as one bent tube, rosettes at the door.
    kitchen_sweep(kitchen_fillet([(0,yb+.3,-zz),(0,yf,-zz),(0,yf,zz),(0,yb+.3,zz)],1.8,7),r,n=14)
    for s in (-1,1):kitchen_sweep([(0,yb,s*zz),(0,yb+.3,s*zz),(0,yb+.42,s*zz)],[1.22,1.22,.92],n=16)

GROUP_BUILDERS.update({'Kitchen base cabinet':kitchen_cabinet,'Cabinet plinth':kitchen_plinth,'Kitchen drawer front':kitchen_drawer,
    'Kitchen door front':kitchen_door,'Drawer pull':kitchen_pull,'Kitchen countertop':kitchen_worktop,'Splashback tile':kitchen_tile,
    'Inset sink':kitchen_sink_rim,'Sink basin':kitchen_basin,'Tap upright':kitchen_tap_body,'Tap spout':kitchen_spout,
    'Induction hob':kitchen_hob,'Hob ring':kitchen_hob_ring,'Oven fascia':kitchen_oven,'Oven handle':kitchen_oven_handle,
    'Tall refrigerator':kitchen_fridge,'Fridge seam':kitchen_fridge_seam,'Fridge handle':kitchen_fridge_handle})

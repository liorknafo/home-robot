"""Author every house object in Blender and export shared render/collision meshes.
Run: Blender --background --python Scripts/build_house_assets_blender.py
Room-specific builders live in Scripts/house_assets/*.py; each registers
scene-object names in GROUP_BUILDERS and keeps that object's footprint.
Input layout is deterministic; no robot results, maps or camera tracks are read.
Dimensions and output positions use centimetres in the existing house convention.
"""
import bpy,math,json,random,os
from pathlib import Path
from mathutils import Vector
R=Path(os.environ.get('HOUSE_ROOT',Path(__file__).resolve().parents[1]));rng=random.Random(27)
# Optional development overrides: write outputs elsewhere / build named assets only.
OUT=Path(os.environ.get('HOUSE_ASSET_OUT',R));ONLY=set(filter(None,os.environ.get('HOUSE_ASSET_ONLY','').split('|')))
for d in ['Content/RobotData','Blender','reports']:(OUT/d).mkdir(parents=True,exist_ok=True)
scene=json.loads((R/'Content/RobotData/house-scene.json').read_text())
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
materials={};result={};all_meshes=[];current=[];mat='paint';col='#cccccc'

def material(kind,color):
    key=kind+color
    if key in materials:return materials[key]
    m=bpy.data.materials.new(key);m.diffuse_color=tuple(int(color[i:i+2],16)/255 for i in [1,3,5])+(1,);m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=m.diffuse_color
    bs.inputs['Roughness'].default_value={'metal':.27,'ceramic':.23,'mirror':.03,'rubber':.8}.get(kind,.65)
    bs.inputs['Metallic'].default_value=1 if kind in ['metal','mirror'] else 0
    texture=R/'assets/house'/f'{kind}.png'
    if kind=='terrazzo':texture=R/'assets/terrazzo-floor.png'
    if texture.exists():
        n=m.node_tree.nodes.new('ShaderNodeTexImage');n.image=bpy.data.images.load(str(texture),check_existing=True)
        mix=m.node_tree.nodes.new('ShaderNodeMixRGB');mix.blend_type='MULTIPLY';mix.inputs[0].default_value=1;mix.inputs[1].default_value=m.diffuse_color
        m.node_tree.links.new(n.outputs['Color'],mix.inputs[2]);m.node_tree.links.new(mix.outputs[0],bs.inputs['Base Color'])
    materials[key]=m;return m

def keep(o,kind=None,color=None,smooth=False):
    o.data.materials.clear();o.data.materials.append(material(kind or mat,color or col));o['surface']=kind or mat;o['color']=color or col
    if smooth:
        for p in o.data.polygons:p.use_smooth=True
    current.append(o);return o

def box(pos,size,bevel=.3,kind=None,color=None):
    bpy.ops.mesh.primitive_cube_add(size=1,location=pos);o=bpy.context.object;o.scale=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    if bevel:
        b=o.modifiers.new('Rounded manufactured edges','BEVEL');b.width=min(bevel,min(size)*.42);b.segments=3
        b.affect='EDGES';b.harden_normals=True
        n=o.modifiers.new('Face normals','WEIGHTED_NORMAL');n.keep_sharp=True;n.weight=40
    return keep(o,kind,color,True)

def mesh(name,verts,faces,kind=None,color=None,smooth=True):
    me=bpy.data.meshes.new(name);me.from_pydata(verts,[],faces);me.update();o=bpy.data.objects.new(name,me);bpy.context.collection.objects.link(o)
    return keep(o,kind,color,smooth)

def tube(points,radius=.15,kind=None,color=None,cyclic=False):
    curve=bpy.data.curves.new('Curved detail','CURVE');curve.dimensions='3D';curve.resolution_u=6;curve.bevel_depth=radius;curve.bevel_resolution=2
    s=curve.splines.new('POLY');s.points.add(len(points)-1)
    for p,v in zip(s.points,points):p.co=(*v,1)
    s.use_cyclic_u=cyclic;o=bpy.data.objects.new('Curved detail',curve);bpy.context.collection.objects.link(o)
    bpy.context.view_layer.objects.active=o;o.select_set(True);bpy.ops.object.convert(target='MESH');o=bpy.context.object;o.select_set(False)
    return keep(o,kind,color,True)

def lathe(profile,xy=(1,1),kind=None,color=None,segments=48):
    v=[(r*math.cos(a*2*math.pi/segments)*xy[0],r*math.sin(a*2*math.pi/segments)*xy[1],z) for r,z in profile for a in range(segments)]
    f=[(j*segments+i,j*segments+(i+1)%segments,(j+1)*segments+(i+1)%segments,(j+1)*segments+i) for j in range(len(profile)-1) for i in range(segments)]
    return mesh('Turned hollow surface',v,f,kind,color)

def cushion(size,pos=(0,0,0),kind=None,color=None):
    sx,sy,sz=size;v=[];f=[];nu,nv=40,20
    def sq(x,p):return math.copysign(abs(x)**p,x)
    for j in range(nv+1):
        phi=-math.pi/2+math.pi*j/nv
        for i in range(nu):
            theta=2*math.pi*i/nu;c=abs(math.cos(phi))**.35
            v.append((pos[0]+sx/2*c*sq(math.cos(theta),.35),pos[1]+sy/2*c*sq(math.sin(theta),.35),pos[2]+sz/2*sq(math.sin(phi),.72)))
    for j in range(nv):
        for i in range(nu):f.append((j*nu+i,j*nu+(i+1)%nu,(j+1)*nu+(i+1)%nu,(j+1)*nu+i))
    o=mesh('Soft sewn cushion',v,f,kind,color)
    points=[(pos[0]+sx/2*.997*sq(math.cos(t),.35),pos[1]+sy/2*.997*sq(math.sin(t),.35),pos[2]) for t in [2*math.pi*i/100 for i in range(100)]]
    tube(points,.085,kind or 'fabric',color or col,True);return o

def bowl(size,kind='ceramic',color=None):
    x,y,z=size
    lathe([(0,-z/2),(.60,-z/2),(.82,-z*.36),(1,z*.30),(1,z/2),(.94,z/2),(.90,z*.24),(.69,-z*.27),(0,-z*.30)],(x/2,y/2),kind,color)

def ring(rx,ry,z,rad,kind=None,color=None):
    return tube([(rx*math.cos(t),ry*math.sin(t),z) for t in [2*math.pi*i/64 for i in range(64)]],rad,kind,color,True)

def leaf(size):
    x,y,z=size;v=[];f=[]
    for i in range(17):
        t=i/16;wide=math.sin(math.pi*t)**.8*y/2
        for j in range(5):
            s=j/2-1;v.append(((t-.5)*x,s*wide,(math.sin(t*math.pi)*.35+.3*s*s*math.sin(t*math.pi))*z-z*.25))
    for i in range(16):
        for j in range(4):a=i*5+j;f.append((a,a+5,a+6,a+1))
    o=mesh('Curled leaf blade',v,f,'leaf');solid=o.modifiers.new('Leaf thickness','SOLIDIFY');solid.thickness=.05
    tube([(-x*.55,0,-z*.3),(0,0,z*.1),(x*.48,0,-z*.25)],.07,'leaf','#8b9963')

def slipper(size):
    x,y,z=size;segments=64;outline=[]
    for i in range(segments):
        a=2*math.pi*i/segments;xx=math.cos(a)*x/2;yy=math.sin(a)*y/2*(.84+.13*math.cos(a))
        outline.append((xx,yy))
    v=[(xx,yy,h) for h in [-z/2,-z/2+.9] for xx,yy in outline]
    f=[tuple(range(segments-1,-1,-1)),tuple(range(segments,2*segments))]
    for i in range(segments):j=(i+1)%segments;f.append((i,j,j+segments,i+segments))
    sole=mesh('Shaped rubber outsole',v,f,'rubber','#554b43');b=sole.modifiers.new('Soft sole edge','BEVEL');b.width=.18;b.segments=3
    v=[(0,0,-z/2+1.0)]+[(a*.94,b*.94,-z/2+1.05) for a,b in outline]
    mesh('Footbed',v,[(0,i+1,(i+1)%segments+1) for i in range(segments)],'fabric','#bfad98')
    vv=[];ff=[];n,m=22,24
    for i in range(n+1):
        t=i/n;xx=-x*.10+t*x*.60;factor=math.sqrt(max(.0001,1-(xx/(x*.51))**2));height=(z-1.15)*(1-.15*t)*factor
        for j in range(m+1):
            a=math.pi*j/m;vv.append((xx,math.cos(a)*y*.49*factor,-z/2+.85+math.sin(a)*height))
    for i in range(n):
        for j in range(m):a=i*(m+1)+j;ff.append((a,a+m+1,a+m+2,a+1))
    upper=mesh('Open-backed fabric upper',vv,ff,'fabric',col);s=upper.modifiers.new('Fabric thickness','SOLIDIFY');s.thickness=.22
    tube(vv[:m+1],.17,'fabric','#cfbfa8')
    ringpoints=[(a,b,-z/2+.58) for a,b in outline];tube(ringpoints,.08,'rubber','#d2c2a9',True)

def curtain(size):
    x,y,z=size;v=[];f=[];n=60;k=12
    for j in range(k+1):
        h=-z/2+z*j/k
        for i in range(n+1):
            t=i/n;v.append(((t-.5)*x,math.sin(t*math.pi*14)*y*.43,h+.35*math.cos(t*math.pi*14)*(1-j/k)))
    for j in range(k):
        for i in range(n):a=j*(n+1)+i;f.append((a,a+1,a+n+2,a+n+1))
    o=mesh('Hanging folded fabric',v,f,'fabric');s=o.modifiers.new('Cloth thickness','SOLIDIFY');s.thickness=.12

def cloth(size):
    x,y,z=size;n,m=42,38;v=[];f=[]
    for j in range(m+1):
        b=j/m
        for i in range(n+1):
            a=i/n;edge=min(a,1-a,b,1-b);fold=.12*math.sin(a*17+b*8)+.16*math.cos(b*23+a*4)
            v.append(((a-.5)*x,(b-.5)*y,z*.22+min(1,edge*10)*z*.18*fold))
    for j in range(m):
        for i in range(n):a=j*(n+1)+i;f.append((a,a+1,a+n+2,a+n+1))
    o=mesh('Draped woven surface',v,f,'fabric');s=o.modifiers.new('Hem and thickness','SOLIDIFY');s.thickness=z*.65;s.offset=-1

def book(size):
    x,y,z=size
    # Smallest dimension is book thickness; inset paper block and two covers.
    axis=min(range(3),key=lambda i:size[i]);t=size[axis];paper=[v*.94 for v in size];paper[axis]=max(.15,t-.3)
    box((0,0,0),paper,.06,'paper','#e3ddcd')
    for sign in [-1,1]:
        s=list(size);s[axis]=min(.14,t*.15);p=[0,0,0];p[axis]=sign*(t/2-s[axis]/2);box(p,s,.05,'book',col)
    s=list(size);other=(axis+1)%3;s[other]=min(.28,size[other]*.08);p=[0,0,0];p[other]=-size[other]/2+s[other]/2;box(p,s,.08,'book',col)

def material_kind(o):
    n=o['name'].lower()
    if o.get('material')=='terrazzo':return 'terrazzo'
    if o.get('material')=='white_paint':return 'paint'
    if any(s in n for s in ['cushion','pillow','duvet','mattress','sofa back','sofa arm','upholstered','rug','runner','mat','blanket','curtain','blind','chair seat','chair back']):return 'fabric'
    # Painted joinery, trim and appliance fronts are not raw timber or bare metal.
    if 'handle' not in n and any(s in n for s in ['window','door casing','door open','kitchen base cabinet','kitchen door','kitchen drawer','cabinet plinth','wardrobe','vanity mirror frame']):return 'paint'
    if any(s in n for s in ['television','monitor','oven fascia']):return 'plastic'
    if any(s in n for s in ['handle','pull','tap','rail','upright','stem','hob ring']):return 'metal' if 'plant' not in n else 'wood'
    if any(s in n for s in ['toilet','basin','sink','mug','bowl','shower tray']):return 'ceramic'
    if 'leaf' in n:return 'leaf'
    if any(s in n for s in ['screen','hob','keyboard']):return 'plastic'
    if 'mirror' in n:return 'mirror'
    if any(s in n for s in ['book','magazine']):return 'book'
    if 'box' in n or 'tape' in n:return 'cardboard'
    if 'cable' in n:return 'rubber'
    if 'pot' in n:return 'ceramic'
    if 'soil' in n:return 'soil'
    if any(s in n for s in ['table','desk','leg','wardrobe','console','cabinet','drawer','door','frame','headboard','threshold','toy','bed frame']):return 'wood'
    return 'paint'

# Room builder files register name -> builder(o,index,size) in GROUP_BUILDERS.
# They share this script's helpers (box, mesh, tube, lathe, cushion, keep, ...).
GROUP_BUILDERS={}
for f in sorted(Path(os.environ.get('HOUSE_ASSET_GROUPS',R/'Scripts/house_assets')).glob('*.py')):exec(f.read_text(),globals())

# Every original object receives an authored mesh. Decorative subparts remain
# separate so their inherited collision and surface-response flags stay explicit.
for index,o in enumerate(scene['objects']):
    current=[];name=o['name'];x,y,z=o['size'];col=o['color'];mat=material_kind(o);key=f'asset_{index:03}'
    if ONLY and name not in ONLY:continue
    if name in GROUP_BUILDERS:GROUP_BUILDERS[name](o,index,(x,y,z))
    elif name=='Slipper':slipper((x,y,z))
    elif name=='Plant leaf':leaf((x,y,z))
    elif name=='Terracotta pot':
        lathe([(0,-z/2),(.70,-z/2),(.76,-z*.45),(1,z*.42),(1,z/2),(.87,z/2),(.85,z*.36),(.62,-z*.38),(0,-z*.38)],(x/2,y/2),'ceramic',col)
        ring(x*.485,y*.485,z*.39,.50,'ceramic',col)
    elif name=='Kitchen countertop':
        top=box((0,0,0),(x,y,z),.55,'ceramic')
        bpy.ops.mesh.primitive_cube_add(size=1,location=(-29,1,0));cut=bpy.context.object;cut.scale=(40,30,20);bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
        b=cut.modifiers.new('Sink cutout radius','BEVEL');b.width=4;b.segments=6;bpy.context.view_layer.objects.active=cut;bpy.ops.object.modifier_apply(modifier=b.name)
        b=top.modifiers.new('Open sink cutout','BOOLEAN');b.operation='DIFFERENCE';b.object=cut
        bpy.context.view_layer.objects.active=top;bpy.ops.object.modifier_apply(modifier=b.name);bpy.data.objects.remove(cut,do_unlink=True)
    elif name=='Inset sink':
        # Four rounded rim strips leave a genuinely open centre.
        for sign in [-1,1]:
            box((sign*(x/2-1.6),0,0),(3.2,y,z),.4,'metal')
            box((0,sign*(y/2-1.6),0),(x-4,3.2,z),.4,'metal')
    elif name=='Sink basin':
        bowl((37,27,12),'metal','#9ba5a4')
        for part in current:part.location.z-=5.6
    elif name in ['Coffee mug','Fruit bowl','Vanity basin','Toilet bowl']:
        bowl((x,y,z))
        if name=='Coffee mug':
            tube([(x*.48+2.4*math.sin(a),0,1.5+2.8*math.cos(a)) for a in [math.pi*i/24 for i in range(25)]],.55,'ceramic',col)
            lathe([(0,z*.22),(.81,z*.22)],(x/2,y/2),'plastic','#493224')
        if name=='Toilet bowl':ring(x*.48,y*.48,z*.48,.65,'ceramic','#faf6eb')
    elif name=='Toilet base':lathe([(0,-z/2),(.63,-z/2),(.72,-z*.45),(.54,-z*.27),(.51,0),(.80,z*.30),(1,z*.48),(.92,z/2),(0,z/2)],(x/2,y/2),'ceramic',col)
    elif name=='Bedside lamp shade':lathe([(.76,-z/2),(1,z/2),(.95,z/2),(.71,-z/2),(.76,-z/2)],(x/2,y/2),'fabric',col)
    elif name in ['Pillow','Seat cushion','Back cushion','Terracotta throw pillow','Cream throw pillow','Mattress']:cushion((x,y,z),kind='fabric')
    elif name=='Duvet':cloth((x,y,z));tube([(-x/2,-y/2,z*.22),(x/2,-y/2,z*.22)],.16,'fabric','#b7b9ac')
    elif name=='Shower folded curtain':curtain((x,y,z))
    elif name=='Pale roller blind':
        box((0,0,0),(x,.4,z),.08,'fabric');tube([(-x/2,0,-z/2),(x/2,0,-z/2)],.45,'paint','#e7e4dc')
    elif name=='Folded blanket basket':
        box((0,0,-z/2+.6),(x-2,y-2,1.2),.5,'wood')
        for h in range(0,19):
            zz=-z/2+1+h*(z-2)/18
            tube([(-x/2+1,-y/2+1,zz),(x/2-1,-y/2+1,zz),(x/2-1,y/2-1,zz),(-x/2+1,y/2-1,zz)],.45,'wood',col,True)
        for j in range(10):
            a=-x/2+1+j*(x-2)/9
            for b in [-y/2+1,y/2-1]:tube([(a,b,-z/2+1),(a,b,z/2)],.4,'wood','#9d8465')
        cushion((x*.8,y*.85,12),(0,0,z*.15),'fabric','#ddd0b5')
    elif name=='Low bookcase':
        for xx in [x/2-1.2]:box((xx,0,0),(2.4,y,z),.2)
        for yy in [-y/2+1.2,y/2-1.2]:box((0,yy,0),(x,2.4,z),.3)
        for zz in [-z/2+1.2,-z/2+32,-z/2+60,z/2-1.2]:box((0,0,zz),(x,y-3,2.4),.35)
    elif name=='Shelf shadow':
        # Actual rear board, replacing a fake shadow painted across an opening.
        box((39,0,0),(.6,y,z),.02,'wood','#796246')
    elif name=='Book spine':
        # Real book pages extend from the spine into the open shelves.
        box((13,0,0),(25,y*.9,z*.96),.08,'paper','#d8d3c5')
        for yy in [-y/2+.1,y/2-.1]:box((13,yy,0),(26,.2,z),.1,'book',col)
        box((0,0,0),(.7,y,z),.18,'book',col)
    elif name in ['Book on coffee table','Magazine stack']:book((x,y,z))
    elif name=='Keyboard':
        box((0,0,-.3),(x,y,z*.65),.6,'plastic','#4f544f')
        for a in range(13):
            for b in range(4):box((-x/2+2+a*2.85,-y/2+2+b*2.6,z*.35),(2.4,2.05,.65),.15,'plastic','#7d827b')
        box((0,-y/2+1.6,z*.4),(13,1.8,.6),.15,'plastic','#858881')
    elif name=='Desk power cable':
        # Each section curves through its neighbours; no disconnected box links.
        prev=scene['objects'][max(0,index-1)];nex=scene['objects'][min(len(scene['objects'])-1,index+1)]
        p0=[0,-y/2,0];p1=[0,y/2,0]
        if prev['name']==name:p0=[(prev['center'][0]-o['center'][0])/2,-2,0]
        if nex['name']==name:p1=[(nex['center'][0]-o['center'][0])/2,2,0]
        tube([p0,(0,0,0),p1],x/2,'rubber','#343832')
    elif name=='Fruit':
        bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=16,radius=1);fruit=bpy.context.object
        for v in fruit.data.vertices:
            p=v.co.copy();v.co=(p.x*x/2,p.y*y/2,p.z*z/2*(.96-.08*abs(p.z)))
        keep(fruit,'fruit','#b9a552' if index%2 else '#ba6751',True);tube([(0,0,z*.42),(.2,0,z*.53)],.18,'wood','#53452b')
    elif name=='Picture print':box((0,0,0),(x,y,z),.1,'art','#ffffff')
    elif name=='Picture frame' or name=='Vanity mirror frame':
        axis=min(range(3),key=lambda j:o['size'][j]);axes=[j for j in range(3) if j!=axis]
        for a in axes:
            b=next(j for j in axes if j!=a)
            for sign in [-1,1]:s=list(o['size']);s[a]=3;p=[0,0,0];p[a]=sign*(o['size'][a]/2-1.5);box(p,s,.4)
    elif name=='Cardboard moving box':
        box((0,0,-.2),(x,y,z-.4),.35,'cardboard')
        tube([(-x/2,0,z/2),(x/2,0,z/2)],.05,'cardboard','#69583e')
        box((0,-y/2-.04,0),(x*.42,.08,z*.25),.01,'paper','#cfc2a3')
    elif name=='Shower tray':
        box((0,0,-z*.3),(x,y,z*.4),1.0,'ceramic')
        for a in [-1,1]:
            box((a*(x/2-2.5),0,0),(5,y,z),1.2,'ceramic');box((0,a*(y/2-2.5),0),(x-7,5,z),1.2,'ceramic')
        ring(2,2,-z*.08,.12,'metal','#969b97')
    elif name=='Hob ring':ring(x*.47,y*.47,0,.10,'metal','#a6aba5')
    elif name=='Tap spout':tube([(-0.,-y/2,-.4),(0,y*.25,0),(0,y/2,-1.0)],x/2,'metal','#a7aca7')
    elif o['group']=='rug' and name!='Rug border':
        box((0,0,0),(x,y,z),min(.14,z*.2),'fabric')
        for sign in [-1,1]:
            for j in range(max(2,int(x/2))):tube([(-x/2+1+j*2,sign*(y/2-.5),z*.1),(-x/2+1+j*2,sign*(y/2+1),z*.03)],.055,'fabric',col)
    elif 'leg' in name.lower() or name=='Chair back post':
        # Tapered leg mesh, retaining the existing support position and height.
        v=[(a*x/2*scale,b*y/2*scale,zz) for zz,scale in [(-z/2,.76),(z/2,1)] for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]]
        oleg=mesh('Tapered furniture support',v,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],smooth=False);b=oleg.modifiers.new('Support edge','BEVEL');b.width=min(.25,x*.07);b.segments=3
    elif name in ['Dining chair seat','Chair back']:cushion((x,y,z),kind='fabric')
    elif o['shape']=='cylinder':
        # Authored lathed cylindrical details, with rounded rims and closed ends.
        lathe([(0,-z/2),(.92,-z/2),(1,-z*.47),(1,z*.47),(.92,z/2),(0,z/2)],(x/2,y/2))
    else:
        bevel=.16 if o['group'] in ['wall','floor','ceiling','floor_finish','trim'] else min(1.3,min(x,y,z)*.12)
        if 'sofa' in name.lower():bevel=min(x,y,z)*.28
        if name=='Toy block':bevel=.28;mat='toy'
        box((0,0,0),(x,y,z),bevel)
        if name in ['Kitchen door front','Wardrobe door']:
            axis=min(range(3),key=lambda j:o['size'][j]);s=[q*.83 for q in o['size']];s[axis]=.16;p=[0,0,0];p[axis]=(-1 if axis==0 else 1)*(o['size'][axis]/2+.04);box(p,s,.06,'paint',col)
    # Evaluate modifiers and export split corner normals + UVs for each material.
    parts=[];tri_count=0;minv=[1e9]*3;maxv=[-1e9]*3
    for part_index,obj in enumerate(current):
        bpy.context.view_layer.objects.active=obj
        dg=bpy.context.evaluated_depsgraph_get();evaluated=obj.evaluated_get(dg);me=evaluated.to_mesh();me.calc_loop_triangles();uv_layer=me.uv_layers.new(name='HouseUV');me.uv_layers.active=uv_layer;uv_layer.active_render=True
        # Wood grain follows each part's longest dimension (legs, rails, boards).
        extent=[max((v.co[j] for v in me.vertices),default=0)-min((v.co[j] for v in me.vertices),default=0) for j in range(3)];long_axis=max(range(3),key=lambda j:extent[j])
        vertices=[];normals=[];uv=[];indices=[];lookup={}
        for triangle in me.loop_triangles:
            corners=[me.vertices[me.loops[li].vertex_index].co for li in triangle.loops]
            if (corners[1]-corners[0]).cross(corners[2]-corners[0]).length<1e-7:continue
            for li in triangle.loops:
                loop=me.loops[li];pos=obj.matrix_world@me.vertices[loop.vertex_index].co;normal=(obj.matrix_world.to_3x3()@me.corner_normals[li].vector).normalized()
                # Surface-oriented physical scale; floor uses one 8 x 6 m image.
                axis=max(range(3),key=lambda j:abs(normal[j]));a,b=[j for j in range(3) if j!=axis]
                if obj.get('surface') in ['wood','toy'] and long_axis in (a,b):a,b=[j for j in (a,b) if j!=long_axis][0],long_axis
                scale={'fabric':25,'leaf':25,'paper':25,'cardboard':25,'toy':12,'wood':70}.get(obj.get('surface'),60)
                coord=(pos[a]/scale,pos[b]/scale)
                if o.get('material')=='terrazzo':coord=((pos.x+x/2)/x,1-(pos.y+y/2)/y)
                if obj.get('surface')=='art':coord=((pos[a]+o['size'][a]/2)/o['size'][a],1-(pos[b]+o['size'][b]/2)/o['size'][b])
                uv_layer.data[li].uv=coord
                keyvert=tuple(round(v,5) for v in (*pos,*normal,*coord))
                if keyvert not in lookup:
                    lookup[keyvert]=len(vertices)//3;vertices.extend(keyvert[:3]);normals.extend(keyvert[3:6]);uv.extend(keyvert[6:])
                    for j in range(3):minv[j]=min(minv[j],pos[j]);maxv[j]=max(maxv[j],pos[j])
                indices.append(lookup[keyvert])
        baked=me.copy();evaluated.to_mesh_clear();obj.modifiers.clear();obj.data=baked
        if not indices:continue
        clean=[]
        for k in range(0,len(indices),3):
            ids=indices[k:k+3];a,b,c=[Vector(vertices[t*3:t*3+3]) for t in ids]
            if (b-a).cross(c-a).length>1e-7:clean.extend(ids)
        indices=clean
        parts.append(dict(vertices=vertices,normals=normals,uv=uv,triangles=indices,surface=obj['surface'],color=obj['color']))
        tri_count+=len(indices)//3
        # Place the native Blender model at the matching world transform.
        obj.name=f"{index:03}_{name}_{part_index:02}";obj.location.rotate(__import__('mathutils').Euler((0,0,math.radians(o['yaw']))));obj.location+=Vector(o['center']);obj.rotation_euler.z+=math.radians(o['yaw']);obj['source_name']=name;obj['source_index']=index
        all_meshes.append(obj)
    o['mesh']=f'asset_{index:03}';o['mesh_bounds_local_cm']=[minv,maxv]
    result[o['mesh']]=dict(name=name,parts=parts,triangles=tri_count,collision=o['collision'])
scene['version']=3;scene['title']='Blender furnished apartment';scene['mesh_file']='house-meshes.json'
scene['features']=['Blender-authored household meshes shared by Unreal and browser','Five rooms, white walls and ceiling, traditional 30 cm terrazzo','Curved upholstery, open slippers and bowls, real shelf cavities, woven textiles','Static collision follows visible triangle surfaces; doorway layout retained']
scene['limitations']=['Generic model, not a scan of the user home','Furniture and textiles are static rigid surfaces; no cloth deformation or cable entanglement','Window blinds are opaque; reflective mirrors depend on renderer reflection support','Lighting and material response are approximations, not calibrated household measurements']
(OUT/'Content/RobotData/house-scene.json').write_text(json.dumps(scene,separators=(',',':')))
(OUT/'Content/RobotData/house-meshes.json').write_text(json.dumps(dict(version=1,units='cm',triangle_winding='counterclockwise',assets=result),separators=(',',':')))
# Editable authoring file, metre display (one scene unit still represents 1 cm).
bpy.context.scene.unit_settings.system='METRIC';bpy.context.scene.unit_settings.scale_length=.01
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D':
        area.spaces.active.region_3d.view_location=Vector((100,0,70));area.spaces.active.region_3d.view_distance=1100
        area.spaces.active.shading.color_type='MATERIAL'
bpy.ops.file.pack_all()
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'Blender/furnished-apartment.blend'))
summary=dict(objects=len(result),mesh_parts=sum(len(a['parts']) for a in result.values()),triangles=sum(a['triangles'] for a in result.values()),collision_objects=sum(o['collision'] for o in scene['objects']),authoring='Blender 4.5 LTS; original procedural mesh assets; no third-party asset downloads',collision='Visible static triangle surfaces, including hollow objects; cables remain non-colliding visual geometry',bounds_cm=scene['bounds_cm'])
(OUT/'reports/house-asset-build.json').write_text(json.dumps(summary,indent=2));print('HOUSE_ASSETS_COMPLETE',json.dumps(summary))

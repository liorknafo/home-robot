"""Deterministic furnished apartment, shared by Unreal and browser inspection.
Units are Unreal centimetres (+X forward, +Y right, +Z up). No vision markers.
"""
from pathlib import Path
import json,random,math
R=Path(__file__).resolve().parents[1];items=[];rng=random.Random(41)
cream='#e9e5db';trim='#f4f0e6';wood='#9a7350';dark='#494640';sage='#798c80'
def box(name,c,s,col,collision=True,yaw=0,group='furniture',shape='box'):
    items.append(dict(name=name,center=c,size=s,color=col,collision=collision,yaw=yaw,group=group,shape=shape))
def wall(name,c,s,col=cream):
    box(name,c,s,col,group='wall');items[-1]['material']='white_paint'
    if s[2]>210:
        box(name+' skirting',[c[0],c[1],5],[s[0]+1.2,s[1]+1.2,10],trim,group='trim')
def wallx(x,y0,y1):wall('Plaster partition',[x,(y0+y1)/2,120],[10,y1-y0,240])
def wally(y,x0,x1):wall('Plaster partition',[(x0+x1)/2,y,120],[x1-x0,10,240])
def doorway(axis,coord,lo,hi,height=205):
    if axis=='x':
        wall('Door lintel',[coord,(lo+hi)/2,(height+240)/2],[10,hi-lo,240-height])
        for y in [lo+1,hi-1]:box('Door casing',[coord,y,height/2],[13,2,height],trim,group='trim')
        box('Door casing header',[coord,(lo+hi)/2,height],[13,hi-lo,3],trim,group='trim')
    else:
        wall('Door lintel',[(lo+hi)/2,coord,(height+240)/2],[hi-lo,10,240-height])
        for x in [lo+1,hi-1]:box('Door casing',[x,coord,height/2],[2,13,height],trim,group='trim')
        box('Door casing header',[(lo+hi)/2,coord,height],[hi-lo,13,3],trim,group='trim')
def legs(name,x,y,width,depth,height,thick=4,col=wood):
    for dx in [-width/2+thick/2,width/2-thick/2]:
        for dy in [-depth/2+thick/2,depth/2-thick/2]:box(name+' leg',[x+dx,y+dy,height/2],[thick,thick,height],col)
def table(name,x,y,w,d,h=74):
    legs(name,x,y,w-10,d-10,h-4);box(name+' top',[x,y,h-2],[w,d,4],wood)
def chair(x,y,yaw=0):
    # Local shape is rotated around chair center, including its slender legs.
    start=len(items);legs('Dining chair',x,y,38,40,43,3.2);box('Dining chair seat',[x,y,45],[42,44,4],sage)
    for dx in [-18,18]:box('Chair back post',[x+dx,y+19,66],[3,3,44],wood)
    box('Chair back',[x,y+19,77],[39,3,17],sage)
    a=math.radians(yaw)
    for o in items[start:]:
        px,py=o['center'][0]-x,o['center'][1]-y;o['center'][0]=x+px*math.cos(a)-py*math.sin(a);o['center'][1]=y+px*math.sin(a)+py*math.cos(a);o['yaw']=yaw

def plant(x,y,h=70):
    box('Terracotta pot',[x,y,12],[25,25,24],'#b18063',shape='cylinder')
    box('Soil',[x,y,24],[22,22,1],'#403931',False,shape='cylinder')
    for i in range(8):
        a=i*2.399;box('Plant leaf',[x+10*math.cos(a),y+10*math.sin(a),h-18+(i%3)*9],[26,12,14],['#637b59','#738762','#536b52'][i%3],False,yaw=math.degrees(a),shape='ellipsoid')
    box('Plant stem',[x,y,h/2],[2,2,h],'#675943',False,shape='cylinder')

# Reference-guided traditional Israeli terrazzo, 30cm tiles and narrow joints.
box('Continuous structural floor',[100,0,-5],[820,620,10],'#c3c0b8',group='floor')
box('Traditional terrazzo surface',[100,0,.015],[800,600,.03],'#ffffff',False,group='floor_finish')
items[-1]['material']='terrazzo'
box('White painted ceiling',[100,0,246],[800,600,12],'#ffffff',group='ceiling')
items[-1]['material']='white_paint'
# Outer shell, with actual openings for windows along the two long sides.
wall('West exterior',[-305,0,120],[10,610,240]);wall('East exterior',[505,0,120],[10,610,240])
for y in [-305,305]:
    wall('Window sill wall',[100,y,45],[800,10,90]);wall('Window header',[100,y,222.5],[800,10,35])
    for a,b in [(-300,-240),(-100,240),(420,500)]:wall('Wall beside window',[(a+b)/2,y,147.5],[b-a,10,115])
    for a,b in [(-240,-100),(240,420)]:
        for x in [a+2,b-2,(a+b)/2]:box('Window upright',[x,y,147],[4,9,114],trim,group='window')
        for z in [92,202]:box('Window frame',[(a+b)/2,y,z],[b-a,10,4],trim,group='window')
        box('Deep window sill',[(a+b)/2,y,90],[b-a+8,20,4],trim,group='window')
        # Closed pale blind above eye level; not pretending to simulate optical glass.
        box('Pale roller blind',[(a+b)/2,y+(-1 if y>0 else 1)*4,155],[b-a-4,1,90],'#d5d9d5',False,group='window')
# Main living/kitchen doorway: 86cm after trim. Smaller interior doors 76 / 66cm.
wallx(100,-300,-45);wallx(100,45,300);doorway('x',100,-45,45)
wally(100,-300,-80);wally(100,0,100);doorway('y',100,-80,0)
wally(70,100,170);wally(70,250,365);wally(70,435,500);doorway('y',70,170,250);doorway('y',70,365,435)
wallx(320,70,300)
box('10 mm hardwood threshold',[100,0,.5],[7,86,1],'#987959',group='threshold')
box('8 mm bathroom threshold',[400,70,.4],[66,7,.8],'#b8b4a9',group='threshold')
# Open bedroom and study door leaves, set against the adjacent walls.
box('Bedroom door open',[-85,141,102.5],[3.5,78,205],'#e5ddd0',group='door')
box('Study door open',[166,111,102.5],[3.5,78,205],'#e5ddd0',group='door')
# Living room: upholstered sofa, rug, coffee table, low TV unit, books, plants.
legs('Sofa',-243,-137,68,188,14,6,dark)
box('Sofa upholstered base',[-243,-137,25],[83,208,24],'#7b8980')
box('Sofa back',[-276,-137,55],[18,208,72],'#839187')
for y in [-237,-37]:box('Sofa arm',[-241,y,48],[88,15,48],'#74847b')
for y in [-200,-137,-74]:
    box('Seat cushion',[-229,y,42],[60,60,14],'#96a297')
    box('Back cushion',[-261,y,66],[17,60,38],'#9ba79d')
box('Terracotta throw pillow',[-247,-196,68],[23,36,32],'#b48b76',yaw=8)
box('Cream throw pillow',[-247,-73,68],[23,36,32],'#d3c8b6',yaw=-9)
box('Woven living rug',[-84,-149,.3],[190,212,.6],'#b1aaa0',group='rug')
for x in [-173,5]:box('Rug border',[x,-149,.615],[4,204,.02],'#918b81',False,group='rug')
for y in [-246,-52]:box('Rug border',[-84,y,.615],[180,4,.02],'#918b81',False,group='rug')
table('Coffee table',-95,-150,75,105,42)
for i,col in enumerate(['#d2c6b0','#866e5e','#7d8984']):box('Book on coffee table',[-104,-151,43.8+i*2],[22,30,2],col,yaw=i*4)
box('Coffee mug',[-76,-127,46],[8,8,9],'#e7e1d3',shape='cylinder')
legs('TV console',66,-171,42,154,12,4,dark);box('TV console',[66,-171,32],[46,165,39],wood)
for y in [-225,-171,-117]:box('Console drawer front',[41.8,y,34],[1.5,51,28],'#af8d67')
box('Television frame',[69,-171,94],[5,137,78],'#313633');box('Television screen',[66.3,-171,94],[.4,129,70],'#48595c',False)
plant(-260,53,86);plant(62,-271,65)
box('Folded blanket basket',[-262,-276,19],[36,34,38],'#b19b7d')
for i in range(3):box('Magazine stack',[24,-61,1.5+i*1.5],[23,31,1.5],['#d8c8ac','#788889','#b0a289'][i],yaw=-12)
# Kitchen cabinetry, tiled splashback, appliance fronts and narrow toe kicks.
for x in [140,200,260,320,380,440]:
    box('Kitchen base cabinet',[x,-268,45],[58,56,82],'#d0d4c9')
    box('Cabinet plinth',[x,-270,5],[56,48,10],'#535951')
    box('Kitchen drawer front',[x,-238.8,76],[54,1.5,15],'#e4e5dc')
    box('Kitchen door front',[x,-238.8,38],[54,1.5,55],'#e4e5dc')
    box('Drawer pull',[x,-237,76],[21,2.5,1.5],'#6e746d')
box('Kitchen countertop',[289,-267,89],[364,62,4],'#dcd8cf')
for x in range(110,476,20):
    for z in [104,124]:box('Splashback tile',[x,-299,z],[19.7,.5,19.7],'#d9e0db',False)
box('Inset sink',[260,-266,91.1],[46,35,1.2],'#687779',False)
box('Sink basin',[260,-266,91.8],[37,27,.5],'#879595',False)
box('Tap upright',[260,-284,103],[2.5,2.5,26],'#828b85',False,shape='cylinder')
box('Tap spout',[260,-278,115],[2.5,15,2.5],'#828b85',False)
box('Induction hob',[380,-266,91.2],[51,43,.8],'#383d3b')
for x in [366,394]:
    for y in [-278,-253]:box('Hob ring',[x,y,91.7],[16,16,.2],'#60655f',False,shape='cylinder')
box('Oven fascia',[380,-237.7,43],[51,1,43],'#414641');box('Oven handle',[380,-235.5,62],[36,3,2],'#898d84')
box('Tall refrigerator',[469,-171,92],[59,65,184],'#ced2cc');box('Fridge seam',[469,-137.9,64],[57,.5,1],'#8e978c',False)
for z in [42,132]:box('Fridge handle',[444,-135,z],[2,4,26],'#727d75')
# Dining table and chairs produce real leg occlusions.
table('Dining table',347,-59,112,78)
chair(347,-121,180);chair(347,4);chair(270,-57,-90);chair(424,-57,90)
box('Table runner',[347,-59,74.1],[34,73,.2],'#b19d84',False)
box('Fruit bowl',[347,-59,78],[23,23,8],'#b4aea0',shape='ellipsoid')
for i in range(4):box('Fruit',[341+i*4,-60+(i%2)*5,82],[7,7,7],['#aa814d','#9a7048'][i%2],False,shape='ellipsoid')
# Bedroom, rug, wardrobe, bedside lamps.
legs('Bed frame',-200,204,160,170,17,6);box('Bed frame',[-200,204,22],[172,184,19],'#ac906d')
box('Mattress',[-200,204,41],[166,178,22],'#ddd7c8');box('Duvet',[-200,186,55],[164,135,9],'#a5aba0')
box('Headboard',[-200,295,53],[178,8,100],'#8b7761')
for x in [-242,-158]:box('Pillow',[x,265,58],[65,42,11],'#e9e3d5')
box('Bedroom runner',[-57,216,.25],[65,133,.5],'#b9aaa1',group='rug')
box('Bedside table',[-81,271,25],[42,45,50],wood)
box('Bedside lamp stem',[-81,271,62],[3,3,26],'#736853',shape='cylinder')
box('Bedside lamp shade',[-81,271,77],[25,25,24],'#e8ddc4',False,shape='cylinder')
box('Wardrobe',[69,218,106],[51,155,212],'#dfdcd1')
for y in [181,255]:
    box('Wardrobe door',[42.8,y,108],[1,71,200],'#e8e4d9');box('Wardrobe handle',[41,y+24,102],[2,2,24],'#8c897e')
# Study: desk, chair, low shelves with books, moving box, cable on floor.
table('Writing desk',215,269,153,53,74);box('Monitor base',[219,271,75],[30,18,2],dark)
box('Monitor stem',[219,280,91],[5,5,34],dark);box('Monitor screen',[219,280,110],[57,4,34],'#3b4c51')
box('Keyboard',[219,254,76],[40,14,2],'#666d64');chair(215,205,180)
box('Low bookcase',[289,173,47],[42,109,94],wood)
for z in [22,50,78]:
    box('Shelf shadow',[266.9,173,z],[.6,102,23],'#655a4c',False)
    for j in range(10):box('Book spine',[265.9,130+j*9,z],[2.4,6+(j%3),17+(j%4)],['#889186','#b6a285','#827365','#c1b8a5'][j%4],False)
box('Cardboard moving box',[129,243,19],[42,38,38],'#ad9471');box('Box tape',[129,243,38.2],[7,38,.4],'#c4b490',False)
for i in range(12):box('Desk power cable',[258+8*math.sin(i*.32),220+i*4,.35],[.7,4.5,.7],'#46483f',False,group='clutter')
# Bathroom: vanity, toilet, shower tray with frame (no simulated glass).
box('Vanity',[466,139,39],[57,95,78],'#b9b5a7');box('Vanity countertop',[466,139,81],[61,98,6],'#e4e1d5')
box('Vanity basin',[466,139,85],[39,52,5],'#bcc4bd',shape='ellipsoid')
box('Vanity mirror frame',[498,140,142],[3,86,85],'#b9b3a4');box('Mirror opaque placeholder',[496.2,140,142],[.5,80,79],'#8eaaa7',False)
box('Toilet base',[358,166,20],[33,47,40],'#e6e6dc',shape='ellipsoid')
box('Toilet bowl',[358,162,42],[39,55,12],'#f1eee2',shape='ellipsoid');box('Toilet tank',[358,191,55],[37,19,63],'#ebe8dc')
box('Shower tray',[412,254,3],[161,82,6],'#d1d4cc')
for x in [334,490]:box('Shower upright',[x,213,103],[2,2,206],'#9ca69b')
box('Shower rail',[412,213,205],[158,2,2],'#9ca69b')
box('Shower folded curtain',[339,216,117],[11,4,166],'#b7c6be',False)
box('Bathroom mat',[411,151,.35],[41,67,.7],'#82928b',group='rug')
# Household clutter: one slipper pair and toys, offset from the nominal route.
for y in [18,35]:box('Slipper',[-129,y,3],[27,11,6],'#a89886',shape='ellipsoid')
for x,y in [(-163,-10),(-156,-18),(-166,-21)]:box('Toy block',[x,y,2.5],[5,5,5],'#a17757',yaw=14)
# A small number of ordinary wall pictures above camera height.
for x,y,z,s in [(-299,-112,143,[3,88,64]),(498,-48,147,[3,80,55]),(-120,98,145,[63,3,70])]:
    box('Picture frame',[x,y,z],s,'#736b5b',False,group='decor')
    inset=[max(.5,d-7) for d in s];inset[s.index(min(s))]=min(s)+.3
    box('Picture print',[x,y,z],inset,'#b3b6a3',False,group='decor')
scene=dict(version=2,title='Furnished apartment',units='cm',coordinates='Unreal +X forward +Y right +Z up',bounds_cm=[-310,510,-310,310],rooms=[dict(name='Living room',center=[-100,-100,0]),dict(name='Kitchen / dining',center=[300,-110,0]),dict(name='Bedroom',center=[-100,200,0]),dict(name='Study',center=[210,185,0]),dict(name='Bathroom',center=[410,185,0])],features=['Five rooms with 66–86 cm clear door openings','White painted walls and ceiling; traditional 30 cm terrazzo with mild polish','3.2 cm chair legs, rugs, 8 / 10 mm thresholds and household clutter'],limitations=['Procedural geometry and approximate materials; not a scan of this home','Opaque mirror and blinds; transparent glass and true mirror reflection not modeled','Ceiling visible to robot cameras, hidden in overhead inspection','Terrazzo appearance and finish are approximate; screen-space reflections omit off-screen objects','Small cable geometry is visual only'],objects=items)
(R/'Content/RobotData/house-scene.json').write_text(json.dumps(scene,separators=(',',':')))
print(f'House: {len(items)} shapes, {sum(x["collision"] for x in items)} collision envelopes')

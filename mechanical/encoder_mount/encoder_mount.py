"""Simple direct-drive odometry mount. All dimensions in mm, angles in degrees.
Edit the parameters below and run in FreeCAD's Python console:
    p = '/absolute/path/encoder_mount.py'
    exec(compile(open(p).read(), p, 'exec'), {'__file__': p})
The script creates a new document and regenerates four side-specific printable STLs.
Part-local X = trailing direction; local Y = shaft axis; local front face Y=0.
Vehicle coordinates: +X forward, +Y right, centerline Y=0, ground Z=0.
The unchanged assembly is rigidly rotated 180 degrees about Z and positioned outside the edge.
"""
from pathlib import Path
import math
import inspect

MOUNT_SIDE = "RIGHT"  # "RIGHT" or "LEFT": selected/default installation; both are always generated
CHASSIS_HALF_WIDTH = 80.0  # UNKNOWN reference half-width; edges Y=+/-80, verify on vehicle

# Encoder: measured dimensions and explicitly marked provisional details.
ENCODER_BODY_DIAMETER = 38.0
ENCODER_BODY_LENGTH = 35.0  # cylindrical body, in addition to flange
ENCODER_FLANGE_THICKNESS = 5.0
ENCODER_FLANGE_DIAMETER = 38.0
ENCODER_SHAFT_DIAMETER = 6.0
ENCODER_SHAFT_LENGTH = 12.0  # assumed measured from front flange mounting face
ENCODER_BOSS_DIAMETER = 20.0
ENCODER_BOSS_PROJECTION = 2.0  # UNKNOWN: verify actual projection
ENCODER_PATTERN_A_PCD = 28.0
ENCODER_PATTERN_A_ANGLES = (30.0, 150.0, 270.0)
ENCODER_PATTERN_B_PCD = 30.0
ENCODER_PATTERN_B_ANGLES = (0.0, 120.0, 240.0)
# Front view from wheel side toward encoder: clockwise from local +Z vertical.
ENCODER_MOUNT_HOLE_DIAMETER = 3.4
ENCODER_REFERENCE_THREAD_DIAMETER = 2.5  # simplified holes, no modeled thread

WHEEL_DIAMETER = 61.0
WHEEL_WIDTH = 6.0
WHEEL_BORE = 6.0
WHEEL_HUB_DIAMETER = 20.0
WHEEL_HUB_LENGTH = 9.0  # hub projection in addition to 6 mm disk
HUB_TO_MOUNT_PLATE_GAP = 0.6
MINIMUM_SHAFT_ENGAGEMENT = 8.0

CHASSIS_HEIGHT = 76.0  # underside of reference plate
CHASSIS_HOLE_SPACING = 13.0
CHASSIS_HOLE_DIAMETER = 3.4
CHASSIS_REFERENCE_LENGTH = 180.0  # reference only, central clear region
CHASSIS_PLATE_THICKNESS = 3.0  # UNKNOWN actual plate thickness; underside stays at Z=76
CHASSIS_MOUNT_HOLE_EDGE_DISTANCE = 10.0  # UNKNOWN measured row location; inboard from edge
LONGITUDINAL_FREE_SPACE = 180.0
MAIN_WHEEL_OUTER_Y = 10.0  # measured distance OUTWARD from chassis edge
REFERENCE_WHEEL_DEPTH = 60.0  # visualization extent beyond +/-90, NOT tire diameter
REFERENCE_WHEEL_WIDTH = 30.0  # UNKNOWN; rectangular exclusion-volume illustration
REFERENCE_WHEEL_HEIGHT = 100.0  # UNKNOWN; actual tire outline is not asserted
ENCODER_FRONT_FACE_Y = 22.0  # placement-only choice, puts wheel innermost face at Y=3.4
FRONT_BOUNDARY_CLEARANCE = 5.0  # keep existing bracket behind front clear-region boundary

ARM_LENGTH = 70.0  # pivot to encoder shaft center
ARM_THICKNESS = 5.0
ARM_WIDTH = 14.0
ENCODER_MOUNT_RADIUS = 23.0
ENCODER_MOUNT_PLATE_THICKNESS = 3.0  # local relief preserves shaft engagement
BOSS_DIAMETRAL_CLEARANCE = 0.4
M3_COUNTERSINK_DIAMETER = 6.4  # flush 90-degree flat-head M3 screws required
BRACKET_THICKNESS = 5.0  # vertical wall
BRACKET_FLANGE_THICKNESS = 5.0
CHASSIS_FLANGE_OVERLAP = 20.0  # inward from the corresponding chassis side edge
PIVOT_DIAMETER = 4.0  # independent M4 articulation pivot
PIVOT_HOLE_DIAMETER = 4.4
PIVOT_BOSS_RADIUS = 10.0
PIVOT_DROP_BELOW_CHASSIS = 20.0
PIVOT_SIDE_CLEARANCE = 0.3
NOMINAL_ARM_ANGLE = math.degrees(math.asin(
    (WHEEL_DIAMETER/2-(CHASSIS_HEIGHT-PIVOT_DROP_BELOW_CHASSIS))/ARM_LENGTH))
MAX_UP_ANGLE = 12.0  # relative to nominal
MAX_DOWN_ANGLE = 3.0  # relative to nominal; droop/compliance allowance

BRACKET_REAR_LENGTH = 30.0
BRACKET_FRONT_LENGTH = 12.0
BRACKET_BOTTOM_DROP = 16.0
STOP_LEVER_RADIUS = 18.0
STOP_NECK_WIDTH = 4.0
STOP_FOLLOWER_RADIUS = 3.0
STOP_POST_RADIUS = 3.0
SPRING_HOLE_DIAMETER = 3.2
SPRING_ARM_X = 30.0
SPRING_ARM_Z = -10.0
SPRING_TAB_RADIUS = 5.0
SPRING_BRACKET_X = 6.0
SPRING_BRACKET_DROP = 14.0
CABLE_CLEARANCE_RADIUS = 5.0  # UNKNOWN assumed rear axial cable exit
CABLE_CLEARANCE_LENGTH = 20.0
PIVOT_HEAD_DIAMETER = 8.0
PIVOT_HEAD_THICKNESS = 3.0
PIVOT_PLACEHOLDER_INBOARD_Y = -5.8  # retain the existing placeholder geometry/placement
PIVOT_PIN_EXTRA_LENGTH = 2.0  # placeholder only; actual bolt length needs nut/washer allowance
MESH_LINEAR_DEFLECTION = 0.08
MESH_ANGULAR_DEFLECTION = 0.15
TRAVEL_CHECK_POSITIONS = 31
OUTPUT_DIRECTORY = Path(globals().get('__file__', inspect.currentframe().f_code.co_filename)).resolve().parent

import FreeCAD as App
import Part
import MeshPart
if App.GuiUp:
    import FreeCADGui as Gui

V = App.Vector
Y = V(0,1,0)
pivot_z = CHASSIS_HEIGHT-PIVOT_DROP_BELOW_CHASSIS
pivot_vehicle_x = LONGITUDINAL_FREE_SPACE/2-BRACKET_REAR_LENGTH-FRONT_BOUNDARY_CLEARANCE
wall_y = -PIVOT_SIDE_CLEARANCE-BRACKET_THICKNESS
hub_y = ENCODER_MOUNT_PLATE_THICKNESS+HUB_TO_MOUNT_PLATE_GAP
shaft_engagement = min(WHEEL_HUB_LENGTH,ENCODER_SHAFT_LENGTH-hub_y)
assert MOUNT_SIDE in ("RIGHT","LEFT"), 'MOUNT_SIDE must be RIGHT or LEFT'
assert CHASSIS_HALF_WIDTH>CHASSIS_FLANGE_OVERLAP
assert 0 < ENCODER_MOUNT_PLATE_THICKNESS <= ARM_THICKNESS
assert shaft_engagement >= MINIMUM_SHAFT_ENGAGEMENT, 'Insufficient shaft engagement'
assert ENCODER_BOSS_PROJECTION < hub_y, 'Boss would touch wheel hub'
assert M3_COUNTERSINK_DIAMETER < min(ENCODER_PATTERN_A_PCD,ENCODER_PATTERN_B_PCD)-ENCODER_BOSS_DIAMETER-BOSS_DIAMETRAL_CLEARANCE
assert -85 < NOMINAL_ARM_ANGLE-MAX_DOWN_ANGLE < NOMINAL_ARM_ANGLE+MAX_UP_ANGLE < 0


def cylinder(radius,length,x=0,y=0,z=0):
    return Part.makeCylinder(radius,length,V(x,y,z),Y)


def box(dx,dy,dz,x,y,z):
    return Part.makeBox(dx,dy,dz,V(x,y,z))


def link(x1,z1,x2,z2,radius,thickness):
    dx,dz=x2-x1,z2-z1
    length=math.hypot(dx,dz)
    nx,nz=-dz*radius/length,dx*radius/length
    p=[V(x1+nx,0,z1+nz),V(x2+nx,0,z2+nz),V(x2-nx,0,z2-nz),V(x1-nx,0,z1-nz)]
    web=Part.Face(Part.makePolygon(p+[p[0]])).extrude(V(0,thickness,0))
    return web.fuse(cylinder(radius,thickness,x1,0,z1)).fuse(cylinder(radius,thickness,x2,0,z2))


def articulate(shape,angle):
    result=shape.copy()
    result.rotate(V(0,0,0),V(0,-1,0),angle)
    result.translate(V(0,0,pivot_z))
    return result


def mirror_y(shape):
    """True geometric reflection through the vehicle X-Z plane; preserves solid orientation."""
    return shape.mirror(V(0,0,0),V(0,1,0))


def vehicle_position(shape,side=None):
    """Existing edge-relative RIGHT pose; LEFT is exactly its Y -> -Y mirror."""
    side=MOUNT_SIDE if side is None else side
    assert side in ('RIGHT','LEFT')
    result=shape.copy()
    result.rotate(V(0,0,0),V(0,0,1),180)
    result.translate(V(pivot_vehicle_x,CHASSIS_HALF_WIDTH+ENCODER_FRONT_FACE_Y,0))
    return mirror_y(result) if side=='LEFT' else result


def outside_edge_clearance(shape,side):
    bb=shape.BoundBox
    return bb.YMin-CHASSIS_HALF_WIDTH if side=='RIGHT' else -bb.YMax-CHASSIS_HALF_WIDTH


# Single flat arm with flange ring, pivot, rear stop follower and one spring hole.
arm=link(0,0,ARM_LENGTH,0,ARM_WIDTH/2,ARM_THICKNESS)
arm=arm.fuse(cylinder(PIVOT_BOSS_RADIUS,ARM_THICKNESS))
arm=arm.fuse(cylinder(ENCODER_MOUNT_RADIUS,ARM_THICKNESS,ARM_LENGTH))
arm=arm.fuse(link(0,0,-STOP_LEVER_RADIUS,0,STOP_NECK_WIDTH/2,ARM_THICKNESS))
arm=arm.fuse(cylinder(STOP_FOLLOWER_RADIUS,ARM_THICKNESS,-STOP_LEVER_RADIUS))
arm=arm.fuse(link(SPRING_ARM_X,0,SPRING_ARM_X,SPRING_ARM_Z,SPRING_TAB_RADIUS,ARM_THICKNESS))
# 3 mm flange ring keeps wheel hub close enough for 8.4 mm shaft engagement.
arm=arm.cut(cylinder(ENCODER_MOUNT_RADIUS+0.01,ARM_THICKNESS,ARM_LENGTH,ENCODER_MOUNT_PLATE_THICKNESS))
arm=arm.cut(cylinder((ENCODER_BOSS_DIAMETER+BOSS_DIAMETRAL_CLEARANCE)/2,ARM_THICKNESS+2,ARM_LENGTH,-1))
arm=arm.cut(cylinder(PIVOT_HOLE_DIAMETER/2,ARM_THICKNESS+2,0,-1))
arm=arm.cut(cylinder(SPRING_HOLE_DIAMETER/2,ARM_THICKNESS+2,SPRING_ARM_X,-1,SPRING_ARM_Z))
mount_centers=[]
for pcd,angles in ((ENCODER_PATTERN_A_PCD,ENCODER_PATTERN_A_ANGLES),
                   (ENCODER_PATTERN_B_PCD,ENCODER_PATTERN_B_ANGLES)):
    for angle in angles:
        a=math.radians(angle)
        # Wheel-side view looks along -Y: screen right is local -X.
        # Zero is +Z vertical. Shaft and boss stay at (ARM_LENGTH,0).
        x,z=ARM_LENGTH-pcd/2*math.sin(a),pcd/2*math.cos(a)
        mount_centers.append((x,z))
        arm=arm.cut(cylinder(ENCODER_MOUNT_HOLE_DIAMETER/2,ARM_THICKNESS+2,x,-1,z))
        depth=(M3_COUNTERSINK_DIAMETER-ENCODER_MOUNT_HOLE_DIAMETER)/2
        arm=arm.cut(Part.makeCone(ENCODER_MOUNT_HOLE_DIAMETER/2,M3_COUNTERSINK_DIAMETER/2,depth,
                                 V(x,ENCODER_MOUNT_PLATE_THICKNESS-depth,z),Y))
arm=arm.removeSplitter()

# One-piece top-mounted L bracket. Build in the existing assembly-local axes;
# local +Y points INBOARD after vehicle_position(). Pivot and arm stay fixed.
chassis_top_z=CHASSIS_HEIGHT+CHASSIS_PLATE_THICKNESS
base_z=chassis_top_z
flange_local_y_max=ENCODER_FRONT_FACE_Y+CHASSIS_FLANGE_OVERLAP
mounting_hole_local_y=ENCODER_FRONT_FACE_Y+CHASSIS_MOUNT_HOLE_EDGE_DISTANCE
assert CHASSIS_PLATE_THICKNESS>0 and BRACKET_FLANGE_THICKNESS>0
assert CHASSIS_FLANGE_OVERLAP>CHASSIS_MOUNT_HOLE_EDGE_DISTANCE+CHASSIS_HOLE_DIAMETER/2
assert CHASSIS_MOUNT_HOLE_EDGE_DISTANCE>CHASSIS_HOLE_DIAMETER/2
bracket=box(BRACKET_REAR_LENGTH+BRACKET_FRONT_LENGTH,flange_local_y_max-wall_y,
            BRACKET_FLANGE_THICKNESS,-BRACKET_REAR_LENGTH,wall_y,base_z)
bracket=bracket.fuse(box(BRACKET_REAR_LENGTH+BRACKET_FRONT_LENGTH,BRACKET_THICKNESS,
            base_z-(pivot_z-BRACKET_BOTTOM_DROP),-BRACKET_REAR_LENGTH,wall_y,pivot_z-BRACKET_BOTTOM_DROP))
bracket=bracket.cut(cylinder(PIVOT_HOLE_DIAMETER/2,BRACKET_THICKNESS+2,0,wall_y-1,pivot_z))
for x in (-CHASSIS_HOLE_SPACING/2,CHASSIS_HOLE_SPACING/2):
    bracket=bracket.cut(Part.makeCylinder(CHASSIS_HOLE_DIAMETER/2,BRACKET_FLANGE_THICKNESS+2,
                                         V(x,mounting_hole_local_y,base_z-1)))
bracket=bracket.cut(cylinder(SPRING_HOLE_DIAMETER/2,BRACKET_THICKNESS+2,
                            SPRING_BRACKET_X,wall_y-1,pivot_z-SPRING_BRACKET_DROP))
stop_delta=math.degrees(2*math.asin((STOP_FOLLOWER_RADIUS+STOP_POST_RADIUS)/(2*STOP_LEVER_RADIUS)))
for angle in (NOMINAL_ARM_ANGLE+MAX_UP_ANGLE+180+stop_delta,
              NOMINAL_ARM_ANGLE-MAX_DOWN_ANGLE+180-stop_delta):
    a=math.radians(angle)
    bracket=bracket.fuse(cylinder(STOP_POST_RADIUS,ARM_THICKNESS+PIVOT_SIDE_CLEARANCE-wall_y,
                 STOP_LEVER_RADIUS*math.cos(a),wall_y,pivot_z+STOP_LEVER_RADIUS*math.sin(a)))
bracket=bracket.removeSplitter()

# One shared vehicle reference with symmetric mounting rows at both side edges.
chassis=box(CHASSIS_REFERENCE_LENGTH,2*CHASSIS_HALF_WIDTH,CHASSIS_PLATE_THICKNESS,
            -CHASSIS_REFERENCE_LENGTH/2,-CHASSIS_HALF_WIDTH,CHASSIS_HEIGHT)
for sign in (1,-1):
    for dx in (-CHASSIS_HOLE_SPACING/2,CHASSIS_HOLE_SPACING/2):
        chassis=chassis.cut(Part.makeCylinder(CHASSIS_HOLE_DIAMETER/2,CHASSIS_PLATE_THICKNESS+2,
              V(pivot_vehicle_x+dx,sign*(CHASSIS_HALF_WIDTH-CHASSIS_MOUNT_HOLE_EDGE_DISTANCE),CHASSIS_HEIGHT-1)))
front_wheel_reference=box(REFERENCE_WHEEL_DEPTH,REFERENCE_WHEEL_WIDTH,REFERENCE_WHEEL_HEIGHT,
       LONGITUDINAL_FREE_SPACE/2,CHASSIS_HALF_WIDTH+MAIN_WHEEL_OUTER_Y-REFERENCE_WHEEL_WIDTH,0)
rear_wheel_reference=box(REFERENCE_WHEEL_DEPTH,REFERENCE_WHEEL_WIDTH,REFERENCE_WHEEL_HEIGHT,
       -LONGITUDINAL_FREE_SPACE/2-REFERENCE_WHEEL_DEPTH,CHASSIS_HALF_WIDTH+MAIN_WHEEL_OUTER_Y-REFERENCE_WHEEL_WIDTH,0)
wheel_references={
    'RIGHT':(front_wheel_reference,rear_wheel_reference),
    'LEFT':(mirror_y(front_wheel_reference),mirror_y(rear_wheel_reference))}
all_wheel_references=[shape for refs in wheel_references.values() for shape in refs]
encoder=cylinder(ENCODER_BODY_DIAMETER/2,ENCODER_BODY_LENGTH,ARM_LENGTH,-ENCODER_FLANGE_THICKNESS-ENCODER_BODY_LENGTH)
encoder=encoder.fuse(cylinder(ENCODER_FLANGE_DIAMETER/2,ENCODER_FLANGE_THICKNESS,ARM_LENGTH,-ENCODER_FLANGE_THICKNESS))
encoder=encoder.fuse(cylinder(ENCODER_BOSS_DIAMETER/2,ENCODER_BOSS_PROJECTION,ARM_LENGTH))
encoder=encoder.fuse(cylinder(ENCODER_SHAFT_DIAMETER/2,ENCODER_SHAFT_LENGTH,ARM_LENGTH))
for x,z in mount_centers:
    encoder=encoder.cut(cylinder(ENCODER_REFERENCE_THREAD_DIAMETER/2,ENCODER_FLANGE_THICKNESS,x,-ENCODER_FLANGE_THICKNESS,z))
encoder=encoder.removeSplitter()
wheel=cylinder(WHEEL_HUB_DIAMETER/2,WHEEL_HUB_LENGTH,ARM_LENGTH,hub_y)
wheel=wheel.fuse(cylinder(WHEEL_DIAMETER/2,WHEEL_WIDTH,ARM_LENGTH,hub_y+WHEEL_HUB_LENGTH))
wheel=wheel.cut(cylinder(WHEEL_BORE/2,WHEEL_HUB_LENGTH+WHEEL_WIDTH+2,ARM_LENGTH,hub_y-1)).removeSplitter()
pin=cylinder(PIVOT_DIAMETER/2,ARM_THICKNESS-PIVOT_PLACEHOLDER_INBOARD_Y+PIVOT_PIN_EXTRA_LENGTH/2,0,PIVOT_PLACEHOLDER_INBOARD_Y,pivot_z)
pin=pin.fuse(cylinder(PIVOT_HEAD_DIAMETER/2,PIVOT_HEAD_THICKNESS,0,
                      PIVOT_PLACEHOLDER_INBOARD_Y-PIVOT_HEAD_THICKNESS,pivot_z)).removeSplitter()

# Geometry-only cable keep-out for validation; not another document component.
cable=cylinder(CABLE_CLEARANCE_RADIUS,CABLE_CLEARANCE_LENGTH,ARM_LENGTH,
                -ENCODER_BODY_LENGTH-ENCODER_FLANGE_THICKNESS-CABLE_CLEARANCE_LENGTH)
assert abs(pivot_z+ARM_LENGTH*math.sin(math.radians(NOMINAL_ARM_ANGLE))-WHEEL_DIAMETER/2)<1e-7
assert articulate(wheel,NOMINAL_ARM_ANGLE).BoundBox.ZMin > -1e-6
assert abs(articulate(wheel,NOMINAL_ARM_ANGLE).BoundBox.ZMin)<1e-6,'Wheel must touch ground'
for a,b in ((wheel,encoder),(wheel,arm),(encoder,arm),(cable,arm)):
    assert a.common(b).Volume<1e-5,'Encoder/wheel/arm/cable collision'
validation={}
for side in ('RIGHT','LEFT'):
    fixed_bracket=vehicle_position(bracket,side)
    fixed_pin=vehicle_position(pin,side)
    assert fixed_bracket.common(chassis).Volume<1e-6,'Bracket intersects chassis '+side
    assert fixed_bracket.distToShape(chassis)[0]<1e-7,'Flange does not contact chassis '+side
    bb=fixed_bracket.BoundBox
    overlap=CHASSIS_HALF_WIDTH-bb.YMin if side=='RIGHT' else bb.YMax+CHASSIS_HALF_WIDTH
    assert abs(overlap-CHASSIS_FLANGE_OVERLAP)<1e-6,'Flange overlap changed '+side
    sign=1 if side=='RIGHT' else -1
    for dx in (-CHASSIS_HOLE_SPACING/2,CHASSIS_HOLE_SPACING/2):
        hole_y=sign*(CHASSIS_HALF_WIDTH-CHASSIS_MOUNT_HOLE_EDGE_DISTANCE)
        through=Part.makeCylinder(CHASSIS_HOLE_DIAMETER/2-.01,
            CHASSIS_PLATE_THICKNESS+BRACKET_FLANGE_THICKNESS+2,
            V(pivot_vehicle_x+dx,hole_y,CHASSIS_HEIGHT-1))
        assert through.common(chassis).Volume<1e-7
        assert through.common(fixed_bracket).Volume<1e-7,'M3 holes not aligned '+side
    metrics={'front':float('inf'),'rear':float('inf'),'encoder_chassis':float('inf'),
             'cable_chassis':float('inf'),'front_x':float('inf'),'rear_x':float('inf')}
    for step in range(TRAVEL_CHECK_POSITIONS):
        angle=NOMINAL_ARM_ANGLE-MAX_DOWN_ANGLE+(MAX_UP_ANGLE+MAX_DOWN_ANGLE)*step/(TRAVEL_CHECK_POSITIONS-1)
        moving=[]
        for name,shape in [('arm',arm),('encoder',encoder),('wheel',wheel),('cable',cable)]:
            placed=vehicle_position(articulate(shape,angle),side)
            assert placed.isValid() and len(placed.Solids)==1,'Invalid mirrored solid '+side+' '+name
            assert placed.common(fixed_bracket).Volume<1e-5,'Moving part hits bracket '+side
            assert placed.common(chassis).Volume<1e-5,'Moving part hits chassis '+side
            for ref in all_wheel_references:
                assert placed.common(ref).Volume<1e-5,'Moving part hits vehicle wheel reference '+side
            # Hardware stays in its own half of the vehicle for every arm position.
            assert sign*placed.Solids[0].CenterOfMass.y>0
            if side=='RIGHT':assert placed.BoundBox.YMin>0
            else:assert placed.BoundBox.YMax<0
            if name in ('encoder','wheel','cable'):
                assert outside_edge_clearance(placed,side)>0,'Part not outside chassis '+side
            if name=='cable':metrics['cable_chassis']=min(metrics['cable_chassis'],placed.distToShape(chassis)[0])
            else:moving.append(placed)
            if name=='encoder':metrics['encoder_chassis']=min(metrics['encoder_chassis'],placed.distToShape(chassis)[0])
            if name=='wheel' and abs(angle-NOMINAL_ARM_ANGLE)<1e-7:
                assert abs(placed.BoundBox.ZMin)<1e-6,'Nominal ground contact '+side
        for i,first in enumerate(moving):
            for second in moving[i+1:]:assert first.common(second).Volume<1e-5,'Moving parts intersect '+side
        for placed in moving+[fixed_bracket,fixed_pin]:
            front_ref,rear_ref=wheel_references[side]
            metrics['front']=min(metrics['front'],placed.distToShape(front_ref)[0])
            metrics['rear']=min(metrics['rear'],placed.distToShape(rear_ref)[0])
            metrics['front_x']=min(metrics['front_x'],LONGITUDINAL_FREE_SPACE/2-placed.BoundBox.XMax)
            metrics['rear_x']=min(metrics['rear_x'],placed.BoundBox.XMin+LONGITUDINAL_FREE_SPACE/2)
    nominal_wheel=vehicle_position(articulate(wheel,NOMINAL_ARM_ANGLE),side)
    assert abs(nominal_wheel.BoundBox.ZMin)<1e-6
    for angle in (NOMINAL_ARM_ANGLE-MAX_DOWN_ANGLE-0.2,NOMINAL_ARM_ANGLE+MAX_UP_ANGLE+0.2):
        assert vehicle_position(articulate(arm,angle),side).common(fixed_bracket).Volume>1e-5,'Stop ineffective '+side
    # Mirror the actual clearance/countersink probes as well as the arm, so
    # each countersink opens on its matching wheel-facing accessible side.
    nominal_arm=vehicle_position(articulate(arm,NOMINAL_ARM_ANGLE),side)
    for x,z in mount_centers:
        depth=(M3_COUNTERSINK_DIAMETER-ENCODER_MOUNT_HOLE_DIAMETER)/2
        probe=Part.makeCone(ENCODER_MOUNT_HOLE_DIAMETER/2-.01,M3_COUNTERSINK_DIAMETER/2-.01,depth,
              V(x,ENCODER_MOUNT_PLATE_THICKNESS-depth,z),Y)
        probe=vehicle_position(articulate(probe,NOMINAL_ARM_ANGLE),side)
        assert probe.common(nominal_arm).Volume<1e-7,'Mirrored countersink obstructed '+side
    for angle in (NOMINAL_ARM_ANGLE-MAX_DOWN_ANGLE,NOMINAL_ARM_ANGLE+MAX_UP_ANGLE):
        a=math.radians(angle)
        rx=SPRING_ARM_X*math.cos(a)-SPRING_ARM_Z*math.sin(a)
        rz=SPRING_ARM_X*math.sin(a)+SPRING_ARM_Z*math.cos(a)
        assert rx*(-SPRING_BRACKET_DROP-rz)-rz*(SPRING_BRACKET_X-rx)<0,'Spring torque would lift wheel'
    assert min(metrics['front_x'],metrics['rear_x'],metrics['cable_chassis'])>0
    validation[side]=metrics
# An exact reflection preserves the complete spring/stop/hole/cable geometry.
for shape in (arm,bracket,encoder,wheel,pin,cable):
    right=vehicle_position(shape,'RIGHT')
    reflected=mirror_y(right)
    left=vehicle_position(shape,'LEFT')
    assert reflected.cut(left).Volume+left.cut(reflected).Volume<1e-7
    assert abs(left.Volume-right.Volume)<1e-6
# Flanges remain separated by the central chassis, even for independent travel.
assert vehicle_position(bracket,'RIGHT').BoundBox.YMin>0
assert vehicle_position(bracket,'LEFT').BoundBox.YMax<0
fixed_vehicle_parts=[vehicle_position(bracket,'RIGHT'),vehicle_position(pin,'RIGHT')]
# Continuous conservative X envelopes: optimize rotated local bounding-box
# corners at interval endpoints and every interior trigonometric extremum.
# This proves longitudinal separation from +/-90 for the whole travel interval,
# independent of the unmeasured main-wheel width, height or curved outline.
angle_low=math.radians(NOMINAL_ARM_ANGLE-MAX_DOWN_ANGLE)
angle_high=math.radians(NOMINAL_ARM_ANGLE+MAX_UP_ANGLE)
x_envelopes=[]
for shape in (arm,encoder,wheel):
    bb=shape.BoundBox
    values=[]
    for x in (bb.XMin,bb.XMax):
        for z in (bb.ZMin,bb.ZMax):
            candidates=[angle_low,angle_high]
            for k in range(-2,3):
                extremum=math.atan2(-z,x)+k*math.pi
                if angle_low<=extremum<=angle_high:candidates.append(extremum)
            values.extend(x*math.cos(a)-z*math.sin(a) for a in candidates)
    x_envelopes.append((pivot_vehicle_x-max(values),pivot_vehicle_x-min(values)))
x_envelopes += [(s.BoundBox.XMin,s.BoundBox.XMax) for s in fixed_vehicle_parts]
continuous_front_bound=LONGITUDINAL_FREE_SPACE/2-max(b for a,b in x_envelopes)
continuous_rear_bound=min(a for a,b in x_envelopes)+LONGITUDINAL_FREE_SPACE/2
assert min(continuous_front_bound,continuous_rear_bound)>0,'Continuous travel envelope crosses wheel boundary'
# Both complete nominal installations on one shared vehicle reference.
doc=App.newDocument('EncoderMount')
doc.Label='Mirrored direct-drive mounts — selected '+MOUNT_SIDE
obj=doc.addObject('PartDesign::Feature','ChassisReference')
obj.Shape=chassis
if App.GuiUp:obj.ViewObject.ShapeColor=(0.65,0.65,0.68)
local_components=[('ChassisBracket',bracket,(0.2,0.45,0.85)),
    ('ArticulatedArm',articulate(arm,NOMINAL_ARM_ANGLE),(0.95,0.55,0.15)),
    ('Encoder',articulate(encoder,NOMINAL_ARM_ANGLE),(0.35,0.36,0.39)),
    ('OdometryWheel',articulate(wheel,NOMINAL_ARM_ANGLE),(0.12,0.12,0.14)),
    ('PivotPlaceholder',pin,(0.65,0.65,0.65))]
assemblies={}
for side in (MOUNT_SIDE,'LEFT' if MOUNT_SIDE=='RIGHT' else 'RIGHT'):
    suffix=side.title()
    group=doc.addObject('App::DocumentObjectGroup',suffix+'Assembly')
    group.Label=side+' installation'+(' (selected)' if side==MOUNT_SIDE else '')
    assemblies[side]=group
    for name,shape,color in local_components:
        obj=doc.addObject('PartDesign::Feature',name+suffix)
        obj.Shape=vehicle_position(shape,side)
        assert obj.Shape.isValid() and len(obj.Shape.Solids)==1
        group.addObject(obj)
        if App.GuiUp:obj.ViewObject.ShapeColor=color
selected_assembly=assemblies[MOUNT_SIDE]
refs=doc.addObject('App::DocumentObjectGroup','VehicleReferences')
for side in ('RIGHT','LEFT'):
    sign=1 if side=='RIGHT' else -1
    for label,shape in zip(('Front','Rear'),wheel_references[side]):
        obj=doc.addObject('PartDesign::Feature',label+'VehicleWheelReference'+side.title())
        obj.Shape=shape
        refs.addObject(obj)
        obj.addProperty('App::PropertyString','Note').Note='Reference prism only: 180 mm clear region and outer face 10 mm beyond this side edge are measured; other dimensions illustrative.'
        if App.GuiUp:
            obj.ViewObject.ShapeColor=(0.55,0.65,0.75)
            obj.ViewObject.Transparency=65
    edge=doc.addObject('PartDesign::Feature','ChassisEdge'+side.title())
    edge.Shape=Part.makeLine(V(-LONGITUDINAL_FREE_SPACE/2,sign*CHASSIS_HALF_WIDTH,chassis_top_z),
                            V(LONGITUDINAL_FREE_SPACE/2,sign*CHASSIS_HALF_WIDTH,chassis_top_z))
    refs.addObject(edge)
    if App.GuiUp:
        edge.ViewObject.LineColor=(0.05,0.65,0.25)
        edge.ViewObject.LineWidth=4.0
    obj=doc.addObject('PartDesign::Feature','CableClearanceReference'+side.title())
    obj.Shape=vehicle_position(articulate(cable,NOMINAL_ARM_ANGLE),side)
    refs.addObject(obj)
    if App.GuiUp:
        obj.ViewObject.ShapeColor=(0.85,0.2,0.2)
        obj.ViewObject.Transparency=75

auto_backup=App.ParamGet('User parameter:BaseApp/Preferences/Document')
previous_backups=auto_backup.GetInt('CountBackupFiles',1)
exports=[]
try:
    # Keep the requested directory free of FreeCAD backup files on regeneration.
    auto_backup.SetInt('CountBackupFiles',0)
    doc.recompute()
    if App.GuiUp:
        Gui.activeDocument().activeView().viewAxonometric()
        Gui.activeDocument().activeView().fitAll()
        view=Gui.activeDocument().activeView()
        for label,method in [('top','viewTop'),('side','viewFront'),('front','viewRight'),('isometric','viewAxonometric')]:
            getattr(view,method)()
            if label=='isometric':
                camera_z=V(1,1,0.8)  # look from the OUTBOARD side
                camera_x=V(0,0,1).cross(camera_z)
                camera_y=camera_z.cross(camera_x)
                view.setCameraOrientation(App.Rotation(camera_x,camera_y,camera_z,'ZXY').Q)
            view.fitAll()
            view.saveImage(str(OUTPUT_DIRECTORY/('vehicle_layout_'+label+'.png')),1600,1100,'White')
    path=OUTPUT_DIRECTORY/'encoder_mount.FCStd'
    doc.saveAs(str(path))
    exports.append(str(path))
finally:
    auto_backup.SetInt('CountBackupFiles',previous_backups)

stl_dir=OUTPUT_DIRECTORY/'stl'
stl_dir.mkdir(parents=True,exist_ok=True)
for side in ('RIGHT','LEFT'):
    for name,shape in [('chassis_bracket',bracket),('articulated_arm',arm)]:
        # Reflect the solid before tessellation; FreeCAD retains outward normals.
        printable=mirror_y(shape) if side=='LEFT' else shape.copy()
        if name=='chassis_bracket':angle=180
        else:angle=-90 if side=='LEFT' else 90
        # Both arms have the flat encoder-facing rear surface on the print bed.
        printable.rotate(V(0,0,0),V(1,0,0),angle)
        b=printable.BoundBox
        printable.translate(V(-b.XMin,-b.YMin,-b.ZMin))
        assert printable.isValid() and len(printable.Solids)==1
        mesh=MeshPart.meshFromShape(Shape=printable,LinearDeflection=MESH_LINEAR_DEFLECTION,
                                   AngularDeflection=MESH_ANGULAR_DEFLECTION,Relative=False)
        assert mesh.isSolid(),name+' '+side+' STL is not closed'
        path=stl_dir/(name+'_'+side.lower()+'.stl')
        mesh.write(str(path))
        exports.append(str(path))
# Superseded unsuffixed exports are replaced by the requested four variants.
for legacy_name in ('chassis_bracket.stl','articulated_arm.stl'):
    legacy=stl_dir/legacy_name
    if legacy.exists():legacy.unlink()
height=pivot_z+ARM_LENGTH*math.sin(math.radians(NOMINAL_ARM_ANGLE))
shaft_vehicle_x=pivot_vehicle_x-ARM_LENGTH*math.cos(math.radians(NOMINAL_ARM_ANGLE))
App.Console.PrintMessage(
    f'\nRIGHT and LEFT mounts generated; MOUNT_SIDE={MOUNT_SIDE} selects the default group.\n'
    f'Vehicle center plane Y=0; chassis edges +/-{CHASSIS_HALF_WIDTH:.2f} mm (reference half-width, verify).\n'
    f'LEFT is the exact Y -> -Y mirror of RIGHT. Local geometry, M3 patterns, countersinks, springs and stops preserved.\n'
    f'Both: pivot X={pivot_vehicle_x:.2f}, encoder X={shaft_vehicle_x:.2f}, center Z={height:.2f} mm; trailing direction unchanged.\n'
    f'Flange overlap {CHASSIS_FLANGE_OVERLAP:.2f} mm; M3 spacing {CHASSIS_HOLE_SPACING:.2f} mm; travel +{MAX_UP_ANGLE:.1f}/-{MAX_DOWN_ANGLE:.1f} deg.\n'
    f'Continuous longitudinal clearance bounds: front {continuous_front_bound:.3f}, rear {continuous_rear_bound:.3f} mm.\n')
for side in ('RIGHT','LEFT'):
    m=validation[side]
    App.Console.PrintMessage(
        f'{side}: {TRAVEL_CHECK_POSITIONS} travel positions passed; ground contact, chassis/wheel/body/cable clearance, flange contact, M3 alignment and effective stops verified.\n'
        f"  Front/rear reference minimum: {m['front']:.3f}/{m['rear']:.3f} mm; encoder/chassis {m['encoder_chassis']:.3f}; cable/chassis {m['cable_chassis']:.3f} mm.\n")
App.Console.PrintMessage('Files exported:\n'+'\n'.join(exports)+'\n')

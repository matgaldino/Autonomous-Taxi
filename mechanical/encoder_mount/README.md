# Concept

A simple direct-drive odometry mount on each side: chassis → one-piece L-bracket → independent M4 pivot → trailing arm → flange-mounted encoder → wheel on the encoder's 6 mm shaft. The wheel carries only a light odometry contact load, not vehicle weight.

`MOUNT_SIDE = "RIGHT"` or `"LEFT"` selects the default/primary installation in the script and document label. **Both assemblies are always generated and visible** as `RightAssembly` and `LeftAssembly`, and all four printable variants are always exported. The LEFT assembly is the exact geometric reflection of RIGHT through the vehicle longitudinal X-Z plane: `(X,Y,Z) → (X,−Y,Z)`. No additional rotation reverses the trailing direction. Pivot, spring holes, stops, flange overlap, encoder holes and countersinks are mirrored together.

The vehicle reference now uses its **centerline at Y=0**, with +X forward, +Y designated RIGHT and Z upward. `CHASSIS_HALF_WIDTH = 80` mm is an **unmeasured reference assumption**, giving a 160 mm wide plate and edges at Y=±80. This recenters the previous edge-relative layout: every installation retains the same offset from its nearest edge. Encoder bodies and rear cable exits point outward; the odometry wheels are closer to the edges but entirely outside the plate.

The same shared chassis has matching 13 mm-spaced M3 hole pairs on both sides. Each bracket flange rests on top of the plate and overlaps it by 20 mm. Each vertical wall remains outside its side edge. Four vehicle-wheel reference prisms show the 180 mm longitudinal free region and main-wheel outer envelopes 10 mm beyond each edge; their other dimensions are illustrative, not measured tires.

[Top-view comparison of both installations](vehicle_layout_top.png). The FreeCAD document also contains both complete assemblies for visual comparison; either group can be hidden independently. The local encoder hole diagram remains `encoder_mount_front_view.png`.

# Dimensions

| Feature | Both sides |
| --- | --- |
| Encoder | E38S6G5-600B-G24N; Ø38 × 35 mm body, 5 mm flange |
| Shaft / boss | Ø6 × approximately 12 mm exposed shaft; Ø20 mm boss |
| Encoder mounting | A: Ø28 PCD at 30°/150°/270°; B: Ø30 PCD at 0°/120°/240°, mirrored with each mount |
| Mount clearance | Ø3.4 mm holes, Ø6.4 mm flush 90° countersinks; Ø20.4 boss opening |
| Wheel | Ø61 mm, 6 mm disk, Ø6 bore; Ø20 × 9 mm inward-facing hub |
| Arm | 70 mm pivot-to-shaft; 5 mm thick, locally 3 mm at encoder flange |
| Bracket | Single printable L shape, 5 mm flange and 5 mm wall |
| Chassis overlap | **20 mm on each side** |
| Chassis mounting | Two Ø3.4 mm holes per side, 13 mm apart; row 10 mm inboard of edge, provisional |
| Chassis thickness | `CHASSIS_PLATE_THICKNESS=3` mm, provisional; underside Z=76, top Z=79 |
| Pivot | Independent M4, Ø4.4 mm holes; X=+55, Z=56; Y=±102 with default half-width |
| Nominal encoder axis | X=−10.19, Z=30.5; front mounting faces Y=±102 |
| Nominal angle / travel | −21.36°; +12° upward / −3° downward from nominal |
| Vertical wheel travel | Approximately 14.11 mm rise / 3.38 mm droop |
| Hub clearance / engagement | 0.6 mm to plate, 3.6 mm to encoder face; 8.4 mm shaft engagement |
| Maximum protrusion from each edge | 62 mm hardware; 82 mm including assumed cable keep-out |
| Front/rear wheel-reference clearance | 5.00 / 45.43 mm minimum at 31 checked positions, identical both sides |

Continuous conservative longitudinal envelopes also remain at least 5.00 mm clear in front and 39.97 mm behind throughout travel. Mirroring leaves X and Z unchanged, including nominal ground contact. The center chassis separates the two mounts even with independent arm travel.

# Assembly instructions

1. Select the matching side pair: `stl/chassis_bracket_right.stl` + `stl/articulated_arm_right.stl`, or `stl/chassis_bracket_left.stl` + `stl/articulated_arm_left.stl`. The old unsuffixed STLs are superseded. Both brackets print upside down on the broad outside flange face; both arms are oriented with their flat encoder-facing rear surface on the bed. Short stop posts and horizontal holes may need localized support/cleanup.
2. Verify plate thickness and the actual chassis hole-row offset. Place the appropriate flange on the chassis top, with 20 mm overlap and its two M3 holes aligned. Use M3 fasteners long enough for the 5 mm flange, actual plate, washers and nuts.
3. Fit the arm using an M4 smooth-shank pivot and locking nut with appropriate washers. Retain free articulation and the 0.3 mm wall clearance; select actual bolt length for both parts and fasteners. The modeled pin is a placeholder.
4. Attach the encoder by its flange using six flush 90° flat-head M3 screws of suitable length for the actual thread depth. LEFT is mirrored as a complete solid, including the countersinks, so their openings stay on the corresponding wheel-facing side. The simplified encoder is mirrored too; verify the real encoder's flange orientation against the LEFT hole pattern before printing.
5. Fit the wheel directly on the shaft with its hub toward the encoder, preserving the 0.6 mm plate gap. Confirm the existing wheel's retention works with 8.4 mm engagement; detailed retention hardware is not modeled.
6. Check free wheel rotation, stops and ground contact. Leave a cable loop exiting outward: toward +Y on RIGHT and −Y on LEFT. A future light tension spring can use the mirrored bracket/arm holes; no spring is modeled. Set only light contact force.

To regenerate in FreeCAD 1.1 or later, edit the parameters at the beginning of `encoder_mount.py`, then run:

```python
p = '/home/matheus/Projects/Autonomous-Taxi/mechanical/encoder_mount/encoder_mount.py'
exec(compile(open(p).read(), p, 'exec'), {'__file__': p})
```

The script saves both assemblies in `encoder_mount.FCStd` and exports all four side-specific STLs. GUI execution refreshes the top, side, front and isometric CAD views. The supplied top comparison is an annotated projection of the actual CAD triangles; GUI regeneration produces its plain CAD counterpart. No additional mechanical components are introduced.

The same validation runs for **31 travel positions on EACH side**, including chassis/vehicle-wheel and moving-part interference, outward body/wheel/cable placement, ground contact, flange contact, coaxial M3 mounting holes, effective end stops and downward spring geometry. It also verifies exact LEFT/RIGHT reflection, correct countersink openings, single valid solids and closed STL meshes. Geometric checks are not stress or physical fit tests.

# Parameters requiring physical verification

- `CHASSIS_HALF_WIDTH` (80 mm assumed), `CHASSIS_PLATE_THICKNESS` (3 mm assumed), and `CHASSIS_MOUNT_HOLE_EDGE_DISTANCE` (10 mm assumed). Only the 13 mm hole spacing is confirmed.
- The 180 mm region through vehicle steering/suspension travel, the 10 mm wheel-envelope offset, and actual main-wheel outlines. The reference prisms are not measured tires.
- Encoder shaft datum/projection, body/flange length convention, boss projection, usable M3 thread depth, and the actual flange pattern orientation on both installations. A true mirrored encoder reference is not a substitute for checking the purchased component.
- Printed bore fit and wheel retention on the available shaft engagement; screw-head seating and M4 bolt/washer/nut clearance.
- Cable exit and bend radius. The assumed rear-axial Ø10 × 20 mm keep-out is outward on each side, at Y=142–162 mm RIGHT and −162 to −142 mm LEFT with the default half-width.
- Actual wheel contact force and encoder bearing capacity. Keep spring preload light; do not transfer vehicle weight through the encoder shaft.

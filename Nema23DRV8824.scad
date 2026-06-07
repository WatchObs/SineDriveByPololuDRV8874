//////////////////////////////////////////////////////////////
// Servo57D Rear Mount Plate + Snap-On Cover (offset)
// DRV8874 based driver board replaces Makerbase driver
// Literal T-cutout + 4  holes + recess for board slide-up
//////////////////////////////////////////////////////////////

$fn = 180;

DoCover = 1;
DoBase  = 0;

// ------------------------------------------------------------
// Motor rear pattern
// ------------------------------------------------------------
bolt_spacing_x   = 31.7;
bolt_spacing_y   = 51.0;
bolt_hole_dia    = 3.5;
nema23_spacing   = 47.14;

// ------------------------------------------------------------
// Base plate geometry
// ------------------------------------------------------------
plate_w          = 56;
plate_h          = 90;
plate_thickness  = 6;

// ------------------------------------------------------------
// Snap-on cover groove
// ------------------------------------------------------------
groove_offset    = 1.5;
groove_height    = 1.5;
groove_depth     = 1.0;

groove_top       = plate_thickness - groove_offset;
groove_bottom    = groove_top - groove_height;

// ------------------------------------------------------------
// Literal T-cutout dimensions
// ------------------------------------------------------------
stem_w = 11;
stem_h = 17;

T_top_w = 24;
T_top_h = 12;

// Side region width (tabs)
side_w = (T_top_w - stem_w) / 2;

// PCB hole spacing
m1_d = 1.8;
hole_dy = 11/2;

// Recess depth for board slide-up
recess_depth = 4.33;

// ------------------------------------------------------------
// mainPCBboard standoffs
// ------------------------------------------------------------
mainPCB_w           = nema23_spacing;
mainPCB_h           = nema23_spacing;
mainPCB_standoff_h  = 9.24;
mainPCB_hole_offset = 0;
mainPCBWall         = 1.6;

post_dia         = 8.0;
post_hole_dia    = 2.8;

// ------------------------------------------------------------
// Cover geometry
// ------------------------------------------------------------
coverHeight = 18+mainPCBWall;
cover_internal_h = 25;
cover_wall       = 2.0;
cover_clearance  = 0.3;
cover_narrow     = 1.0;

rib_height       = groove_height;
rib_depth        = groove_depth * 0.9;

cover_offset_x   = plate_w + 20;

//////////////////////////////////////////////////////////////
// Literal T-shaped through-cut
//////////////////////////////////////////////////////////////
module encoder_T_cutout() {

    translate([-stem_w/2, -stem_h/2, -0.1])
        cube([stem_w, stem_h, plate_thickness + 10]);

    translate([-T_top_w/2, stem_h/2, -0.1])
        cube([T_top_w, T_top_h, plate_thickness + 10]);
}

//////////////////////////////////////////////////////////////
// PCB holes
//////////////////////////////////////////////////////////////
module pcb_bolt_holes() {

    x_left  = -(stem_w/2 + side_w/2);
    x_right =  (stem_w/2 + side_w/2);

    for (x = [x_left, x_right])
    for (y = [-hole_dy, hole_dy])
        translate([x, y, -0.1])
            cylinder(d=m1_d, h=plate_thickness + 10);
}

//////////////////////////////////////////////////////////////
// Recess under side regions
//////////////////////////////////////////////////////////////
module encoder_recess() {

    translate([-(stem_w/2 + side_w), -stem_h/2, 0])
        cube([side_w, stem_h, recess_depth]);

    translate([stem_w/2, -stem_h/2, 0])
        cube([side_w, stem_h, recess_depth]);
}

//////////////////////////////////////////////////////////////
// Triangular groove profile (horizontal wedge)
//////////////////////////////////////////////////////////////
module tri_xz() {
    polygon(points = [
        [0, 0],                 // outer bottom
        [groove_depth, groove_height/2],      // inner bottom
        [0, groove_height]      // vertex inward
    ]);
}

//////////////////////////////////////////////////////////////
// Base plate
//////////////////////////////////////////////////////////////
module base_plate() {
    difference() {

        // Plate + your tab thickener block
        union() {
            translate([0,-1,plate_thickness])
                cube([T_top_w+4, stem_h+2, 4],center=true);

            translate([-plate_w/2, -plate_w/2, 0])
                cube([plate_w, plate_h, plate_thickness]);

            // Wall
            translate([0, plate_h/2-plate_w/2, plate_thickness+mainPCB_standoff_h/2])
            difference()
            {
              cube([plate_w,plate_h,mainPCB_standoff_h],center=true);
              cube([plate_w-mainPCBWall*2,plate_h-mainPCBWall*2,mainPCB_standoff_h],center=true);
            }
        }

        // Motor bolt holes
        for (x = [-bolt_spacing_x/2, bolt_spacing_x/2])
        for (y = [-bolt_spacing_y/2, bolt_spacing_y/2])
            translate([x, y, -0.1])
                cylinder(d=bolt_hole_dia, h=plate_thickness + 10);

        // Literal T-cutout
        encoder_T_cutout();

        // PCB holes
        pcb_bolt_holes();

        // Recess
        encoder_recess();

        // ------------------------------------------------------------
        // Triangular perimeter grooves (correctly rotated)
        // ------------------------------------------------------------

//        // LEFT side (extrude along Y)
//        translate([-plate_w/2, -plate_h/2, groove_bottom])
//            rotate([-90,0,0])
//            linear_extrude(height = plate_h)
//                tri_xz();

//        // RIGHT side (mirror X)
//        translate([plate_w/2, -plate_h/2, groove_bottom])
//            rotate([-90,0,0])
//            mirror([1,0,0])
//                linear_extrude(height = plate_h)
//                    tri_xz();

//        // BOTTOM side (rotate 90°)
//        translate([plate_w/2, -plate_h/2, groove_bottom])
//            rotate([-90,0,90])
//                linear_extrude(height = plate_w)
//                    tri_xz();

//        // TOP side (rotate -90°)
//        translate([-plate_w/2, plate_h/2, groove_bottom])
//            rotate([-90,0,-90])
//                linear_extrude(height = plate_w)
//                    tri_xz();
    }
}

//////////////////////////////////////////////////////////////
// mainPCBboard standoffs
//////////////////////////////////////////////////////////////
module mainPCBboard_standoffs() {

    for (x = [-mainPCB_w/2 + mainPCB_hole_offset, mainPCB_w/2 - mainPCB_hole_offset])
    for (y = [-mainPCB_h/2 + mainPCB_hole_offset, mainPCB_h/2 - mainPCB_hole_offset]) {

        difference() {
            translate([x, y, plate_thickness])
                cylinder(d=post_dia, h=mainPCB_standoff_h, $fn=32);

            translate([x, y, -100])
                cylinder(d=post_hole_dia, h=200, $fn=32);
        }
    }
    
}

//////////////////////////////////////////////////////////////
// Cover (unchanged)
//////////////////////////////////////////////////////////////
module rib_xz() {
    polygon(points = [
        [0, 0],
        [rib_depth, 0],
        [0, rib_height]
    ]);
}

//////////////////////////////////////////////////////////////
// Cover ribs: same isosceles profile + matching rotations
//////////////////////////////////////////////////////////////
module rib_xz() {
    polygon(points = [
        [0, 0],
        [rib_depth, rib_height/2],
        [0, rib_height]
    ]);
}

//////////////////////////////////////////////////////////////
// Cover ribs: same isosceles profile + correct rotations
//////////////////////////////////////////////////////////////
module rib_xz() {
    polygon(points = [
        [0, 0],
        [rib_depth, rib_height/2],
        [0, rib_height]
    ]);
}

module cover() {
postDiaL = 7.0;
postDiaU = 8.8;
postDiaIL = 3.5;
postDiaIU = 6.5;
postDepthU=coverHeight-10;


    difference() {

        // Plate 
        union() 
        {

         translate([-plate_w/2, -plate_w/2, 0]) cube([plate_w, plate_h, mainPCBWall]);

         // Wall
         translate([0, plate_h/2-plate_w/2, coverHeight/2+mainPCBWall/2])
         difference()
         {
           cube([plate_w,plate_h,coverHeight],center=true);
           cube([plate_w-mainPCBWall*2,plate_h-mainPCBWall*2,coverHeight*1.1],center=true);
         }

         // Retainer bolts posts & well (body)
         for (x = [-mainPCB_w/2 + mainPCB_hole_offset, mainPCB_w/2 - mainPCB_hole_offset])
           for (y = [-mainPCB_h/2 + mainPCB_hole_offset, mainPCB_h/2 - mainPCB_hole_offset]) 
           {
             translate([x, y, mainPCBWall/2]) cylinder(d=postDiaL, h=coverHeight,   $fn=32);
             translate([x, y, mainPCBWall/2]) cylinder(d=postDiaU, h=postDepthU, $fn=32);
           }
         // Host connector post (strength)
         translate([plate_w/2-mainPCBWall-2,-plate_w/2+26-1.5,mainPCBWall/2]) cube([2,1.5,coverHeight]);
       }

       // Retainer bolts posts & well (holes)
       for (x = [-mainPCB_w/2 + mainPCB_hole_offset, mainPCB_w/2 - mainPCB_hole_offset])
         for (y = [-mainPCB_h/2 + mainPCB_hole_offset, mainPCB_h/2 - mainPCB_hole_offset]) 
         {
           translate([x, y, -5]) cylinder(d=postDiaIL, h=30, $fn=32);
          #translate([x, y, 0])  cylinder(d=postDiaIU, h=postDepthU-2, $fn=32);
         }
        // Motor connector
        translate([plate_w/2-10,-plate_w/2+11.5,coverHeight+mainPCBWall/2-9+.1])
         cube([20,13,9]);
        // Host connector
        translate([plate_w/2-10,-plate_w/2+26,coverHeight+mainPCBWall/2-3+.1])
         cube([20,21,3]);
        // USB connector
        translate([plate_w/2-10,-plate_w/2+68.5+10/2,coverHeight+mainPCBWall/2-5/2-13.7+.1])
         cube([20,10,5]);
        // Motor bolt head clearance
        translate([-bolt_spacing_x/2, -bolt_spacing_y/2, (coverHeight+mainPCBWall/2-3)]) cylinder(d=7, h=3);
        translate([ bolt_spacing_x/2, -bolt_spacing_y/2, (coverHeight+mainPCBWall/2-3)]) cylinder(d=7, h=3);
     } // Difference
    }

module cover2() {

    difference() {

        // Outer shell
        translate([cover_offset_x - plate_w/2 - cover_clearance - cover_wall,
                   -plate_h/2 - cover_clearance - cover_wall,
                   0])
            cube([plate_w + 2*(cover_clearance + cover_wall),
                  plate_h + 2*(cover_clearance + cover_wall),
                  cover_internal_h]);

        // Hollow interior
        translate([cover_offset_x - plate_w/2 - cover_clearance,
                   -plate_h/2 - cover_clearance,
                   -cover_narrow])
            cube([plate_w + 2*cover_clearance,
                  plate_h + 2*cover_clearance,
                  cover_internal_h]);
    }

if (0)
{
    // ------------------------------------------------------------
    // RIBS — matching plate groove rotations AND correct positions
    // ------------------------------------------------------------

    // LEFT rib (inside left wall)
    translate([
        cover_offset_x - plate_w/2 - cover_clearance,
        -plate_h/2 - cover_clearance,
        groove_bottom
    ])
        rotate([-90,0,0])
        linear_extrude(height = plate_h + 2*cover_clearance)
            rib_xz();

    // RIGHT rib (inside right wall)
    translate([
        cover_offset_x + plate_w/2 + cover_clearance,
        -plate_h/2 - cover_clearance,
        groove_bottom
    ])
        rotate([-90,0,0])
        mirror([1,0,0])
        linear_extrude(height = plate_h + 2*cover_clearance)
            rib_xz();

    // BOTTOM rib (inside bottom wall)
    translate([
        cover_offset_x + plate_w/2 + cover_clearance,
        -plate_h/2 - cover_clearance,
        groove_bottom
    ])
        rotate([-90,0,90])
        linear_extrude(height = plate_w + 2*cover_clearance)
            rib_xz();

    // TOP rib (inside top wall)
    translate([
        cover_offset_x - plate_w/2 - cover_clearance,
        plate_h/2 + cover_clearance,
        groove_bottom
    ])
        rotate([-90,0,-90])
        linear_extrude(height = plate_w + 2*cover_clearance)
            rib_xz();
}
}

//////////////////////////////////////////////////////////////
// Final assembly
//////////////////////////////////////////////////////////////
union() {
    if (DoBase)
    {
      base_plate();
      mainPCBboard_standoffs();
    }

    if (DoCover) 
    {
      translate([100,0,0]) cover();
    }
}
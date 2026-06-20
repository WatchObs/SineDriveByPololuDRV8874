//////////////////////////////////////////////////////////////
// Standalone Base + cover 
// DRV8874 based driver - AS5074D or ABI glass encoder
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
// mainPCBboard standoffs
// ------------------------------------------------------------
mainPCB_w           = nema23_spacing;
mainPCB_h           = nema23_spacing;
mainPCB_standoff_h  = 5; //9.24;
mainPCB_hole_offset = 0;
mainPCBWall         = 1.6;
mainPCBLip          = 0;

post_dia         = 8.0;
post_hole_dia    = 2.8;

// ------------------------------------------------------------
// Base plate geometry
// ------------------------------------------------------------
plate_w          = 56+2*mainPCBWall+1;
plate_h          = 90+2*mainPCBWall+1;
plate_thickness  = 4;

// ------------------------------------------------------------
// Cover geometry
// ------------------------------------------------------------
coverHeight      = 18 + mainPCBWall;
cover_internal_h = 25;
cover_wall       = 2.0;
cover_clearance  = 0.3;
cover_narrow     = 1.0;
cover_offset_x   = plate_w + 20;

//////////////////////////////////////////////////////////////
// Base plate
//////////////////////////////////////////////////////////////
module base_plate() 
{
 difference() 
 {
  // Plate + your tab thickener block
  union() 
  {
   translate([-plate_w/2, -plate_w/2, 0])
    cube([plate_w, plate_h, plate_thickness]);

   // Wall
   translate([0, plate_h/2-plate_w/2, plate_thickness+mainPCB_standoff_h/2])
    difference()
    {
      cube([plate_w,plate_h,mainPCB_standoff_h+mainPCBLip],center=true);
      cube([plate_w-mainPCBWall*2,plate_h-mainPCBWall*2,mainPCB_standoff_h+mainPCBLip],center=true);
    }
  }
 }
  // Mounting tabs
 TabW  = 10;
 TabL  = 50;
 BaseW = plate_w+2*TabW;
 BaseH = plate_h;
 translate([-BaseW/2, plate_h/2-plate_w/2-TabL/2, 0])
  difference()
  {
   cube([BaseW, TabL, plate_thickness]);
   for (y = [-plate_w/4, plate_w/4])
    for (x = [-(BaseW-TabW)/2, (BaseW-TabW)/2])
    {
     translate([x+BaseW/2,TabL/2+y,-5])
      cylinder(h=10,d=5);
    }
  }
}

//////////////////////////////////////////////////////////////
// mainPCBboard standoffs
//////////////////////////////////////////////////////////////
module mainPCBboard_standoffs(drill = 0) 
{
 for (x = [-mainPCB_w/2 + mainPCB_hole_offset, mainPCB_w/2 - mainPCB_hole_offset])
  for (y = [-mainPCB_h/2 + mainPCB_hole_offset, mainPCB_h/2 - mainPCB_hole_offset])
  {
   if (drill)
   {
    translate([x, y, -100])            cylinder(d=post_hole_dia, h=200, $fn=32);
   }
   else
   {
    difference() 
    {
     translate([x, y, plate_thickness]) cylinder(d=post_dia, h=mainPCB_standoff_h, $fn=32);
     translate([x, y, -100])            cylinder(d=post_hole_dia, h=200, $fn=32);
    }
   }
  }
}

module cover() 
{
 postDiaL = 7.0;
 postDiaU = 8.8;
 postDiaIL = 3.5;
 postDiaIU = 6.5;
 postDepthU=coverHeight-10;

 difference() 
 {
  // Plate 
  union() 
  {
   translate([-plate_w/2, -plate_w/2, 0]) cube([plate_w, plate_h, mainPCBWall]);
   
   // Wall
   translate([0, plate_h/2-plate_w/2, coverHeight/2+mainPCBWall-mainPCBLip/2])
   difference()
   {
    cube([plate_w,plate_h,coverHeight-mainPCBLip],center=true);
    cube([plate_w-mainPCBWall*2,plate_h-mainPCBWall*2,coverHeight*1.1-mainPCBLip],center=true);
   }

   // Retainer bolts posts & well (body)
   for (x = [-mainPCB_w/2 + mainPCB_hole_offset, mainPCB_w/2 - mainPCB_hole_offset])
    for (y = [-mainPCB_h/2 + mainPCB_hole_offset, mainPCB_h/2 - mainPCB_hole_offset]) 
    {
     translate([x, y, mainPCBWall/2]) cylinder(d=postDiaL, h=coverHeight);
     translate([x, y, mainPCBWall/2]) cylinder(d=postDiaU, h=postDepthU, $fn=32);
    }
    // Host connector post (strength)
//    translate([plate_w/2-mainPCBWall-2,-plate_w/2+29-0.75,mainPCBWall/2]) 
//     cube([2,1.5,coverHeight-mainPCBLip/2-.2]);
  }

  // Retainer bolts posts & well (holes)
  for (x = [-mainPCB_w/2 + mainPCB_hole_offset, mainPCB_w/2 - mainPCB_hole_offset])
   for (y = [-mainPCB_h/2 + mainPCB_hole_offset, mainPCB_h/2 - mainPCB_hole_offset]) 
   {
    translate([x, y, -5]) cylinder(d=postDiaIL, h=30, $fn=32);
    translate([x, y, 0])  cylinder(d=postDiaIU, h=postDepthU-2, $fn=32);
   }
   translate([0,(2*mainPCBWall-.5),-mainPCBLip/2])
   {
    // Motor connector
    translate([plate_w/2-4,-plate_w/2+11.5,coverHeight+mainPCBWall/2-9+.1])
     cube([6,13,12]);
    // Host connector
    translate([plate_w/2-4,-plate_w/2+26,coverHeight+mainPCBWall/2-6+.1])
     cube([6,21,8]);
    // Encoder connector
    translate([plate_w/2-4,-plate_w/2+26+6+21.5,coverHeight+mainPCBWall/2-6+.1])
     cube([6,16,8]);
    // USB connector
    translate([plate_w/2-4,-plate_w/2+67.5+10/2,coverHeight+mainPCBWall/2-5/2-14.7+.1])
     cube([6,12,7]);
   }
 } // Difference
}

//////////////////////////////////////////////////////////////
// Final assembly
//////////////////////////////////////////////////////////////
union()
{
 if (DoBase)
 {
  difference()
  {
   base_plate();
   mainPCBboard_standoffs(1);
  }
  mainPCBboard_standoffs(0);
 }

 if (DoCover) 
 {
  translate([100,0,0]) cover();
 }
}
include <BOSL/constants.scad>
use <BOSL/beziers.scad>

b = 300;
bm =110;
l = 300;

wheel_d = 120;
wheel_z = +20;
wheel_x = 200;
wheed_dist=100;

wheelbox_d = wheel_d+40;
wheelbox_z = wheel_z;
wheelbox_x = 200;
wheedbox_dist=wheed_dist-10;


path = [ [0, 0], [150, (b-bm)/2], [300, (b-bm)/2], [320, 0] ];
splinesteps=200;
$fn=200;

difference()
{
    union()
    {
        rotate([0,0,0]) scale([1,1,1]) union()
        {
            translate([0,bm/2,0]) revolve_bezier_solid_to_axis(path, angle=90,splinesteps=splinesteps);
            translate([0,bm/2,0]) rotate([90,0,0]) linear_extrude_bezier(path, height=bm,splinesteps=splinesteps);
            translate([0,-bm/2,0]) rotate([0,0,180]) mirror() revolve_bezier_solid_to_axis(path, angle=90,splinesteps=splinesteps);
        }
        rotate([180,0,0]) scale([1,1,0.3]) union()
        {
            translate([0,bm/2,0]) revolve_bezier_solid_to_axis(path, angle=90,splinesteps=splinesteps);
            translate([0,bm/2,0]) rotate([90,0,0]) linear_extrude_bezier(path, height=bm,splinesteps=splinesteps);
            translate([0,-bm/2,0]) rotate([0,0,180]) mirror() revolve_bezier_solid_to_axis(path, angle=90,splinesteps=splinesteps);
        }
    }
    union()
    {
        translate([wheelbox_x,wheedbox_dist,wheel_z]) rotate([0,90,90]) linear_extrude(1000) circle(d=wheelbox_d);
        translate([wheelbox_x,-wheedbox_dist,wheel_z]) rotate([0,90,-90]) linear_extrude(1000) circle(d=wheelbox_d);
        translate([200,0,0]) rotate([0,180,0]) linear_extrude(100) circle(d=190);
        translate([50,0,0]) rotate([0,180,0]) linear_extrude(100) circle(d=50);
    }
}

        translate([wheel_x,wheed_dist,wheel_z]) rotate([0,90,90]) linear_extrude(30) circle(d=wheel_d);
        translate([wheel_x,-wheed_dist,wheel_z]) rotate([0,90,-90]) linear_extrude(30) circle(d=wheel_d);

   translate([305,0,20]) rotate([90-24,0,90]) linear_extrude(15) text("RoboHeci", halign="center", valign="center", size=20, font = "Liberation Sans:style=Bold Italic");

$fn=100;
a1=61.65;
b1=18.7;
b2=20.35;
a2=9;
r=2;
a3=84;
b3=25;
h=12;
R=8.5;
difference() {
    linear_extrude(r) square([a3,b3]);
    translate([(a3-9*R)/2,0,0]) union()
    {
      translate([0*R,(b3)/2-1*R,0]) linear_extrude(10) circle(d=4);
      translate([9*R,(b3)/2-1*R,0]) linear_extrude(10) circle(d=4);
      translate([0*R,(b3)/2-0*R,0]) linear_extrude(10) circle(d=4);
      translate([9*R,(b3)/2-0*R,0]) linear_extrude(10) circle(d=4);
      translate([0*R,(b3)/2+1*R,0]) linear_extrude(10) circle(d=4);
      translate([9*R,(b3)/2+1*R,0]) linear_extrude(10) circle(d=4);
    }
}
translate([(a3-a1-2*r)/2,(b3-b1-2*r)/2,0]) difference(){
    linear_extrude(h+r) square([a1+2*r,b1+2*r]);
    translate([r,r,r]) linear_extrude(h) square([a1,b1]);
    translate([r,r-(b2-b1)/2,r]) linear_extrude(h) square([a2,b2]);
}


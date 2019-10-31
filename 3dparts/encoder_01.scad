$fn=200;
d1=36.85;
d2=21;
d3=5;
d4=34;
d5=38.2;
h1=8;
h2=7.5;
h3=1.5;
h4=1.5;
k=4;

difference(){
  translate([0,0,-k]) linear_extrude(h1+k) circle(d=d5+2);
  translate([0,0,-k]) linear_extrude(k) circle(d=d5);
  difference(){
    translate([0,0,-k]) linear_extrude(k+1.5) circle(d=d5);
    translate([0,0,-k]) linear_extrude(k+1.5) circle(d=d1-0.1);
  }
  linear_extrude(h2) circle(d=d2);
  linear_extrude(h4) circle(d=d4);
  translate([0,-d1/2,-10]) linear_extrude(1.5+10) square([8,8],true);
  rotate([0,0,180]) union() {
    translate([0,d2/2+1.6/2+0.4,h4+0.2]) linear_extrude(h1) square([4.5,1.6],true);
    translate([0,d2/2+10/2+5,h1-2]) linear_extrude(2) square([4.2,10],true);
    translate([-1.5,d2/2+10/2,h1-2]) linear_extrude(2) square([1,10],true);
    translate([0,d2/2+10/2,h1-2]) linear_extrude(2) square([1,10],true);
    translate([+1.5,d2/2+10/2,h1-2]) linear_extrude(2) square([1,10],true);
  }
}
translate([0,0,h1-h3]) linear_extrude(h3) circle(d=d3);

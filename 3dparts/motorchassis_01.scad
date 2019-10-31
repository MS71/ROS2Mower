show_motor=1;
show_mirror=1;

$fn=100;
b=244;
t=169;
maehmotor_ausschnitt=50;
t_achse=94;
h_achse=24.3-3.2-3.5;
d_achse=20;
d_achsen_schrauben=8.5;
d_achsen_schrauben_abstand=90;
d_corner=25;
//h=68-3.5;
h=h_achse+d_achse/2+5;
d1=2;
d2=5;
d3=2;

// motor
d5_motor=45;
d1_motor=38.2;
d2_motor=12.3;
d3_motor=6;
d4_motor=d1_motor+2*d3;
l1_motor=76;
l2_motor=5.6;
l3_motor=50;
l5_motor=60;
x_motor=d1_motor-(31.4-d2_motor/2)-d2_motor/2;
a_motor=23;
b1_motor=3;
b2_motor=4;
d5=2.5; // motor kabelbinderloch

bat_b=150+1;
bat_t1=65+1;
//bat_t2=8;
bat_t2=t-((t-t_achse)+d4_motor/2+bat_t1+2);

module motor() {
    translate([0,0,x_motor]){
        translate([a_motor,t-t_achse,h_achse]) rotate([0,90,0]) linear_extrude(l1_motor) circle(d=d1_motor);  
        translate([a_motor+l5_motor,t-t_achse,h_achse]) rotate([0,90,0]) linear_extrude(l1_motor-l5_motor) circle(d=d5_motor);  
        translate([a_motor-l2_motor,t-t_achse,h_achse-x_motor]) rotate([0,90,0]) linear_extrude(l2_motor) circle(d=d2_motor);  
        translate([a_motor-l2_motor-l3_motor,t-t_achse,h_achse-x_motor]) rotate([0,90,0]) linear_extrude(l3_motor) circle(d=d3_motor);  
        for(r=[60:360/3:360]) {
           translate([a_motor-10,t-t_achse,h_achse]) rotate([r/2,0,0]) translate([0,0,31/2]) rotate([0,90,0]) linear_extrude(10) circle(d=4);  
           translate([a_motor-10,t-t_achse,h_achse]) rotate([-r/2,0,0]) translate([0,0,31/2]) rotate([0,90,0]) linear_extrude(10) circle(d=4);  
        }
    }
}

module drive_motor_holder() {
  difference() {
    union() {
        linear_extrude(d1) polygon([
          [0,0],[0,t],
          [(b-bat_b)/2+bat_t2,t],
          [(b-bat_b)/2+bat_t2,t-bat_t2],
          [(b-bat_b)/2,t-bat_t2],
          [(b-bat_b)/2,t-bat_t1-bat_t2],
          [b/2,t-bat_t1-bat_t2],
          [b/2,maehmotor_ausschnitt],
          [b/2-maehmotor_ausschnitt,maehmotor_ausschnitt],
          [b/2-maehmotor_ausschnitt,0],
          ]);
        linear_extrude(h) square([d2,t-d_corner/2]);
        translate([a_motor-b2_motor,t-t_achse-d4_motor/2,0]) linear_extrude(h_achse+x_motor+b1_motor) square([l1_motor+1*b2_motor,d4_motor]);
        translate([0,t-t_achse-d4_motor/2,0]) linear_extrude(5) square([b/2,d4_motor]);
        linear_extrude(20) square([b/2-maehmotor_ausschnitt,d2]);

        translate([d2,t-t_achse-d_achsen_schrauben_abstand/2+10/2+15,0]) rotate([90,0,0]) linear_extrude(10) polygon([
          [0,0],
          [2*h/4,0],
          [0,2*h/4]
        ]);
        translate([d2,t-t_achse+d_achsen_schrauben_abstand/2+10/2-15,0]) rotate([90,0,0]) linear_extrude(10) polygon([
          [0,0],
          [2*h/4,0],
          [0,2*h/4]
        ]);

        // rigel für mähmotorplatte
        translate([b/2-7.5,maehmotor_ausschnitt,0]) linear_extrude(15) square([7.5,10]);

        // bat holder
        translate([b/2-bat_b/2-30,t-10,0]) rotate([90,0,0]) linear_extrude(10) polygon([
          [0,0],
          [29,60],
          [30,0]
        ]);
        translate([b/2-bat_b/2-30,t-59,0]) rotate([90,0,0]) linear_extrude(10) polygon([
          [0,0],
          [29,60],
          [30,0]
        ]);
        translate([b/2-5,t-bat_t1-bat_t2-30,0]) difference(){
            rotate([90,0,90]) linear_extrude(5) polygon([
              [0,0],
              [29,60],
              [30,0]
            ]);
           translate([0,20,25]) rotate([0,90,0]) linear_extrude(5) circle(d=5);
           translate([0,20,10]) rotate([0,90,0]) linear_extrude(5) circle(d=5);
        }
        translate([b/2-bat_b/2-5,t-bat_t2,0]) linear_extrude(20) square([10,bat_t2]);
       
    }
    translate([0,t,0]) linear_extrude(d1) circle(d=d_corner);
    // M4 30mm in der ecke
    translate([0+30,t-30,0]) linear_extrude(d1) circle(d=4);
    // achse
    translate([-d2,t-t_achse,h_achse]) rotate([0,90,0]) linear_extrude(3*d2) circle(d=d_achse);  
    // achsen schrauben
    translate([-d2,t-t_achse-d_achsen_schrauben_abstand/2,h_achse]) rotate([0,90,0]) linear_extrude(3*d2) circle(d=d_achsen_schrauben);  
    translate([-d2,t-t_achse+d_achsen_schrauben_abstand/2,h_achse]) rotate([0,90,0]) linear_extrude(3*d2) circle(d=d_achsen_schrauben);      
    //
    motor();
    // kabelbinder für motor
    translate([a_motor-b2_motor+1*l1_motor/6+b2_motor,t-t_achse+d4_motor/2,d1+d5/2]) rotate([90,0,0]) linear_extrude(d4_motor) square([2*d5,d5],true);    
    translate([a_motor-b2_motor+2*l1_motor/6+b2_motor,t-t_achse+d4_motor/2,d1+d5/2]) rotate([90,0,0]) linear_extrude(d4_motor) square([2*d5,d5],true);    
    translate([a_motor-b2_motor+3*l1_motor/6+b2_motor,t-t_achse+d4_motor/2,d1+d5/2]) rotate([90,0,0]) linear_extrude(d4_motor) square([2*d5,d5],true);    
    translate([a_motor-b2_motor+4*l1_motor/6+b2_motor,t-t_achse+d4_motor/2,d1+d5/2]) rotate([90,0,0]) linear_extrude(d4_motor) square([2*d5,d5],true);    
    // kabelkanal
    translate([d2+15/2,0,d1+15/2]) rotate([-90,0,0]) linear_extrude(d2) circle(d=13);    
    translate([d2+maehmotor_ausschnitt,0,d1+15/2]) rotate([-90,0,0]) linear_extrude(d2) circle(d=13);    
    // materialsparloch
    //translate([maehmotor_ausschnitt-10,45/2+d2,0]) rotate([0,0,0]) linear_extrude(d1) circle(d=45);    
  }

  if( show_motor ) color([1,0,0]) motor();
}

drive_motor_holder();
if( show_mirror )translate([b,0,0]) mirror([1,0,0]) drive_motor_holder();
 
$fn=100;

show_mirror=1;
enable_stabi=1;
show_lawnmotorholder=1;
show_threads=1;

show_motor=0;
show_motor_driver_A=0;
show_motor_driver_B=0;
show_bat_monitor=0;
show_stabi=0;

include <ISOThread.scad>

b=244+1;
t=169;
maehmotor_ausschnitt=50;
t_achse=94+2;
h_achse=24.3-3.2-3.5+6;
d_achse=20+6.5;
d_achsen_schrauben=8.5;
d_achsen_schrauben_abstand=90;
d_corner=25;
//h=68-3.5;
h=h_achse+d_achse/2+5;
d1=5;
d2=7;
d3=2;

// motor
d5_motor=45;
d1_motor=38.2;
d2_motor=12.3;
d3_motor=6;
d4_motor=d1_motor+2*d3;
l1_motor=94;//76;
l2_motor=5.6;
l3_motor=50;
l5_motor=60;
l4_motor=76-l5_motor;
x_motor=d1_motor-(31.4-d2_motor/2)-d2_motor/2;
a_motor=23;
b1_motor=3;
b2_motor=4;
d5=2.5; // motor kabelbinderloch

bat_b=150+1;
bat_t1=65+1;
//bat_t2=8;
bat_t2=t-((t-t_achse)+d4_motor/2+bat_t1+2);

module save_fil() {
    translate([42,28,-10]) rotate([0,0,0]) linear_extrude(50) circle(d=40);
    translate([a_motor-b2_motor+1*l1_motor/6+b2_motor,t-t_achse,-10]) rotate([0,0,0]) linear_extrude(50) circle(d=25);
    translate([a_motor-b2_motor+3*l1_motor/6+b2_motor,t-t_achse,-10]) rotate([0,0,0]) linear_extrude(50) circle(d=25);
    translate([a_motor-b2_motor+5*l1_motor/6+b2_motor,t-t_achse,-10]) rotate([0,0,0]) linear_extrude(50) circle(d=25);
    //translate([28,t-t_achse+48,-10]) rotate([0,0,0]) linear_extrude(50) circle(d=25);
}

module stabi() {
    /*translate([0,85,53]){*/
    translate([-1,t-t_achse,h_achse+d_achsen_schrauben_abstand/2]){
        //rotate([0,90,0]) linear_extrude(b) circle(d=d_achsen_schrauben);
        rotate([0,90,0]) thread_out(8,b+2);
    }
}

module lawnmotorholder() {
    difference()
    {
        union()
        {
            linear_extrude(d1*2) polygon([
                [b/2-maehmotor_ausschnitt,0],
                [b/2-maehmotor_ausschnitt,maehmotor_ausschnitt],
                [b/2+maehmotor_ausschnitt,maehmotor_ausschnitt],
                [b/2+maehmotor_ausschnitt,0],
            ]);        
            translate([b/2,maehmotor_ausschnitt/2,0]) rotate([0,0,90]) linear_extrude(20) 
                square([maehmotor_ausschnitt,maehmotor_ausschnitt*2],true);
        }
        union()
        {
            translate([b/2,maehmotor_ausschnitt-16-25/2,-1]) rotate([0,0,90]) linear_extrude(30) circle(d=25);
            translate([b/2,maehmotor_ausschnitt-16-25/2,3]) rotate([0,0,90]) linear_extrude(30) circle(d=25+10);
            translate([b/2,maehmotor_ausschnitt/2,10]) rotate([0,0,90]) linear_extrude(40) 
                square([maehmotor_ausschnitt-8,maehmotor_ausschnitt+25],true);
            if(show_threads) translate([b/2-30,maehmotor_ausschnitt-16-25/2+10+5,0]) thread_out(6,20);
            if(show_threads) translate([b/2-30,maehmotor_ausschnitt-16-25/2-10+5,0]) thread_out(6,20);
            if(show_threads) translate([b/2+30,maehmotor_ausschnitt-16-25/2+10+5,0]) thread_out(6,20);
            if(show_threads) translate([b/2+30,maehmotor_ausschnitt-16-25/2-10+5,0]) thread_out(6,20);
        }
    }
}

module bat_monitor() {
    translate([8,115,5]){
        linear_extrude(5) square([38,32]);
        translate([2.5,2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([38-2.5,2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([38-2.5,32-2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([2.5,32-2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);

        translate([2.5,2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([38-2.5,2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([38-2.5,322.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([2.5,32-2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
    }
}

module motor_driver_A() {
    translate([15,10,5]){
        linear_extrude(5) square([54.4,28]);
        translate([2.5,2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([54.4-2.5,2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([54.4-2.5,28-2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([2.5,28-2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);

        translate([2.5,2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([54.4-2.5,2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([54.4-2.5,28-2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([2.5,28-2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
    }
}

module motor_driver_B() {
    translate([21,8,5]){
        linear_extrude(5) square([43,43]);
        translate([2.5,2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([43-2.5,2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([43-2.5,43-2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);
        translate([2.5,43-2.5,-20]) rotate([0,0,90]) linear_extrude(30) circle(d=3);

        translate([2.5,2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([43-2.5,2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([43-2.5,43-2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
        translate([2.5,43-2.5,-5]) rotate([0,0,90]) linear_extrude(2) circle(d=6);
    }
}

module motor() {
    translate([0,0,x_motor]){
        translate([a_motor,t-t_achse,h_achse]) rotate([0,90,0]) linear_extrude(l1_motor) circle(d=d1_motor);  
        translate([a_motor+l5_motor,t-t_achse,h_achse]) rotate([0,90,0]) linear_extrude(l4_motor/*l1_motor-l5_motor*/) circle(d=d5_motor);  
        translate([a_motor-l2_motor,t-t_achse,h_achse-x_motor]) rotate([0,90,0]) linear_extrude(l2_motor) circle(d=d2_motor);  
        translate([a_motor-l2_motor-l3_motor,t-t_achse,h_achse-x_motor]) rotate([0,90,0]) linear_extrude(l3_motor) circle(d=d3_motor);  
       for(r=[60:360/3:360]) {
           translate([a_motor-10,t-t_achse,h_achse]) rotate([r/2,0,0]) translate([0,0,31/2]) rotate([0,90,0]) linear_extrude(10) circle(d=4);  
           translate([a_motor-10,t-t_achse,h_achse]) rotate([-r/2,0,0]) translate([0,0,31/2]) rotate([0,90,0]) linear_extrude(10) circle(d=4);  
           translate([a_motor-25-17,t-t_achse,h_achse]) rotate([r/2,0,0]) translate([0,0,31/2]) rotate([0,90,0]) linear_extrude(10+25) circle(d=6.5);
           translate([a_motor-25-17,t-t_achse,h_achse]) rotate([-r/2,0,0]) translate([0,0,31/2]) rotate([0,90,0]) linear_extrude(10+25) circle(d=6.5);  
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
        linear_extrude(h) square([d2,t-25]);
        translate([0,t-t_achse,h]) rotate([0,90,0]) linear_extrude(d2) circle(h);
        
        translate([0,t-t_achse-d4_motor/2,0]) linear_extrude(h_achse+x_motor+b1_motor) square([l1_motor+1*b2_motor+a_motor-b2_motor,d4_motor]);
        translate([0,t-t_achse-d4_motor/2,0]) linear_extrude(h_achse+x_motor+b1_motor+20+24) square([a_motor+2,d4_motor]);
        
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
        translate([b/2-bat_b/2-30,t-10+5,0]) rotate([90,0,0]) linear_extrude(15) polygon([
          [0,0],
          [25,60],
          [29,60],
          [30,0]
        ]);
        translate([b/2-bat_b/2-30-10,t-59,0]) rotate([90,0,0]) linear_extrude(15) polygon([
          [0,0],
          [25+10,60],
          [29+10,60],
          [30+10,0]
        ]);

        x = t - maehmotor_ausschnitt - bat_t1 - bat_t2;
        translate([b/2-5,t-bat_t1-bat_t2-x,0]) difference(){
            rotate([90,0,90]) linear_extrude(5) polygon([
              [0,0],
              [0,80],
              [x,80],
              [x,0]
            ]);           
        }
        translate([b/2-5-25-10,t-bat_t1-bat_t2-30+30,0]) difference(){
            rotate([90,0,0]) linear_extrude(5) polygon([
              [0,0],
              [10+25,70],
              [10+30,70],
              [10+30,0]
            ]);
        }
        translate([b/2-bat_b/2-5,t-bat_t2,0]) linear_extrude(20) square([10,bat_t2]);     
        
    }
    translate([0,t-25+d_corner/2,0]) linear_extrude(d1) circle(d=d_corner);
    translate([0,t-d_corner/2,0]) linear_extrude(d1) square([d_corner/2,d_corner/2]);
    // M4 30mm in der ecke
    translate([0+30,t-30,0]) linear_extrude(d1) circle(d=4);
    // achse
    translate([-d2,t-t_achse,h_achse]) rotate([0,90,0]) linear_extrude(3*d2) circle(d=d_achse);  
    // achsen schrauben
    translate([-d2,t-t_achse-d_achsen_schrauben_abstand/2,h_achse]) rotate([0,90,0]) linear_extrude(3*d2) circle(d=d_achsen_schrauben);  
    translate([-d2,t-t_achse+d_achsen_schrauben_abstand/2,h_achse]) rotate([0,90,0]) linear_extrude(3*d2) circle(d=d_achsen_schrauben);      
    //
    motor();
    // durchbroch für voderen motor flansch
    translate([0,0,x_motor]){
        translate([a_motor-l2_motor-30,t-t_achse,h_achse-x_motor]) rotate([0,90,0]) linear_extrude(l2_motor+40) circle(d=d2_motor);  
    }    
    motor_driver_A();
    motor_driver_B();
    bat_monitor();   
    //stabi();  
    save_fil();

    // montagelöscher für achsenkupplung    
    d = 7;
    translate([d2+d/2,t-t_achse,h_achse]) rotate([45,0,0]) linear_extrude(100) circle(d=d);
    translate([d2+d/2,t-t_achse,h_achse]) rotate([0,0,0]) linear_extrude(100) circle(d=d);
    translate([d2+d/2,t-t_achse,h_achse]) rotate([180,0,0]) linear_extrude(100) circle(d=d);
    translate([d2+d/2,t-t_achse,h_achse]) rotate([-45,0,0]) linear_extrude(100) circle(d=d);

    // kabelbinder für motor
    translate([a_motor-b2_motor+2*l1_motor/6+b2_motor,t-t_achse+d4_motor/2,d1+d5/2]) rotate([90,0,0]) linear_extrude(d4_motor) square([2*d5,d5],true);    
    translate([a_motor-b2_motor+3*l1_motor/6+b2_motor,t-t_achse+d4_motor/2,d1+d5/2]) rotate([90,0,0]) linear_extrude(d4_motor) square([2*d5,d5],true);    
    // kabelkanal
    translate([d2+15/2,0,d1+15/2]) rotate([-90,0,0]) linear_extrude(d2) circle(d=13);    
    translate([d2+maehmotor_ausschnitt,0,d1+15/2]) rotate([-90,0,0]) linear_extrude(d2) circle(d=13);    
    // materialsparloch
    //translate([maehmotor_ausschnitt-10,45/2+d2,0]) rotate([0,0,0]) linear_extrude(d1) circle(d=45);    
  }

  if( show_motor ) color([1,0,0]) motor();
  if( show_motor_driver_A ) color([1,0,0]) motor_driver_A();      
  if( show_motor_driver_B ) color([1,0,0]) motor_driver_B();    
  if( show_bat_monitor ) color([1,0,0]) bat_monitor();          
  //if( show_stabi ) color([1,0,0]) stabi();                
  //save_fil();
}

difference() {
    union() {
        drive_motor_holder();
        if( show_mirror )translate([b,0,0]) mirror([1,0,0]) drive_motor_holder();
        if( show_lawnmotorholder ) lawnmotorholder();
    }
    union() {
        if ( enable_stabi ) stabi();                
    }
}

 
if ( enable_stabi ) if( show_stabi ) color([1,0,0]) stabi();                

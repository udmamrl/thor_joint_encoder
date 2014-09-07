wheel_r=25.4*8;
wheel_w=100;

front_d=500;
front_w=600;
front_h=400;

back_d=400;
back_w=600;
back_h=500;

scale([1/1000, 1/1000, 1/1000]){

/*
// front
translate([00,front_w/2+wheel_w/2,wheel_r]) rotate([90, 0, 0]) cylinder(h=wheel_w,r=wheel_r,center=true);
translate([00,-(front_w/2+wheel_w/2),wheel_r]) rotate([90, 0, 0]) cylinder(h=wheel_w,r=wheel_r,center=true);
translate([00,0,wheel_r+front_h-100])cube([front_d, front_w, front_h],center=true);
*/

// back
translate([0,0,wheel_r+front_h+125]) cylinder(h=50,r=50,center=true);
translate([-500,0,wheel_r+front_h+125])cube([1000, 50, 50],center=true);
translate([-1000,0,0])union(){
translate([00,back_w/2+wheel_w/2,wheel_r]) rotate([90, 0, 0]) cylinder(h=wheel_w,r=wheel_r,center=true);
translate([00,-(back_w/2+wheel_w/2),wheel_r]) rotate([90, 0, 0]) cylinder(h=wheel_w,r=wheel_r,center=true);
translate([00,0,wheel_r+back_h-200])cube([back_d, back_w, back_h],center=true);
// bar top
}

}

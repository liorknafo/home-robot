#include "../Source/HomeRobotSim/DriveCore.h"
#include <cassert>
#include <limits>
#include <iostream>
int main(){
 double l,r;DriveCore d;
 d.update(0,.01,0,0,false,l,r);assert(!d.armed&&l==0&&r==0);
 d.command(0,.15,0,true,false);d.update(.01,.01,0,0,false,l,r);assert(d.armed&&l>0&&r>0&&std::abs(l)<=d.max_torque);assert(d.left_target<=d.accel*.01+1e-8);
 d.update(.5,.01,0,0,false,l,r);assert(!d.armed&&l==0&&r==0);
 d.command(.6,.1,0,false,false);d.update(.61,.01,0,0,false,l,r);assert(!d.armed);
 d.command(.7,.1,0,true,false);d.update(.71,.01,0,0,true,l,r);assert(!d.armed);
 d.update(.72,.01,0,0,false,l,r);assert(!d.armed);
 d.command(.8,.1,0,true,false);d.update(.81,.01,0,0,false,l,r);assert(d.armed);
 d.command(.9,.1,0,true,true);d.update(.91,.01,0,0,false,l,r);assert(d.estop&&!d.armed&&l==0);
 d.clear_stop();d.command(1,.1,0,false,false);d.update(1.01,.01,0,0,false,l,r);assert(!d.armed);
 d.command(1.1,.1,0,true,false);d.update(1.11,.01,0,0,false,l,r);assert(d.armed);
 assert(!d.command(1.2,std::numeric_limits<double>::quiet_NaN(),0,true,false));assert(!d.armed);
 DriveCore turn;turn.command(0,0,.7,true,false);turn.update(.01,.01,0,0,false,l,r);assert(l<0&&r>0);
 // Existing ramp (0.25 m/s^2) bounds every setpoint change, forward braking and reverse starts alike; no extra slew.
 {DriveCore s;s.command(0,.15,0,true,false);double t=0;for(int i=0;i<200;i++){t+=.01;s.update(t,.01,0,0,false,l,r);s.command(t,.15,0,false,false);}
  assert(std::abs(s.left_target-.15)<1e-9);
  for(int i=0;i<10;i++){const double before=s.left_target;t+=.01;s.command(t,-.15,0,false,false);s.update(t,.01,0,0,false,l,r);assert(std::abs(s.left_target-before)<=s.accel*.01+1e-12&&s.left_target<before);}}
 // Front bumper: a press stops and disarms; while it stays pressed only reverse (both wheels <= 0) or a pure spin re-arms.
 DriveCore b;double t=0;auto tick=[&](bool hazard,bool bumper,double ls=0,double rs=0){t+=.01;b.update(t,.01,ls,rs,hazard,l,r,bumper);};
 b.command(t,.1,0,true,false);tick(false,false);assert(b.armed&&b.bump_events==0);
 b.command(t,.1,0,false,false);tick(false,true,2.5,2.5);assert(!b.armed&&l==0&&r==0&&b.bump_events==1&&!b.fault);
 tick(false,true);assert(b.bump_events==1);
 b.command(t,.1,0,true,false);tick(false,true);assert(!b.armed);                                   // forward refused
 b.command(t,.01,0,true,true);b.clear_stop();b.command(t,.01,0,true,false);tick(false,true);assert(!b.armed);
 b.command(t,1e-3,.2,true,false);assert(!b.armed);                                                  // creeping forward + yaw is not a spin
 b.command(t,0,.7,true,false);tick(false,true);assert(!b.armed);                                   // spin faster than bump_spin refused
 b.command(t,0,.51,true,false);assert(!b.armed);
 b.command(t,0,-.51,true,false);assert(!b.armed);
 const double wmax=2*.05/b.track;                                                                   // |w| <= 2|v|/track at v=-0.05: 0.4386
 b.command(t,-.05,wmax+1e-3,true,false);tick(false,true);assert(!b.armed);                         // reverse with too much yaw refused
 b.command(t,-.05,-(wmax+1e-3),true,false);assert(!b.armed);
 b.command(t,-.05,0,true,false);assert(b.armed);tick(false,true);assert(b.armed&&l<0&&r<0&&b.left_target<0&&b.right_target<0);
 // Reverse with yaw up to the limit: BOTH wheels stay <= 0 (no one-wheel pivot forward) and the centre moves back.
 for(double w:{.3,wmax-1e-6,-.3,-(wmax-1e-6)}){
  DriveCore c;double tc=0;c.command(tc,.1,0,true,false);tc+=.01;c.update(tc,.01,0,0,false,l,r,true);assert(!c.armed);
  assert(c.command(tc,-.05,w,true,false)&&c.armed);
  for(int i=0;i<80;i++){tc+=.01;c.command(tc,-.05,w,false,false);c.update(tc,.01,-1,-1,false,l,r,true);assert(c.armed&&c.left_target<=1e-12&&c.right_target<=1e-12);}
  assert(c.left_target+c.right_target<-.09);                                                       // centre speed ~ -0.05 m/s
  assert(std::abs((c.right_target-c.left_target)/c.track-w)<1e-6);                                  // yaw passed through unclamped
 }
 // Pure spin while pressed: accepted up to bump_spin and passed through symmetrically (left == -right != 0).
 for(double w:{.4,-.4,.5,-.5}){
  DriveCore c;double tc=0;c.command(tc,.1,0,true,false);tc+=.01;c.update(tc,.01,0,0,false,l,r,true);assert(!c.armed&&c.bump_events==1);
  assert(c.command(tc,0,w,true,false)&&c.armed);
  for(int i=0;i<40;i++){tc+=.01;c.command(tc,0,w,false,false);c.update(tc,.01,0,0,false,l,r,true);assert(c.armed);}
  assert(std::abs(c.left_target+c.right_target)<1e-12&&std::abs(c.right_target-w*c.track/2)<1e-9);
  assert(w>0?(c.left_target<0&&c.right_target>0&&l<0&&r>0):(c.left_target>0&&c.right_target<0&&l>0&&r<0));
  assert(c.bump_events==1);
 }
 {DriveCore c;double tc=0;c.command(tc,.1,0,true,false);tc+=.01;c.update(tc,.01,0,0,false,l,r,true);assert(c.command(tc,5e-5,.3,true,false)&&c.armed);} // |v|<1e-4 counts as a spin
 // The pawn sets track / wheel_radius from robot-parameters.json (ConfigureDrive): the pressed-reverse limit follows it.
 {DriveCore c;c.track=.30;c.wheel_radius=.05;double tc=0;c.command(tc,.1,0,true,false);tc+=.01;c.update(tc,.01,0,0,false,l,r,true);assert(!c.armed);
  const double lim=2*.05/c.track;                                                                   // 0.333 (0.4386 at .228)
  c.command(tc,-.05,lim+1e-3,true,false);assert(!c.armed);c.command(tc,-.05,lim-1e-3,true,false);assert(c.armed);
  for(int i=0;i<80;i++){tc+=.01;c.command(tc,-.05,lim-1e-3,false,false);c.update(tc,.01,-1,-1,false,l,r,true);assert(c.left_target<=1e-12&&c.right_target<=1e-12);}
  assert(std::abs((c.right_target-c.left_target)/c.track-(lim-1e-3))<1e-6);}
 // Armed + pressed, then a command outside the allowed set arrives without arm: both setpoints ramp to zero together.
 for(int i=0;i<30;i++){b.command(t,-.05,.3,false,false);tick(false,true,-1,-1);assert(b.armed&&b.left_target<=0&&b.right_target<=0);}
 for(int i=0;i<60;i++){b.command(t,.1,0,true,false);tick(false,true,-1,-1);assert(b.armed&&b.left_target<=0&&b.right_target<=0);}
 assert(b.left_target==0&&b.right_target==0);
 for(int i=0;i<30;i++){b.command(t,0,.7,false,false);tick(false,true);assert(b.armed&&b.left_target==0&&b.right_target==0);} // too-fast spin: no pivot, both zero
 for(int i=0;i<30;i++){b.command(t,0,.4,false,false);tick(false,true);assert(b.armed);}
 assert(b.left_target<0&&b.right_target>0&&std::abs(b.left_target+b.right_target)<1e-12);
 for(int i=0;i<30;i++){b.command(t,-.01,.3,false,false);tick(false,true);assert(b.armed);}            // |w| > 2|v|/track: held at zero, not clamped per wheel
 assert(std::abs(b.left_target)<1e-12&&std::abs(b.right_target)<1e-12);
 b.command(t,0,0,false,false);for(int i=0;i<30;i++)tick(false,true);
 b.command(t,-.05,0,false,false);tick(false,false);assert(b.armed&&b.bump_events==1);
 b.command(t,.1,0,false,false);for(int i=0;i<10;i++)tick(false,false);assert(b.armed&&b.left_target>0&&b.right_target>0);
 tick(false,true,2.5,2.5);assert(!b.armed&&b.bump_events==2);
 DriveCore pulse;pulse.command(0,.1,0,true,false);pulse.update(.01,.01,0,0,false,l,r,true);pulse.update(.02,.01,0,0,false,l,r,false);pulse.update(.03,.01,0,0,false,l,r,true);pulse.update(.04,.01,0,0,false,l,r,false);
 assert(pulse.bump_events==2&&!pulse.armed);pulse.command(.05,.1,0,true,false);assert(pulse.armed);
 // Bumper while armed in reverse: estop latch and watchdog still win.
 DriveCore e;e.command(0,.1,0,true,false);e.update(.01,.01,0,0,false,l,r,true);e.command(.02,-.05,0,true,true);assert(e.estop&&!e.armed);
 e.update(.03,.01,0,0,false,l,r,true);e.command(.04,-.05,0,true,false);assert(!e.armed);
 e.clear_stop();e.command(.05,-.05,0,true,false);assert(e.armed);e.update(.06,.01,0,0,false,l,r,true);assert(e.armed&&l<0);
 e.update(.5,.01,0,0,false,l,r,true);assert(!e.armed&&l==0);e.command(.51,-.05,0,false,false);e.update(.52,.01,0,0,false,l,r,true);assert(!e.armed);
 e.command(.53,0,.4,true,true);assert(!e.armed);                                                      // estop also beats a pressed spin
 // Cliff/tilt hazards stay hard stops: disarm every tick and refuse every arm (reverse AND spin), with or without the bumper.
 for(bool bumper:{false,true}){
  DriveCore h;h.command(0,.1,0,true,false);h.update(.01,.01,0,0,false,l,r);assert(h.armed);
  h.update(.02,.01,0,0,true,l,r,bumper);assert(!h.armed&&l==0&&r==0&&h.fault);
  for(double v:{-.05,0.,.1}){h.command(.03,v,0,true,false);assert(!h.armed);h.update(.04,.01,0,0,true,l,r,bumper);assert(!h.armed&&l==0&&r==0);}
  h.command(.035,0,.4,true,false);assert(!h.armed);h.update(.045,.01,0,0,true,l,r,bumper);assert(!h.armed&&l==0&&r==0);
  h.command(.05,-.05,0,true,false);h.update(.06,.01,0,0,true,l,r,bumper);assert(!h.armed);
  h.update(.07,.01,0,0,false,l,r,bumper);assert(!h.armed);h.command(.08,-.05,0,true,false);assert(h.armed);
 }
 // Pressed spin direction: only TOWARD the pressed side (left switch only -> w>=0, right only -> w<=0, both -> either);
 // spinning away would carry the obstacle past +/-span onto the unsensed rear shell.
 for(int side:{1,-1}){
  DriveCore c;double tc=0;c.command(tc,.1,0,true,false);tc+=.01;c.update(tc,.01,0,0,false,l,r,true,side);assert(!c.armed&&c.bump_side==side);
  c.command(tc,0,-.4*side,true,false);assert(!c.armed);                                             // away from the contact: refused
  c.command(tc,0,-.1*side,true,false);assert(!c.armed);
  c.command(tc,-.05,.3,true,false);assert(c.armed);c.disarm();                                      // reverse is allowed on either side
  assert(c.command(tc,0,.4*side,true,false)&&c.armed);                                              // toward it: accepted
  for(int i=0;i<40;i++){tc+=.01;c.command(tc,0,.4*side,false,false);c.update(tc,.01,0,0,false,l,r,true,side);assert(c.armed);}
  assert(side>0?(c.left_target<0&&c.right_target>0):(c.left_target>0&&c.right_target<0));
  // The contact crosses the centre zone onto the other side (only the other switch pressed): the same spin is now away
  // from it, so both setpoints ramp to zero together; still armed, no new bump event (the bumper never released).
  for(int i=0;i<60;i++){tc+=.01;c.command(tc,0,.4*side,false,false);c.update(tc,.01,0,0,false,l,r,true,-side);assert(c.armed&&c.bump_events==1);
   assert(std::abs(c.left_target+c.right_target)<1e-12);}
  assert(c.left_target==0&&c.right_target==0);
  // Both switches (centre zone): either direction re-arms.
  DriveCore d2;tc=0;d2.command(tc,.1,0,true,false);tc+=.01;d2.update(tc,.01,0,0,false,l,r,true,0);assert(!d2.armed&&d2.bump_side==0);
  assert(d2.command(tc,0,-.4*side,true,false)&&d2.armed);d2.disarm();assert(d2.command(tc,0,.4*side,true,false)&&d2.armed);
  // Released: the side is cleared and the next press sets it again.
  tc+=.01;d2.update(tc,.01,0,0,false,l,r,false,side);assert(d2.bump_side==0);
 }
 std::cout<<"PASS: startup, acceleration limit, torque limit, watchdog, explicit re-arm, fault latch, estop, non-finite command, differential turn, 0.25 m/s^2 ramp bounds braking/reverse start\n";
 std::cout<<"PASS: bumper press disarms and counts rising edges (incl. 1-tick pulses), forward/creep/fast-spin/over-yaw re-arm refused while pressed, reverse re-arm with |w|<=2|v|/track keeps both wheels <=0 (track from the loaded geometry), pressed pure spin |w|<=0.5 passes symmetrically and only toward the pressed side (left only w>=0, right only w<=0, both either; held at zero once the contact crosses to the other side), disallowed pressed commands hold both setpoints at zero (no one-wheel pivot), normal drive after release, estop/watchdog over bumper, cliff/tilt refuse all arming\n";
 {// Reverse is capped at max_reverse; forward keeps max_speed.
  DriveCore d;double l,r;
  d.command(0,-.15,0,true,false);assert(std::abs(d.v+d.max_reverse)<1e-12);
  for(int k=1;k<=200;k++){d.command(k*.01,-.15,0,true,false);d.update(k*.01,.01,0,0,false,l,r);}
  assert(d.left_target>=-d.max_reverse-1e-9&&d.right_target>=-d.max_reverse-1e-9&&d.left_target<0);
  d.command(3,.15,0,true,false);assert(std::abs(d.v-d.max_speed)<1e-12);
 }
}

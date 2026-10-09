#pragma once
#include <algorithm>
#include <cmath>
// Platform-independent controller core. Inputs/outputs use SI units; v is the speed of base_link, which is the
// drive-axle midpoint (the round robot turns about its centre).
// This same core can be compiled behind a Pico hardware adapter; no Pico firmware is flashed here.
// Hazards (cliff/invalid range, tilt) disarm every tick and block arming. The front bumper behaves like a real
// bumper robot: every new press disarms (stop) and is counted in bump_events. While it stays pressed the drive may be
// re-armed only for
//  - REVERSE: v<=0 and |w|<=2|v|/track, so BOTH wheels turn backwards (or stop) and the centre moves back, or
//  - a pure SPIN about the centre: |v|<spin_v_eps and |w|<=bump_spin, and only TOWARD the pressed side: left switch only
//    (contact at az>0) -> w>=0, right switch only -> w<=0, both switches (centre zone) -> either way. A spin keeps every
//    distance from base_link, but the skin is only ~1.5 mm outside the rigid rear shell, so spinning AWAY would carry a
//    pressed obstacle past +/-span onto the unsensed rear shell. Turning toward it moves the contact toward az 0; once it
//    crosses the centre zone onto the other side the spin direction becomes disallowed and both wheels are held at zero.
// A command outside that set while pressed holds both wheel setpoints at zero (they ramp down); there is no per-wheel
// clamp, so a pressed robot never makes a one-wheel pivot that sweeps the shell outside its disc.
// Acceleration: wheel setpoints ramp at accel (0.25 m/s^2) in both directions, which also bounds braking and reverse
// starts; there is no separate deceleration slew.
// Reverse speed is capped at max_reverse (0.06 m/s): the rear cliff sensors lead the rear caster by only a few cm and
// a hazard disarms the drive (it coasts), so backing up must stay slow enough to stop before the caster reaches an edge.
struct DriveCore {
 double wheel_radius=.04, track=.228, max_speed=.15, max_reverse=.06, max_yaw=1.0;
 double accel=.25, max_torque=.10, kp=.006, ki=.015, timeout=.35, bump_spin=.5, spin_v_eps=1e-4;
 bool armed=false, estop=false, fault=false, bump=false, have_command=false;
 int bump_events=0, bump_side=0;   // bump_side: +1 left switch only, -1 right switch only, 0 both (or not pressed)
 double command_time=-1e9, v=0,w=0,left_target=0,right_target=0,il=0,ir=0;
 void disarm(){armed=false;left_target=right_target=0;il=ir=0;}
 bool pressed_reverse() const {return v<=0&&std::abs(w)<=2*std::abs(v)/track;}
 bool pressed_spin() const {return std::abs(v)<spin_v_eps&&std::abs(w)<=bump_spin&&w*bump_side>=0;}
 // Motion that is allowed while the bumper is held: reverse with both wheels <= 0, or a slow pure spin toward the pressed side.
 bool bump_motion_ok() const {return pressed_reverse()||pressed_spin();}
 bool command(double now,double linear,double yaw,bool request_arm,bool stop) {
  if(!std::isfinite(linear)||!std::isfinite(yaw)){fault=true;disarm();return false;}
  if(stop){estop=true;disarm();}
  v=std::clamp(linear,-max_reverse,max_speed);w=std::clamp(yaw,-max_yaw,max_yaw);
  command_time=now;have_command=true;
  if(request_arm&&!estop&&!fault&&(!bump||bump_motion_ok()))armed=true;
  return true;
 }
 void clear_stop(){estop=false;disarm();}
 // hazard: cliff/invalid range or tilt this tick. bumper: front bumper pressed this tick. side: which switch is pressed
 // (+1 left only, -1 right only, 0 both or unknown; a caller without per-switch data passes 0 = either spin direction).
 void update(double now,double dt,double left_speed,double right_speed,bool hazard,double& tl,double& tr,bool bumper=false,int side=0) {
  const bool press=bumper&&!bump;bump_events+=press;fault=hazard;bump=bumper;bump_side=bumper?(side>0)-(side<0):0;
  if(hazard||press||estop||!have_command||now-command_time>timeout)disarm();
  if(!armed){tl=tr=0;return;}
  double ls=v-w*track/2,rs=v+w*track/2;
  if(bump&&!bump_motion_ok())ls=rs=0;   // pressed: only reverse (both wheels <= 0) or a pure spin pass through
  const double scale=std::max(1.0,std::max(std::abs(ls),std::abs(rs))/max_speed);
  auto ramp=[&](double old,double wanted){return old+std::clamp(wanted-old,-accel*dt,accel*dt);};
  left_target=ramp(left_target,ls/scale);right_target=ramp(right_target,rs/scale);
  auto pid=[&](double wanted,double measured,double& integral){
   const double err=wanted/wheel_radius-measured;
   const double next=std::clamp(integral+err*dt,-max_torque/ki,max_torque/ki);
   const double raw=kp*err+ki*next;
   if(std::abs(raw)<max_torque||err*raw<0)integral=next;
   return std::clamp(kp*err+ki*integral,-max_torque,max_torque);
  };
  tl=pid(left_target,left_speed,il);tr=pid(right_target,right_speed,ir);
 }
};

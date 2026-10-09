#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
// Platform-independent geometry of the ROUND robot (no Unreal types, so tests can compile it with plain clang++).
// Robot frame: x forward, y LEFT (REP-103), z up from the floor, origin = base_link = the centre of the round shell,
// which is also the drive-axle midpoint. Units: cm and degrees. RobotPawn fills a Geometry from
// Content/RobotData/robot-parameters.json (metres there) at BeginPlay; the defaults below are only the fallback.
// Unreal component space has Y pointing RIGHT, so a robot point (x, y_left, z) is FVector(x, -y_left, z-BodyHeight).
namespace RobotGeometry {
constexpr double Pi=3.14159265358979323846;
struct BumpColumn{double X,Y,NX,NY,Az;uint8_t Zone;};
struct Geometry{
 double axle_x=0;              // drive_axle_x_m
 double track_half=11.4;       // wheel_track_m / 2
 double wheel_radius=4;        // wheel_radius_m
 double skin_radius=15.95;     // body_radius_m: bumper contact skin (shell 15.8 + 0.15 foam)
 double rigid_radius=15.8;     // rigid_radius_m: rigid rear shell
 double bumper_travel=.5;      // bumper_travel_m: the front bumper's hard stop lies this far inside the skin
 double bumper_span_deg=90;    // bumper_span_deg: the skin covers azimuth -span..+span about base_link
 double column_step_deg=4;     // probe column spacing (upper bound)
 std::vector<double> zone_edges_deg{-90,-55,-20,20,55,90}; // bump_zone_edges_deg: zone i spans edges[i]..edges[i+1]
 double rear_caster_x=-7.5,rear_caster_y=0; // rear_caster_xy_m (the only caster)
 double caster_radius=1.27;    // caster_ball_radius_m (25.4 mm ball)
 double caster_lift=0;         // rear_caster_z_m: ball bottom above the wheel contact plane
 std::vector<std::array<double,2>> cliff_xy{{10.5,7.2},{10.5,-7.2},{-10.8,7.2},{-10.8,-7.2}}; // cliff_sensors_m FL,FR,RL,RR
 double cliff_z=4.25;          // cliff_z_m
 std::array<double,3> camera{11.7,0,15}; // camera_mount_m
 std::array<double,3> lidar{2,0,19};     // lidar_mount_m (scan plane)
 double shell_bottom_z=1.8;    // shell_bottom_z_m: lowest rigid shell edge above the floor
 double body_top_z=14.0;       // body_top_z_m: top of the body collider (LiDAR housing is separate)
 double bumper_top_z=12.0;     // bumper_top_z_m: top edge of the bumper skin (CAD bumper_z[1]); probe rows stop below it
 std::array<double,2> turret_xy{2,0};           // lidar_turret_xy_m: centre of the LiDAR turret cylinder
 double turret_radius=2.9922;                   // lidar_turret_radius_m
 std::array<double,2> turret_z{13.825,15.515};  // lidar_turret_z_m: turret bottom and top (the top is robot_top_z_m)
};
// Accepts the full ascending list [-90,-55,-20,20,55,90] or the positive half [20,55,90] (mirrored).
inline std::vector<double> normalise_edges(std::vector<double> e){
 std::sort(e.begin(),e.end());
 if(!e.empty()&&e.front()>0){std::vector<double> full;for(auto it=e.rbegin();it!=e.rend();++it)full.push_back(-*it);full.insert(full.end(),e.begin(),e.end());e=full;}
 if(e.size()<2)e={-90,-55,-20,20,55,90};
 return e;
}
// Index of the zone that contains azimuth 0 (bumper centre).
inline int centre_zone(const std::vector<double>& e){
 for(size_t i=0;i+1<e.size();i++)if(e[i+1]>=0)return (int)i;
 return (int)e.size()-2;
}
// Zone of an azimuth (deg, + = left). An azimuth on an edge belongs to the zone nearer the centre (as the
// original BumpZoneOf: -55 -> 1, +20 -> 2). Azimuths outside the edges clamp to the end zones.
inline uint8_t zone_of(const std::vector<double>& e,double az){
 const int n=(int)e.size()-1,c=centre_zone(e);
 if(az>=0){for(int i=c;i<n;i++)if(az<=e[i+1])return (uint8_t)i;return (uint8_t)(n-1);}
 for(int i=c;i>=0;i--)if(az>=e[i])return (uint8_t)i;
 return 0;
}
// Microswitches: the centre zone trips both. Left = centre zone and every zone to its left (+az), right = mirror.
inline uint8_t left_switch_mask(const std::vector<double>& e){uint8_t m=0;for(int i=centre_zone(e);i<(int)e.size()-1&&i<8;i++)m|=1u<<i;return m;}
inline uint8_t right_switch_mask(const std::vector<double>& e){uint8_t m=0;for(int i=0;i<=centre_zone(e)&&i<8;i++)m|=1u<<i;return m;}
// The contract form of the zone keys: bump_zone_edges_deg is the full ascending list and its outer values are
// -bumper_span_deg and +bumper_span_deg (bumper_span_deg is the HALF-span).
inline bool span_matches_edges(const std::vector<double>& e,double span){
 return e.size()>=2&&std::abs(e.front()+span)<1e-6&&std::abs(e.back()-span)<1e-6;
}
// Probe row heights (cm above the floor). The lowest row sits 0.2 mm above the shell bottom, so every obstacle tall enough
// to meet the shell (or the bumper, whose skin starts at the same height) crosses a ray first; with a fixed 2.0 cm row and
// a 1.8 cm shell bottom an 18-20 mm obstacle hit the shell unsensed. The upper rows are fixed heights clipped to the bumper
// skin: none above bumper_top_z - 0.2 (the real bumper does not sense higher; the body collider above it, up to
// body_top_z, is conservative), and a top row 2 mm below the bumper top edge when the last fixed row is > 3 mm under it.
constexpr double UpperBumpRowsCm[]={2.8,3.6,4.4,5.2,6.5,8.0,9.5,11.0,12.5};
inline std::vector<double> bump_rows(const Geometry& g){
 std::vector<double> r{g.shell_bottom_z+.02};
 const double top=std::min(g.bumper_top_z,g.body_top_z)-.2;
 for(double z:UpperBumpRowsCm)if(z>r.front()+.3&&z<=top+1e-9)r.push_back(z);
 if(top-r.back()>.3+1e-9)r.push_back(top);
 return r;
}
// Probe columns on the skin circle about base_link, evenly spaced from -span to +span with spacing <= step
// (46 columns at 4 deg for +/-90). keep_every>1 keeps every n-th column (negative control -BumperSparseColumns).
inline std::vector<BumpColumn> bump_columns(const Geometry& g,int keep_every=1){
 std::vector<BumpColumn> c;const std::vector<double> e=normalise_edges(g.zone_edges_deg);
 const int count=std::max(2,(int)std::ceil(2*g.bumper_span_deg/g.column_step_deg-1e-9)+1);
 for(int k=0;k<count;k++){
  if(keep_every>1&&k%keep_every)continue;
  const double az=-g.bumper_span_deg+2*g.bumper_span_deg*k/(count-1),r=az*Pi/180,nx=std::cos(r),ny=std::sin(r);
  c.push_back({g.skin_radius*nx,g.skin_radius*ny,nx,ny,az,zone_of(e,az)});
 }
 return c;
}
// Axis-aligned (robot frame) box that contains every probe segment from `inside` cm inside to `outside` cm outside the skin.
struct Box2{double cx,cy,ex,ey;};
inline Box2 probe_bounds(const std::vector<BumpColumn>& c,double inside,double outside,double margin=.1){
 double x0=1e9,x1=-1e9,y0=1e9,y1=-1e9;
 for(const BumpColumn& b:c)for(double d:{-inside,outside}){const double x=b.X+d*b.NX,y=b.Y+d*b.NY;x0=std::min(x0,x);x1=std::max(x1,x);y0=std::min(y0,y);y1=std::max(y1,y);}
 if(c.empty())return {0,0,0,0};
 return {(x0+x1)/2,(y0+y1)/2,(x1-x0)/2+margin,(y1-y0)/2+margin};
}
// Radius of the hard stop at an azimuth: under the bumper (|az| <= span) the bumper's travel limit (skin - travel, never
// beyond the rigid shell), behind it the rigid rear shell.
inline double front_stop_radius(const Geometry& g){return std::min(g.skin_radius-g.bumper_travel,g.rigid_radius);}
// Body collider: vertices of a `sides`-gon prism CIRCUMSCRIBING the hard stop (front_stop_radius over the bumper span,
// rigid_radius behind it), from the shell bottom to the body top. The physics engine takes their convex hull, which near
// +/-span bridges the step between the two radii. Returns robot-frame vertices (x, y_left, z above floor).
inline std::vector<std::array<double,3>> body_prism(const Geometry& g,int sides=64){
 std::vector<std::array<double,3>> v;const double k=1/std::cos(Pi/sides);
 for(int i=0;i<sides;i++){
  const double a=2*Pi*(i+.5)/sides,az=std::remainder(a*180/Pi,360.),r=k*(std::abs(az)<=g.bumper_span_deg?front_stop_radius(g):g.rigid_radius);
  for(double z:{g.shell_bottom_z,g.body_top_z})v.push_back({r*std::cos(a),r*std::sin(a),z});
 }
 return v;
}
// LiDAR column collider (replaces the compiled 9.6 x 6.0 x 3.9 cm box that reached z 16.03 cm, above the 15.5 cm robot top):
// a `sides`-gon prism CIRCUMSCRIBING the turret circle (turret_xy, turret_radius), from the body collider's top (body_top_z,
// the lid top) to the turret top (turret_z[1] = robot_top_z_m). Above the LiDAR plane it is exactly the CAD turret, the only
// part there; below it it stands for the LiDAR tray and the LD14 housing under the turret (both at least the turret's
// footprint; the housing reaches 1 mm wider in y and further fore and aft, the tray wider still: not modelled, as before).
// Empty when the turret top is not above the body top. Robot-frame vertices (x, y_left, z above floor), cm.
inline std::vector<std::array<double,3>> lidar_column_prism(const Geometry& g,int sides=32){
 std::vector<std::array<double,3>> v;
 if(!(g.turret_radius>0)||g.turret_z[1]<=g.body_top_z+1e-6||sides<3)return v;
 const double r=g.turret_radius/std::cos(Pi/sides);
 for(int i=0;i<sides;i++){
  const double a=2*Pi*(i+.5)/sides;
  for(double z:{g.body_top_z,g.turret_z[1]})v.push_back({g.turret_xy[0]+r*std::cos(a),g.turret_xy[1]+r*std::sin(a),z});
 }
 return v;
}
// Inertia tensor in the robot frame (y LEFT) -> Unreal component frame (y RIGHT): the products with exactly one y index flip.
inline void inertia_to_unreal(const double in[3][3],double out[3][3]){
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)out[i][j]=in[i][j]*((i==1)!=(j==1)?-1.:1.);
}
// Principal moments and axes of a symmetric 3x3 inertia tensor (cyclic Jacobi). eig[k] is the moment about axes[k] (a unit
// vector in the input frame); the axes are right-handed (axes[2] = axes[0] x axes[1]), so I = sum_k eig[k] axes[k] axes[k]^T.
// Returns false unless the tensor is physical: every principal moment > 0 and each <= the sum of the other two.
inline bool principal_inertia(const double in[3][3],double eig[3],double axes[3][3]){
 double a[3][3],v[3][3]={{1,0,0},{0,1,0},{0,0,1}};
 for(int i=0;i<3;i++)for(int j=0;j<3;j++)a[i][j]=(in[i][j]+in[j][i])/2;
 for(int sweep=0;sweep<64;sweep++){
  const double off=std::abs(a[0][1])+std::abs(a[0][2])+std::abs(a[1][2]),diag=std::abs(a[0][0])+std::abs(a[1][1])+std::abs(a[2][2]);
  if(off<=1e-15*diag||off==0)break;
  for(int p=0;p<2;p++)for(int q=p+1;q<3;q++){
   if(a[p][q]==0)continue;
   const double theta=(a[q][q]-a[p][p])/(2*a[p][q]),t=(theta>=0?1.:-1.)/(std::abs(theta)+std::sqrt(theta*theta+1)),c=1/std::sqrt(t*t+1),s=t*c;
   for(int k=0;k<3;k++){const double kp=a[k][p],kq=a[k][q];a[k][p]=c*kp-s*kq;a[k][q]=s*kp+c*kq;}   // A P
   for(int k=0;k<3;k++){const double pk=a[p][k],qk=a[q][k];a[p][k]=c*pk-s*qk;a[q][k]=s*pk+c*qk;}   // P^T (A P)
   for(int k=0;k<3;k++){const double kp=v[k][p],kq=v[k][q];v[k][p]=c*kp-s*kq;v[k][q]=s*kp+c*kq;}   // V P: columns are the axes
  }
 }
 for(int k=0;k<3;k++){eig[k]=a[k][k];for(int i=0;i<3;i++)axes[k][i]=v[i][k];}
 const double det=axes[0][0]*(axes[1][1]*axes[2][2]-axes[1][2]*axes[2][1])-axes[0][1]*(axes[1][0]*axes[2][2]-axes[1][2]*axes[2][0])+axes[0][2]*(axes[1][0]*axes[2][1]-axes[1][1]*axes[2][0]);
 if(det<0)for(int i=0;i<3;i++)axes[2][i]=-axes[2][i];
 for(int k=0;k<3;k++)if(!(eig[k]>0)||eig[k]>eig[(k+1)%3]+eig[(k+2)%3]+1e-12*(eig[0]+eig[1]+eig[2]))return false;
 return true;
}
}

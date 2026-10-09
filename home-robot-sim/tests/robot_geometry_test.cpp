// Plain-C++ checks of RobotGeometry.h (no Unreal): clang++ -std=c++17 tests/robot_geometry_test.cpp -o /tmp/rg && /tmp/rg
// Covers the LiDAR column collider and the principal-axes split RobotPawn uses to set the body inertia from
// body_inertia_kg_m2 (bay B10).
#include "../Source/HomeRobotSim/RobotGeometry.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace RobotGeometry;
static double rebuild_err(const double in[3][3],const double eig[3],const double ax[3][3]){
 double worst=0;
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){double s=0;for(int k=0;k<3;k++)s+=eig[k]*ax[k][i]*ax[k][j];worst=std::max(worst,std::abs(s-in[i][j]));}
 return worst;
}
static void check_axes(const double ax[3][3]){
 for(int a=0;a<3;a++)for(int b=0;b<3;b++){double d=0;for(int i=0;i<3;i++)d+=ax[a][i]*ax[b][i];assert(std::abs(d-(a==b))<1e-12);}
 const double cx=ax[0][1]*ax[1][2]-ax[0][2]*ax[1][1],cy=ax[0][2]*ax[1][0]-ax[0][0]*ax[1][2],cz=ax[0][0]*ax[1][1]-ax[0][1]*ax[1][0];
 assert(std::abs(cx-ax[2][0])<1e-12&&std::abs(cy-ax[2][1])<1e-12&&std::abs(cz-ax[2][2])<1e-12); // right-handed
}
int main(){
 // 1. LiDAR column: synced low-profile values (cm): turret r 2.9922 at (2, 0), z 13.825..15.515, lid top 10.35.
 Geometry g;g.body_top_z=10.35;g.turret_xy={2,0};g.turret_radius=2.9922;g.turret_z={13.825,15.515};
 auto c=lidar_column_prism(g);assert(c.size()==64);
 double zmin=1e9,zmax=-1e9,rmin=1e9,rmax=0;
 for(auto& p:c){zmin=std::min(zmin,p[2]);zmax=std::max(zmax,p[2]);const double r=std::hypot(p[0]-2,p[1]);rmin=std::min(rmin,r);rmax=std::max(rmax,r);}
 assert(std::abs(zmin-10.35)<1e-12&&std::abs(zmax-15.515)<1e-12);           // lid top to the robot top, never above it (was 16.03)
 const double rc=2.9922/std::cos(Pi/32);assert(std::abs(rmin-rc)<1e-9&&std::abs(rmax-rc)<1e-9);
 // circumscribes the turret circle: every edge midpoint (the closest polygon point) is at the turret radius
 for(int i=0;i<32;i++){const auto& a=c[2*i];const auto& b=c[2*((i+1)%32)];assert(std::abs(std::hypot((a[0]+b[0])/2-2,(a[1]+b[1])/2)-2.9922)<1e-9);}
 Geometry low=g;low.turret_z={9,10};assert(lidar_column_prism(low).empty());   // turret top not above the body: no column
 Geometry none=g;none.turret_radius=0;assert(lidar_column_prism(none).empty());
 // 2. Principal axes: a diagonal tensor stays diagonal with identity axes.
 {const double I[3][3]={{.0117,0,0},{0,.0146,0},{0,0,.0220}};double e[3],ax[3][3];assert(principal_inertia(I,e,ax));
  check_axes(ax);assert(rebuild_err(I,e,ax)<1e-15);for(int k=0;k<3;k++)assert(std::abs(e[k]-I[k][k])<1e-15);}
 // 3. The synced CAD tensor (cad/mass-properties.json at 76232c4, kg m2, robot frame) in the Unreal frame.
 {const double R[3][3]={{.011697,.000089,-.000708},{.000089,.014611,.000073},{-.000708,.000073,.021975}};double U[3][3];inertia_to_unreal(R,U);
  assert(U[0][1]==-.000089&&U[1][0]==-.000089&&U[1][2]==-.000073&&U[2][1]==-.000073&&U[0][2]==-.000708&&U[1][1]==.014611);
  double e[3],ax[3][3];assert(principal_inertia(U,e,ax));check_axes(ax);const double err=rebuild_err(U,e,ax);assert(err<1e-15);
  double tr=0;for(int k=0;k<3;k++)tr+=e[k];assert(std::abs(tr-(.011697+.014611+.021975))<1e-15);   // trace kept
  std::cout.precision(9);
  std::cout<<"CAD inertia (Unreal frame) principal kg m2: "<<e[0]<<" "<<e[1]<<" "<<e[2]<<"; rebuild error "<<err<<"\n";
  for(int k=0;k<3;k++)std::cout<<" axis "<<k<<": "<<ax[k][0]<<" "<<ax[k][1]<<" "<<ax[k][2]<<"\n";}
 // 4. A rotated tensor (30 deg about z, then 20 deg about x) recovers its moments; a non-physical one is refused.
 {const double m[3]={1,2,2.5},cz=std::cos(Pi/6),sz=std::sin(Pi/6),cx=std::cos(Pi/9),sx=std::sin(Pi/9);
  const double Rz[3][3]={{cz,-sz,0},{sz,cz,0},{0,0,1}},Rx[3][3]={{1,0,0},{0,cx,-sx},{0,sx,cx}};double Rm[3][3]={},I[3][3]={};
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)Rm[i][j]+=Rx[i][k]*Rz[k][j];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)I[i][j]+=Rm[i][k]*m[k]*Rm[j][k];
  double e[3],ax[3][3];assert(principal_inertia(I,e,ax));check_axes(ax);assert(rebuild_err(I,e,ax)<1e-13);
  double s[3]={e[0],e[1],e[2]};std::sort(s,s+3);for(int k=0;k<3;k++)assert(std::abs(s[k]-m[k])<1e-12);
  const double Bad[3][3]={{1,0,0},{0,1,0},{0,0,3}};assert(!principal_inertia(Bad,e,ax));
  const double Neg[3][3]={{1,0,0},{0,-1,0},{0,0,1}};assert(!principal_inertia(Neg,e,ax));}
 std::cout<<"robot_geometry_test: all checks passed\n";
}

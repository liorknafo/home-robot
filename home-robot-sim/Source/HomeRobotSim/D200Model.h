#pragma once
#include <cmath>
#include <algorithm>

// Waveshare D200/LD14P: accuracy of a 30-reading average, NOT per-ray sigma.
// https://www.waveshare.com/product/modules/sensors/d200-lidar-kit.htm
// Dark-target operation beyond 6 m is unrated: this model returns no reading.
namespace D200Model {
inline double accuracy(double metres, bool dark) {
 if(!std::isfinite(metres)||metres<.1||metres>(dark?6.:8.))return -1.;
 if(metres<=.5)return dark?.007:.005;
 if(metres<=1.)return dark?.012:.010;
 if(dark)return metres*(metres<=4.?.012:.015);
 return metres*(metres<=6.?.010:.015);
}
inline double measure(double metres,bool dark,double unit_normal,double uniform,
                      double bias_fraction=.5,double jitter_fraction=.25,double dropout=.02) {
 const double bound=accuracy(metres,dark);
 if(bound<0||uniform<dropout)return -1.;
 // These three parameters are test assumptions, not manufacturer statistics.
 const double result=std::round((metres+bound*(bias_fraction+jitter_fraction*unit_normal))*1000.)/1000.;
 return result<.1||result>(dark?6.:8.)?-1.:result;
}
}

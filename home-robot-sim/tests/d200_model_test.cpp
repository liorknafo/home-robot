#include "../Source/HomeRobotSim/D200Model.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
 using namespace D200Model;
 assert(accuracy(.099,false)<0&&accuracy(.1,false)==.005&&accuracy(.5,true)==.007);
 assert(accuracy(.501,false)==.010&&accuracy(1,true)==.012);
 assert(std::abs(accuracy(3,false)-.03)<1e-9&&std::abs(accuracy(4,true)-.048)<1e-9);
 assert(std::abs(accuracy(5,true)-.075)<1e-9&&std::abs(accuracy(7,false)-.105)<1e-9);
 assert(accuracy(6,true)>0&&accuracy(6.001,true)<0&&accuracy(8,false)>.1&&accuracy(8.001,false)<0);
 assert(measure(2,false,0,.5,1,0,0)==2.02&&measure(2,false,0,.5,-1,0,0)==1.98);
 assert(measure(2,false,0,0,.5,.25,.02)<0&&measure(7,true,0,.5)<0);
 assert(accuracy(NAN,false)<0&&measure(-1,false,0,.5)<0);
 std::cout<<"D200 native model: range boundaries, white/dark accuracy, signed bias, dropout and invalid input passed\n";
}

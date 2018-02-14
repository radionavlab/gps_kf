#include <Eigen/Core>

namespace gps_odom {

class gpsTime {
 public:
  gpsTime(){tWeek=0;tSec=0;tFracSec=0;}
  gpsTime(double tw, double ts, double tf){tWeek=tw; tSec=ts; tFracSec=ts;}
  void setGpsTime(double tw, double ts, double tf){tWeek=tw; tSec=ts; tFracSec=ts;}
  void getGpsTime(double& twOut, double& tSecOut, double& tFracOut)
    {tWOut=tWeek; tSecOut=tSec; tFracOut=tFracSec;}

 private:
  double tWeek, tSec, tFracSec;
};

}  // namespace gps_odom


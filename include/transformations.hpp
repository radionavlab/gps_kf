#ifndef GPS_ODOM_TRANSFORM_HPP_
#define GPS_ODOM_TRANSFORM_HPP_

Eigen::Matrix3d ecef2enu_rotMatrix(Eigen::Vector3d ECEF);
Eigen::Vector3d ecef2enu(Eigen::Vector3d ECEF);

#endif // GPS_ODOM_TRANSFORM_HPP_
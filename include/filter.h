#ifndef GPS_ODOM_FILTER_H_
#define GPS_ODOM_FILTER_H_

#include <Eigen/Core>

namespace gps_odom {

class KalmanFilter {
 public:
  static const int n_states = 6;
  static const int n_meas = 3;
  typedef Eigen::Matrix<double, n_states, 1> State_t;
  typedef Eigen::Matrix<double, n_states, n_states> ProcessCov_t;
  typedef Eigen::Matrix<double, n_meas, 1> Measurement_t;
  typedef Eigen::Matrix<double, n_meas, n_meas> MeasurementCov_t;
  typedef Eigen::Matrix<double, n_meas, n_meas/2> Bmat_t;

  KalmanFilter() {}
  KalmanFilter(const State_t &state, const ProcessCov_t &initial_cov,
               const ProcessCov_t &process_noise,
               const MeasurementCov_t &meas_noise)
      : x(state), P(initial_cov), Q(process_noise), R(meas_noise) {}

  void initialize(const State_t &state, const ProcessCov_t &initial_cov,
                  const ProcessCov_t &process_noise,
                  const MeasurementCov_t &meas_noise);

  void processUpdate(double dt);
  void measurementUpdate(const Measurement_t &meas, double dt);
  void setState(const State_t &new_state) {x = new_state;}
  void setEstimateCovariance(const ProcessCov_t &new_covariance) { P = new_covariance; }
  void setProcessNoise(const ProcessCov_t &process_noise) { Q = process_noise; }
  void setMeasurementNoise(const MeasurementCov_t &meas_noise) {
    R = meas_noise;
  }

  const State_t &getState() const { return x; }
  const ProcessCov_t &getProcessNoise() const { return P; }

 private:
  State_t x;
  ProcessCov_t P, Q;
  MeasurementCov_t R;
};

}  // namespace gps_odom

#endif  // GPS_ODOM_FILTER_H_

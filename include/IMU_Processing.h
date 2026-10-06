/* 
This file is part of KILVO, the implementation of our paper.

Derived from FAST-LIVO2.
Modified for KILVO by Jixin Gao <gaojixin99@163.com>, 2026,
for more information, see <https://github.com/JixinGao/KILVO>.

If you use this code, please cite the relevant publications as
listed on the above website.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_inv_expo_cov(const double &inv_expo);
  void set_pos_cnt_cov(const V3D &pos_cnt);
  void set_imu_init_frame_num(const int &num);
  void disable_imu();
  void disable_gravity_est();
  void disable_bias_est();
  void disable_exposure_est();
  void state_prediction(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, float &lowFre_interval, SLAM_MODE &slam_mode_);
  void points_undistort(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  bool *debug_output_en;
  ofstream fout_imu;
  double IMU_mean_acc_norm;

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  V3D cov_pos_cnt;
  double cov_inv_expo;
  double first_lidar_time;
  bool imu_time_init = false;
  bool imu_need_init = true;
  M3D Eye3d;
  V3D Zero3d;
  int lidar_type;
  V3D *d_imu;
  V3D *acc_mean_Init;

private:
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  PointCloudXYZI pcl_wait_proc;
  sensor_msgs::ImuConstPtr last_imu;
  PointCloudXYZI::Ptr cur_pcl_un_;
  deque<Pose6D> IMUpose;
  vector<Pose6D> IMUpose_cacher;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double last_prop_end_time = -1.0;
  double last_lid_prop_end_time = -1.0;
  double time_last_scan;
  int init_iter_num = 1, MAX_INI_COUNT = 50;
  bool b_first_frame = true;
  bool imu_en = true;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;
  bool last_lid_en = false;
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif
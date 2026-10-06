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

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <utils/so3_math.h>
#include <utils/types.h>
#include <utils/color.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sophus/se3.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace Sophus;

#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;
#define G_m_s2 (9.81)
#define DIM_STATE (25)  // Dimension of states (Let Dim(SO(3)) = 3)
#define INIT_COV (0.01)
#define SIZE_LARGE (500)
#define SIZE_SMALL (100)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/" + name))

enum LID_TYPE
{
  AVIA = 1,
  VELO16 = 2,
  OUST64 = 3,
  L515 = 4,
  XT32 = 5,
  PANDAR128 = 6
};
enum SLAM_MODE
{
  ONLY_LO = 0,  // can loss 1 sensor, can reco 
  ONLY_LIO = 1, // can loss 1 sensor, can reco 2 sensor
  LIVO = 2,     // can loss 2 sensor, can reco 1 sensor
  KILO = 3,     // can loss 2 sensor, can reco 1 sensor
  KILVO = 4,    // can loss 3 sensor, can reco 0 sensor
  ONLY_KIO = 5  // can loss 1 sensor, can reco 2 sensor
};
enum EKF_STATE
{
  WAIT = 0,
  VIO = 1,
  LIO = 2,
  LO = 3,
  KIO = 4
};
enum ContactEvent
{
  Unknow = -1,
  Uncontact,
  Contact
};

struct MeasureGroup
{
  double vio_time;
  double lio_time;
  double kio_time;
  sensor_msgs::JointState::ConstPtr leg;
  deque<sensor_msgs::Imu::ConstPtr> imu;
  cv::Mat img;
  MeasureGroup()
  {
    vio_time = 0.0;
    lio_time = 0.0;
    kio_time = 0.0;
  };
};

struct LidarMeasureGroup
{
  double lidar_frame_beg_time;
  double lidar_frame_end_time;
  double last_lio_update_time;
  double last_kio_update_time;
  PointCloudXYZI::Ptr lidar;
  PointCloudXYZI::Ptr pcl_proc_cur;
  PointCloudXYZI::Ptr pcl_proc_next;
  deque<struct MeasureGroup> measures;
  EKF_STATE lio_vio_flg;
  int lidar_scan_index_now;

  LidarMeasureGroup()
  {
    lidar_frame_beg_time = -0.0;
    lidar_frame_end_time = 0.0;
    last_lio_update_time = -1.0;
    last_kio_update_time = -1.0;
    lio_vio_flg = WAIT;
    this->lidar.reset(new PointCloudXYZI());
    this->pcl_proc_cur.reset(new PointCloudXYZI());
    this->pcl_proc_next.reset(new PointCloudXYZI());
    this->measures.clear();
    lidar_scan_index_now = 0;
    last_lio_update_time = -1.0;
  };
};

typedef struct pointWithVar
{
  Eigen::Vector3d point_b;     // point in the lidar body frame
  Eigen::Vector3d point_i;     // point in the imu body frame
  Eigen::Vector3d point_w;     // point in the world frame
  Eigen::Matrix3d var_nostate; // the var removed the state covarience
  Eigen::Matrix3d body_var;
  Eigen::Matrix3d var;
  Eigen::Matrix3d point_crossmat;
  Eigen::Vector3d normal;
  pointWithVar()
  {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;

struct StatesGroup
{
  StatesGroup()
  {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
    this->bias_g = V3D::Zero();
    this->bias_a = V3D::Zero();
    this->gravity = V3D::Zero();
    this->pos_cnt[0] = V3D::Zero();
    this->pos_cnt[1] = V3D::Zero();
    this->inv_expo_time = 1.0;
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
    this->cov(6, 6) = 0.00001;
    this->cov.block<9, 9>(10, 10) = MD(9, 9)::Identity() * 0.00001;
  };

  StatesGroup(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->pos_cnt[0] = b.pos_cnt[0];
    this->pos_cnt[1] = b.pos_cnt[1];
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
  };

  StatesGroup &operator=(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->pos_cnt[0] = b.pos_cnt[0];
    this->pos_cnt[1] = b.pos_cnt[1];
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
    return *this;
  };

  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    StatesGroup a;
    a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.inv_expo_time = this->inv_expo_time + state_add(6, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(16, 0);
    a.pos_cnt[0] = this->pos_cnt[0] + state_add.block<3, 1>(19, 0);
    a.pos_cnt[1] = this->pos_cnt[1] + state_add.block<3, 1>(22, 0);
    a.cov = this->cov;
    return a;
  };

  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->inv_expo_time += state_add(6, 0);
    this->vel_end += state_add.block<3, 1>(7, 0);
    this->bias_g += state_add.block<3, 1>(10, 0);
    this->bias_a += state_add.block<3, 1>(13, 0);
    this->gravity += state_add.block<3, 1>(16, 0);
    this->pos_cnt[0] += state_add.block<3, 1>(19, 0);
    this->pos_cnt[1] += state_add.block<3, 1>(22, 0);
    return *this;
  };

  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
  {
    Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;
    a.block<3, 1>(19, 0) = this->pos_cnt[0] - b.pos_cnt[0];
    a.block<3, 1>(22, 0) = this->pos_cnt[1] - b.pos_cnt[1];
    return a;
  };

  void resetpose()
  {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
    this->pos_cnt[0] = V3D::Zero();
    this->pos_cnt[1] = V3D::Zero();
  }

  M3D rot_end;                              // the estimated attitude (rotation matrix) at the end lidar point
  V3D pos_end;                              // the estimated position at the end lidar point (world frame)
  V3D vel_end;                              // the estimated velocity at the end lidar point (world frame)
  double inv_expo_time;                     // the estimated inverse exposure time (no scale)
  V3D bias_g;                               // gyroscope bias
  V3D bias_a;                               // accelerator bias
  V3D gravity;                              // the estimated gravity acceleration
  std::array<V3D, 2> pos_cnt;               // foot position in world frame (left, right)
  Matrix<double, DIM_STATE, DIM_STATE> cov; // states covariance
};

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
                const Matrix<T, 3, 3> &R)
{
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++)
  {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  return move(rot_kp);
}

#endif
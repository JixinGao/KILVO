/* 
This file is part of KILVO, the implementation of our paper.

Developed for KILVO by Jixin Gao <gaojixin99@163.com>, 2026,
for more information, see <https://github.com/JixinGao/KILVO>.

If you use this code, please cite the relevant publications as
listed on the above website.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef KIO_H_
#define KIO_H_

#include "voxel_map.h"
#include <sensor_msgs/PointCloud2.h> // for pc rviz
#include <pcl_conversions/pcl_conversions.h>  // for pc rviz //pcl->ros

#define KIO_OBSDIM (7)

struct KIO_OBS
{
  double update_time;
  std::array<V3D, 2> cnt_pos_vel; // for single leg, 0->cnt_pos, 1->cnt_vel
  V3D ang_vel;
  V3D acc_vel;
};

struct REF_VAR
{
  REF_VAR()
  {
    mean_imu = V3D::Zero();  sigm_imu = V3D::Zero();
    mean_res = V3D::Zero();  sigm_res = V3D::Zero();
    mean_foot_vel = 0;       sigm_foot_vel = 0;
  };

  V3D mean_imu; V3D sigm_imu;
  V3D mean_res; V3D sigm_res;
  double mean_foot_vel; double sigm_foot_vel;
};

struct STA_VAR
{
  int check_count_imu = 50;
  int check_count_vel = 50;
  /* stationary period after contact detect */
  bool have_sta_imu[2] = {true, true};
  int  count_imu[2] = {0, 0};

  bool have_sta_footvel1[2] = {false, false};
  bool have_sta_footvel2[2] = {false, false};
  int  count_footvel[2] = {0, 0};

  void check_imu(const int &count_check, const int &leg)
  {
    count_imu[leg]++;
    if (count_imu[leg] >= count_check)
    {
      have_sta_imu[leg] = true;
      count_imu[leg] = 0;
    }
  };

  void check_footvel(const int &count_check, const int &leg)
  {
    count_footvel[leg]++;
    if (count_footvel[leg] >= count_check)
    {
      have_sta_footvel1[leg] = true;
      count_footvel[leg] = 0;
    }
  };

  void reset(const int &leg)
  {
    have_sta_imu[leg] = false;
    have_sta_footvel2[leg] = false;
    have_sta_footvel1[leg] = false;
    count_imu[leg] = 0;
    count_footvel[leg] = 0;
  }
};

struct INIT_REFSW
{
 public:
  size_t n = 0;
  V3D aver = V3D::Zero();
  V3D M2   = V3D::Zero();
  V3D std  = V3D::Zero();

 public:
  void updateData(const V3D &new_value)
  {
    n++;
    const V3D delta = new_value - aver;
    aver += delta / n;
    M2   += delta.cwiseProduct(new_value - aver);
    if (n > 1)
      std = (M2 / n).cwiseSqrt();
    else
      std = V3D::Zero();
  }
  int getNum()
  {
    return n;
  }
};

struct LEG_SD
{
  LEG_SD(){};
  LEG_SD(double timestamp_, std::array<double, 2> dist2gnd_)
  {
    timestamp = timestamp_;
    dist2gnd[0] = dist2gnd_[0];
    dist2gnd[1] = dist2gnd_[1];
  }
  double timestamp;
  std::array<double, 2> dist2gnd; // L and R.
};

struct TerrainPatch
{
  std::deque<V3D> cnt_foot_w;
  std::deque<V3D> points;
  Eigen::Vector4d pca_result;
  V3D center;
  bool en_build = false;
  int switch_num = 0;
  std::deque<V3D> cnt_foot_mean_w;
  V3D tmp_foot_cal;

  void reset()
  {
    cnt_foot_w.clear();
    points.clear();
    pca_result.setZero();
    pca_result(2) = 1.0;
    center.setZero();
    en_build = false;
    switch_num = 0;
    cnt_foot_mean_w.clear();
    tmp_foot_cal.setZero();
  };
};

class KIOManager
{
public:
  StatesGroup *state;
  StatesGroup *state_propagat;
  bool en_kio_init;
  double voxel_size;
  int octo_max_layer;
  ContactEvent flg_contact[2];
  ContactEvent flg_last_contact[2];
  double pos_cnt_cov;
  V3D vel_cnt_cov;
  bool en_adapt_cov;
  int height2terr;
  int d_raise_thres, d_conta_thres, d_swsta_thres;
  int d_raise_count_thres, d_conta_count_thres;
  bool d_init = true, leg_reco = false;
  int add_contact_det_num;
  float scale_k_sw2st;
  V3D *d_imu;
  V3D *acc_mean_Init;
  SLAM_MODE *slam_mode;
  bool *seq_phase_flg;
  bool *debug_output_en;
  ros::Publisher pub_pc;
  ros::Publisher pub_dist_foot2gnd;
  ros::Publisher pub_contact;
  ros::Publisher debug_pub_area;
  ros::Publisher pub_footd;
  ros::Publisher debug_pub_distK;
  ofstream fout_d_imu;
  ofstream fout_d_foot_vel;
  ofstream fout_ki_time;

  KIOManager();
  ~KIOManager();

  void initializeFiles();
  void initPublisher(ros::NodeHandle &nh);
  void footIMU2World(const V3D &foot_b, V3D &foot_w);
  void footIMU2World(const sensor_msgs::JointState &foot_state, V3D &footL_w, V3D &footR_w);
  void footIMU2World_vel(const V3D &foot_vel_b, V3D &foot_vel_w);
  void footIMU2World_vel(const sensor_msgs::JointState &foot_state, V3D &footL_w, V3D &footR_w);
  bool esti_plane(const std::deque<V3D> &points, Eigen::Vector4d &pca_result, Eigen::Vector3d &center);
  bool cache_contactData(const int &step, const std::array<V3D, 2> &curr_foot_w);
  void publish_realplot(const ros::Time &time, const Eigen::Vector2d &dist, const ContactEvent contact[2]);
  void publish_footpos(const ros::Time &time, const std::array<V3D, 2> &foot_w);

  void ProcessLeg(MeasureGroup &measure);
  void PredictionAdjust(const sensor_msgs::JointState &foot_state);
  V3D  StateEstimation(const KIO_OBS &obs, const int &leg);
  void EstiContact(MeasureGroup &measure, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &map);
  bool Get_ContactEvent(const sensor_msgs::Imu &imu, const sensor_msgs::JointState &joint);
  void ContactDetect(V3D &acc_, double &time);
  void UncontactDetect(V3D &foot_vel_w, int &leg, double &time);
  void FitPatch_byContact(std::deque<V3D> &points_gnd);
  void UdpPatch_byContact(std::deque<V3D> &points_gnd, std::deque<V3D> &tmp_foot_mean);
  bool SearchPoint_byLidar(std::array<V3D, 2> &foot_w, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &map);
  void FitPatch_byLidar(const VoxelOctoTree *current_octo, const int current_layer);
  Eigen::Vector2d CalDist_Foot2Ground(const std::array<V3D, 2> &foot_w);
  void RecoReset(MeasureGroup &measure);

  void Init_swCycle(const double &time);  
  void Rec_RefVar(const int &leg_, const KIO_OBS &obs_, const V3D &vel_res_, const std::vector<double> &foot_vel_b_);
  REF_VAR Cal_RefFluct();
  bool check_footdist(const int &leg);

private:
  V3D last_angvel;
  int uncontact_idx;
  bool need_CalRef_sw;
  REF_VAR ref_st;
  REF_VAR ref_sw;
  float scale_gait_fir2sec = 0.4; // the first gait is less 0.4, 0.3, 0.2 than the subsequent gait
  float scale_gait_bwd     = 0.05;// retrospective slice 0.0, 0.05, 0.1
  INIT_REFSW init_ref_sw;
  int d_doubleSta_count[2]= {0, 0};
  int d_raise_count[2] = {0, 0};
  int d_land__count[2] = {0, 0};
  int d_contact_count = 0;
  V3D d_res_max = V3D::Ones();

  V3D pred_cnt_h;       // 1.to plane value  2.en_use  3.count
  int foot_data_num;
  int contact_idx;
  bool en_upd_terrain;
  bool en_sw_beg = false;
  bool en_sw_end = false;
  double sw_cycle = 0;
  STA_VAR det_sta;

  std::deque<LEG_SD> footdist_sd;
  std::vector<std::array<V3D, 2>>   ref_imu_res; // 1.imu data  2.cnt vel data.
  std::vector<double> ref_supp_foot_vel;
  std::deque<TerrainPatch> terrain_patch;
  TerrainPatch curr_terrpatch;
  TerrainPatch init_terrpatch;
  std::deque<TerrainPatch> last_terrpatch;

  int kio_frame = 0;
  double cnt_total = 0;
  double ave_cnttotal = 0;
  double tmp_total = 0;
  double ave_total = 0;
};
typedef std::shared_ptr<KIOManager> KIOManagerPtr;
#endif // KIO_H_
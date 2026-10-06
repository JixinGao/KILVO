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

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "kio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <vikit/camera_loader.h>

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleKIO();
  void handleVIO();
  void handleLIO();
  void estiContact();
  void savePCD();
  void processImu_();

  void slam_mode_adjust();
  bool check_imu_d();
  
  bool sync_packages(LidarMeasureGroup &meas);
  bool async_packages(LidarMeasureGroup &meas);
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void legobsTransfrom(sensor_msgs::JointState::Ptr &leg_msg);
 
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void leg_cbk(const sensor_msgs::JointState::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, const ros::Publisher &pubLaserCloudFullRes_RGB, VIOManagerPtr vio_manager);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_path(const ros::Publisher pubPath);
  void readParameters(ros::NodeHandle &nh);
  void fout_state(std::ofstream& fout);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
  double  getOdometryTime();

  std::mutex mtx_buffer;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_, init_mode_;
  float highFre_interval = 0.05, lowFre_interval = 0.3;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  bool seq_phase_flg = false;
  
  bool debug_output_en = false;
  string root_dir;
  string lid_topic, imu_topic, img_topic, leg_topic;
  V3D extT;
  M3D extR;
  M3D Rfl;

  int feats_down_size = 0, max_iterations = 0;

  bool en_auto_mode = false;
  double newest_timestamp = -1.0;
  double lid_endtime = -1.0, cam_endtime = -1.0, leg_endtime = -1.0, imu_endtime = -1.0;
  bool lid_last = true,  cam_last = true,  leg_last = true,  imu_last = true;
  bool lid_deg  = false, cam_deg = false; // used for degenerate detection.
  bool lid_reco = false, cam_reco = false, leg_reco = false, imu_reco = false;
  bool kilvo_data_cache = false;

  V3D d_imu = V3D::Ones(); int smooth_count_legHz = 0;
  V3D acc_mean = V3D(0, 0, -1.0);
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0, last_timestamp_leg = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_frame_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool lidar_pushed = false, imu_en, gravity_est_en = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0; 

  //leg kinematic param
  int leg_en = 1;
  double leg_time_offset = 0.0;
  float ankle2sole_model = 0;
  float pos_cnt_cov = 0.1;
  vector<double> vel_cnt_cov;
  bool en_adapt_cov = false;
  int height2terr = 0;
  int d_raise_thres = 13;
  int d_conta_thres = 15;
  int d_raise_count_thres = 5;
  int d_conta_count_thres = 5;
  int d_swsta_thres = 5;
  int add_contact_det_num = 10;
  float scale_k_sw2st = 13; //4  13
  bool d_init = true;
  bool leg_data_update = false;

  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<sensor_msgs::JointState::ConstPtr> leg_buffer;
  deque<double> leg_time_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  vector<double> extR_urdf2Lid;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;
  PointCloudXYZI::Ptr lid_data_body_for_deg;

  ofstream fout_pre, fout_out, fout_pcd_pos;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;
  KIOManagerPtr kio_manager;

  ros::Publisher plane_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Subscriber sub_leg;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubLaserCloudFullRes_RGB;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  image_transport::Publisher pubImage;

  double last_update_time = -1.0;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  double dedisort_t = 0;
  ofstream fout_li_time;
};
#endif
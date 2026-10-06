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

#include "LIVMapper.h"

LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(nh);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  lid_data_body_for_deg.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  kio_manager.reset(new KIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<int>("common/img_en", img_en, 1);
  nh.param<int>("common/lidar_en", lidar_en, 1);
  nh.param<int>("common/leg_en", leg_en, 0);
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");
  nh.param<string>("common/leg_topic", leg_topic, "/g1/joint_state_r");
  nh.param<bool>("common/en_auto_mode", en_auto_mode, false);
  nh.param<bool>("common/gravity_align_en", gravity_align_en, false);
  nh.param<bool>("common/debug_output_en", debug_output_en, false);

  nh.param<bool>("vio/normal_en", normal_en, true);
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  nh.param<int>("vio/grid_size", grid_size, 5);
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  nh.param<int>("vio/patch_size", patch_size, 8);
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);

  nh.param<float>("kio/ankle2sole_model", ankle2sole_model, 0.0);
  nh.param<float>("kio/pos_cnt_cov", pos_cnt_cov, 0.1);
  nh.param<vector<double>>("kio/vel_cnt_cov", vel_cnt_cov, vector<double>());
  nh.param<bool>("kio/en_adapt_cov", en_adapt_cov, true);
  nh.param<int>("kio/height2terr", height2terr, 0);
  nh.param<int>("contact/d_raise_thres", d_raise_thres, 10);
  nh.param<int>("contact/d_conta_thres", d_conta_thres, 10);
  nh.param<int>("contact/d_raise_count_thres", d_raise_count_thres, 5);
  nh.param<int>("contact/d_conta_count_thres", d_conta_count_thres, 5);
  nh.param<int>("contact/d_swsta_thres", d_swsta_thres, 5);
  nh.param<int>("contact/add_contact_det_num", add_contact_det_num, 5);
  nh.param<float>("contact/scale_k_sw2st", scale_k_sw2st, 13.183727);
  nh.param<bool>("contact/d_init", d_init, true);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/leg_time_offset", leg_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);

  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());
  nh.param<vector<double>>("extrin_calib/extR_urdf2Lid", extR_urdf2Lid, vector<double>());
  nh.param<double>("debug/plot_time", plot_time, -10);
  nh.param<int>("debug/frame_cnt", frame_cnt, 6);

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);
  Rfl << MAT_FROM_ARRAY(extR_urdf2Lid);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->debug_output_en = &debug_output_en;
  vio_manager->initializeVIO();

  kio_manager->state = &_state;
  kio_manager->state_propagat = &state_propagat;
  kio_manager->pos_cnt_cov = pos_cnt_cov;
  kio_manager->vel_cnt_cov << VEC_FROM_ARRAY(vel_cnt_cov);
  kio_manager->en_adapt_cov = en_adapt_cov;
  kio_manager->height2terr = height2terr;
  kio_manager->d_raise_thres = d_raise_thres;
  kio_manager->d_conta_thres = d_conta_thres;
  kio_manager->d_raise_count_thres = d_raise_count_thres;
  kio_manager->d_conta_count_thres = d_conta_count_thres;
  kio_manager->d_swsta_thres = d_swsta_thres;
  kio_manager->d_init = d_init;
  kio_manager->add_contact_det_num = add_contact_det_num;
  kio_manager->scale_k_sw2st = scale_k_sw2st;
  kio_manager->voxel_size = voxelmap_manager->config_setting_.max_voxel_size_;
  kio_manager->octo_max_layer = voxelmap_manager->config_setting_.max_layer_;
  kio_manager->slam_mode = &slam_mode_;
  kio_manager->seq_phase_flg = &seq_phase_flg;
  kio_manager->debug_output_en = &debug_output_en;
  kio_manager->d_imu = &d_imu;
  kio_manager->acc_mean_Init = &acc_mean;
  kio_manager->initializeFiles();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_pos_cnt_cov(V3D(0.1, 0.1, 0.1));
  p_imu->set_imu_init_frame_num(imu_int_frame);
  p_imu->d_imu = &d_imu;
  p_imu->acc_mean_Init = &acc_mean;
  p_imu->debug_output_en = &debug_output_en;

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en && leg_en )? KILVO :
               (img_en && lidar_en) ? LIVO : 
               (leg_en && lidar_en)? KILO :
               (leg_en && imu_en && !lidar_en)? ONLY_KIO :
               imu_en ? ONLY_LIO : ONLY_LO;
  init_mode_ = slam_mode_;
  if (!leg_en) leg_last = false; if(!img_en) cam_last = false; if(!lidar_en) lid_last = false;
  // std::cout << "en auto mode: " << en_auto_mode << "    slam mode: " << slam_mode_ << std::endl;
}

void LIVMapper::initializeFiles() 
{
  if(pcd_save_interval > 0) fout_pcd_pos.open(std::string(ROOT_DIR) + "log/PCD/scans_pos.json", std::ios::out);
  if (debug_output_en)
  {
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
    fout_li_time.open(DEBUG_FILE_DIR("lio_time.txt"), std::ios::out);
  }
}

void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
  sub_pcl = p_pre->lidar_type == AVIA ? 
            nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this): 
            nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
  sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);
  sub_img = nh.subscribe(img_topic, 200000, &LIVMapper::img_cbk, this);
  sub_leg = nh.subscribe(leg_topic, 200000, &LIVMapper::leg_cbk, this);
  
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  pubLaserCloudFullRes_RGB = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_rgb", 100);
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
  plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
  pubImage = it.advertise("/rgb_img", 1);
  voxelmap_manager->voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
  kio_manager->initPublisher(nh);
}

void LIVMapper::fout_state(std::ofstream& fout)
{
  if (!debug_output_en || !fout.is_open())  return;
  euler_cur = RotMtoEuler(_state.rot_end);
  fout << std::setw(20) << LidarMeasures.last_lio_update_time - _first_frame_time << " " << euler_cur.transpose() * 57.3 << " "
       << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
       << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    if (LidarMeasures.lio_vio_flg == KIO)
    {
      _first_frame_time = LidarMeasures.last_kio_update_time;
    }
    else
    {
      _first_frame_time = LidarMeasures.last_lio_update_time;
    }
    p_imu->first_lidar_time = _first_frame_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu_()
{
  p_imu->state_prediction(LidarMeasures, _state, lowFre_interval ,slam_mode_);
  double t1 = omp_get_wtime();
  p_imu->points_undistort(LidarMeasures, _state, *feats_undistort);
  double t2 = omp_get_wtime();
  dedisort_t += t2 - t1;

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

  // std::cout << "[ Predict ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Predict ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Predict ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case KIO:
      handleKIO();
      break;
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);     
  publish_path(pubPath); 
}

void LIVMapper::handleKIO()
{
  leg_data_update = true;
  if (slam_mode_ == ONLY_KIO)
    kio_manager->ProcessLeg(LidarMeasures.measures.back());
  else
  {
    if (lidar_map_inited)
      kio_manager->ProcessLeg(LidarMeasures.measures.back());
  }
}

void LIVMapper::estiContact()
{
  if (LidarMeasures.lio_vio_flg == KIO && p_imu->imu_time_init)
    kio_manager->EstiContact(LidarMeasures.measures.back(), voxelmap_manager->voxel_map_);
}

void LIVMapper::handleVIO() 
{
  fout_state(fout_pre);
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr))
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  // std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;
  if (fabs((LidarMeasures.last_lio_update_time - _first_frame_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_frame_time);
  publish_frame_world(pubLaserCloudFullRes, pubLaserCloudFullRes_RGB, vio_manager);
  publish_img_rgb(pubImage, vio_manager);
  fout_state(fout_out);
}

void LIVMapper::handleLIO() 
{
  fout_state(fout_pre);
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);

  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  voxelmap_manager->lid_last = lid_last;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();
  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;
  double t2 = omp_get_wtime();

  double t3 = omp_get_wtime();
  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++)
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  // std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  double t5 = omp_get_wtime();

  if (slam_mode_ != KILVO && slam_mode_ != LIVO) publish_frame_world(pubLaserCloudFullRes, pubLaserCloudFullRes_RGB, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();

  if (debug_output_en)  fout_li_time << std::fixed << setprecision(8) << ros::Time::now() << " " << ((t4 - t0) + dedisort_t) * 1000.0 << endl;
  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + ((t4 - t0) + dedisort_t)  / frame_num;
  dedisort_t = 0.0;
  seq_phase_flg = true;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;34m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;34m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);

  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  // printf("\033[1;34m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  // printf("\033[1;34m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;34m| %-29s | %-27f |\033[0m\n", "LIO Average Total Time", aver_time_consu*1000.0);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  fout_state(fout_out);
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && pcl_wait_save->points.size() > 0)
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "log/PCD/all_raw_points.pcd";
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
    std::cout << GREEN << "RGB point cloud data saved to: " << raw_points_dir 
              << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
  }

  if (pcd_save_en && pcl_wait_save_intensity->points.size() > 0)
  {
    std::string points_intensity_dir = std::string(ROOT_DIR) + "log/PCD/all_raw_points_intensity.pcd";
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(points_intensity_dir, *pcl_wait_save_intensity);
    std::cout << GREEN << "Raw point cloud data saved to: " << points_intensity_dir 
          << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
  }
}

bool LIVMapper::check_imu_d()
{
  bool en_smooth = false;
  if (leg_last == false) {return true;} // if leg lost, 'd_imu' and 'leg_data_update' will not be update, so return true directly.
  if (d_imu == V3D::Ones()) {en_smooth = true; return en_smooth;} // init or no leg process.

  if (leg_data_update)  // 1000 Hz
  {
    leg_data_update = false;
    double curr_d = 0.6*d_imu(0) + 0.6*d_imu(1) + 1.8*d_imu(2);
    if (curr_d < d_conta_thres) smooth_count_legHz++;
    else smooth_count_legHz = 0;

    if (smooth_count_legHz >= 100) // 100 frame = 0.1s in legHz
    {
      en_smooth = true; 
    }
  }
  return en_smooth;
}

void LIVMapper::slam_mode_adjust()
{
  // std::cout << "slam mode = " << slam_mode_ << endl;
  if (!en_auto_mode || !imu_en) return; // enable auto mode and enable IMU
  if (init_mode_ != KILVO && init_mode_ != KILO && init_mode_ != LIVO) return;
  if (imu_endtime > 0){
    if (newest_timestamp - imu_endtime > 1.0) {
      ROS_WARN("imu data loss: last imu time = %lf, curr data time = %lf.", imu_endtime, newest_timestamp);
      return;
    }
  }
  else return;

  // ROS_INFO("newest time: %lf, leg: %lf, lid: %lf, cam: %lf", newest_timestamp, leg_endtime, lid_endtime, cam_endtime);
  switch (init_mode_)
  {
  case KILVO:
  {
    if (lid_endtime < 0 || cam_endtime < 0 || leg_endtime < 0) return;

    /* img, leg, lid recovery */
    if (check_imu_d())
    {
      if (!cam_last && cam_reco)  { cam_last = true; cam_reco = false; }
      if (!lid_last && lid_reco)  { lid_last = true; lid_reco = false; }
      if (!leg_last && leg_reco)  { leg_last = true; leg_reco = false; }
    }
    else
    {
      if ((!cam_last && cam_reco) && img_buffer.size() > 1) {
        ROS_WARN("Wait Img Recover when flatter.");
        img_buffer.pop_front(); img_time_buffer.pop_front();
      }
      if ((!lid_last && lid_reco) && lid_raw_data_buffer.size() > 1) {
        ROS_WARN("Wait LiD Recover when flatter.");
        lid_header_time_buffer.pop_front(); lid_raw_data_buffer.pop_front();
      }
      if ((!leg_last && leg_reco) && leg_buffer.size() > 1) {
        ROS_WARN("Wait Leg Recover when flatter.");
        leg_time_buffer.pop_front(); leg_buffer.pop_front();
      }
    }

    /* loss img, lid, leg */ 
    if (cam_last && newest_timestamp - cam_endtime > lowFre_interval) {cam_last = false; lid_raw_data_buffer.clear(); lid_header_time_buffer.clear(); imu_buffer.clear();}
    if (lid_last && newest_timestamp - lid_endtime > lowFre_interval) lid_last = false;
    if (leg_last && newest_timestamp - leg_endtime > highFre_interval) {leg_last = false; leg_buffer.clear(); leg_time_buffer.clear();}
    if (!lid_last) cam_last = false;
    if (!cam_last && !lid_last && !leg_last) return;

    if (lid_deg && !leg_last) return;
    slam_mode_ = (!lid_last || lid_deg) ? ONLY_KIO : 
                 (!leg_last && (!cam_last || cam_deg)) ? ONLY_LIO :
                 (!leg_last) ? LIVO : (!cam_last || cam_deg) ? KILO : KILVO;
    break;
  }

  case KILO:
  {
    if (lid_endtime < 0 || leg_endtime < 0) return;

    /* leg, lid recovery */
    if (check_imu_d())
    {
      if (!lid_last && lid_reco)  { lid_last = true; lid_reco = false; }
      if (!leg_last && leg_reco)  { leg_last = true; leg_reco = false; }
    }
    else
    {
      if ((!lid_last && lid_reco) && lid_raw_data_buffer.size() > 1) {
        ROS_WARN("Wait LiD Recover when flatter.");
        lid_header_time_buffer.pop_front(); lid_raw_data_buffer.pop_front();
      }
      if ((!leg_last && leg_reco) && leg_buffer.size() > 1) {
        ROS_WARN("Wait Leg Recover when flatter.");
        leg_time_buffer.pop_front(); leg_buffer.pop_front();
      }
    }

    /* loss lid, leg */
    if (lid_last && newest_timestamp - lid_endtime > lowFre_interval) {lid_last = false; voxelmap_manager->lid_recovery_count = 10;}
    if (leg_last && newest_timestamp - leg_endtime > highFre_interval) {leg_last = false;}
    if (!leg_last && !lid_last) return;

    slam_mode_ = (!leg_last) ? ONLY_LIO : (!lid_last || lid_deg) ? ONLY_KIO : KILO;
    break;
  }

  case LIVO:
  {
    if (lid_endtime < 0 || cam_endtime < 0) return;

    /* lid, cam recovery */
    if (check_imu_d())
    {
      if (!lid_last && lid_reco)  { lid_last = true; lid_reco = false; }
      if (!cam_last && cam_reco)  { cam_last = true; cam_reco = false; }
    }
    else
    {
      if ((!cam_last && cam_reco) && img_buffer.size() > 1) {
        ROS_WARN("Wait Img Recover when flatter.");
        img_buffer.pop_front(); img_time_buffer.pop_front();
      }
    }

    /* lid cam loss */
    if (lid_last && newest_timestamp - lid_endtime > lowFre_interval) lid_last = false;
    if (cam_last && newest_timestamp - cam_endtime > lowFre_interval) {cam_last = false; lid_raw_data_buffer.clear(); lid_header_time_buffer.clear(); imu_buffer.clear();}
    if (!lid_last) cam_last = false;
    if (!lid_last && !cam_last) return;

    slam_mode_ = (!cam_last || cam_deg) ? ONLY_LIO : LIVO;
    break;
  }

  default:
    break;
  }
}

void LIVMapper::run() 
{
  ros::Rate rate(5000);
  while (ros::ok()) 
  {
    ros::spinOnce();
    slam_mode_adjust();
    if (!sync_packages(LidarMeasures) && !async_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }

    handleFirstFrame();
    estiContact();
    processImu_();

    if (!p_imu->imu_time_init) continue;
    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // ROS_INFO("\033[34mGet LiDAR, its header time: %.6f, cur_head_time(fixed): %.6f\033[0m", cur_head_time - lidar_time_offset, cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  lid_endtime =  cur_head_time + (lid_raw_data_buffer.back()->points.back().curvature / 1000.0);
  if (lid_endtime > newest_timestamp) newest_timestamp = lid_endtime;
  if (!lid_last) lid_reco = true;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));

  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
  }

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // ROS_INFO("\033[34mGet LiDAR, its header time: %.6f\033[0m", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  lid_endtime = cur_head_time + (lid_raw_data_buffer.back()->points.back().curvature / 1000.0);
  if (lid_endtime > newest_timestamp) newest_timestamp = lid_endtime;
  if (!lid_last) lid_reco = true;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  if (!imu_en) return;
  if (slam_mode_ != ONLY_KIO)
  {
    if (last_timestamp_lidar < 0.0) return;
  }

  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  double timestamp = msg->header.stamp.toSec();

  // ROS_INFO("\033[33mGet IMU Data, its header time: %.6f\033[0m", timestamp);
  if (slam_mode_ != ONLY_KIO && fabs(last_timestamp_lidar - timestamp) > 0.5)
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  last_timestamp_imu = timestamp;
  imu_buffer.push_back(msg);
  if (slam_mode_ == KILO || slam_mode_ == KILVO || slam_mode_ == ONLY_KIO)
  {
    while (imu_buffer.size() > 100)
    {
      imu_buffer.pop_front();
    }
  }

  imu_endtime = timestamp;
  if (imu_endtime > newest_timestamp) newest_timestamp = imu_endtime;
  if (!imu_last) imu_reco = true;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::legobsTransfrom(sensor_msgs::JointState::Ptr &leg_msg)
{
  V3D pos_cnt_l, pos_cnt_r, vel_cnt_l, vel_cnt_r;
  /* urdf diff */
  pos_cnt_l << leg_msg->position[0], leg_msg->position[1], leg_msg->position[2]-ankle2sole_model;
  pos_cnt_r << leg_msg->position[3], leg_msg->position[4], leg_msg->position[5]-ankle2sole_model;
  vel_cnt_l << leg_msg->velocity[0], leg_msg->velocity[1], leg_msg->velocity[2];
  vel_cnt_r << leg_msg->velocity[3], leg_msg->velocity[4], leg_msg->velocity[5];

  /* urdf lid -> real lid -> imu */
  pos_cnt_l = extR * (Rfl * pos_cnt_l) + extT;
  pos_cnt_r = extR * (Rfl * pos_cnt_r) + extT;
  vel_cnt_l = extR * Rfl * vel_cnt_l;
  vel_cnt_r = extR * Rfl * vel_cnt_r;

  for (int i = 0; i < 3; i++)
  {
    leg_msg->position[i]   = pos_cnt_l(i);
    leg_msg->position[i+3] = pos_cnt_r(i);
    leg_msg->velocity[i]   = vel_cnt_l(i);
    leg_msg->velocity[i+3] = vel_cnt_r(i);
  }
}

void LIVMapper::leg_cbk(const sensor_msgs::JointState::ConstPtr &msg_in)
{
  if(!leg_en)  return;
  sensor_msgs::JointState::Ptr msg(new sensor_msgs::JointState(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec());
  double timestamp = msg->header.stamp.toSec() + leg_time_offset;
  // ROS_INFO("\033[32mGet Leg Data, its header time: %.6f\033[0m", timestamp);

  if(fabs(last_timestamp_lidar - timestamp) > 0.5)
  {
    // ROS_WARN("\033[1;32mLeg and LiDAR not synced! delta time: %lf .\033[0m\n", last_timestamp_lidar - timestamp);
  }
  legobsTransfrom(msg);
  mtx_buffer.lock();
  if(last_timestamp_leg > 0.0 && timestamp < last_timestamp_leg)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("leg loop back, offset: %lf \n\033[0", last_timestamp_leg - timestamp);
    return;
  }

  last_timestamp_leg = timestamp;
  leg_buffer.push_back(msg);
  leg_time_buffer.push_back(timestamp);

  leg_endtime = timestamp;
  if (leg_endtime > newest_timestamp) newest_timestamp = leg_endtime;
  if (!leg_last) {leg_reco = true; kio_manager->leg_reco = leg_reco;}

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));

  double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  // ROS_INFO("\033[36mGet image, its header time: %.6f\033[0m", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  cam_endtime = img_time_correct;
  if (cam_endtime > newest_timestamp) newest_timestamp = cam_endtime;
  if (!cam_last) cam_reco = true;

  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::async_packages(LidarMeasureGroup &meas)
{
  switch (slam_mode_)
  {
  case ONLY_LIO:
  case LIVO:
  case ONLY_LO:
    return false;
  } //KILO, KILVO, ONLY_KIO remain.

  // ROS_INFO("async leg time = %lf, lastkiotime = %lf, lastlidtime = %lf",  leg_time_buffer.front(), meas.last_kio_update_time, meas.last_lio_update_time);
  if (leg_buffer.empty() || imu_buffer.empty()) {leg_buffer.clear(); leg_time_buffer.clear(); return false;} 
  if (leg_time_buffer.back() < meas.last_kio_update_time || leg_time_buffer.back() < meas.last_lio_update_time) { return false; } // time retrospective
  if (imu_buffer.front()->header.stamp.toSec() > leg_time_buffer.front()) {leg_buffer.pop_front(); leg_time_buffer.pop_front(); return false;}
  if (leg_time_buffer.front() - imu_buffer.back()->header.stamp.toSec() > 0.5) {ROS_ERROR("imu/leg data timestamp error.");}
  if (meas.last_kio_update_time < 0.0) meas.last_kio_update_time = leg_time_buffer.front();

  if (slam_mode_ == KILVO) 
  {
    if (kilvo_data_cache)
      return false;
  }

  /* KILO or KILVO */ //leg and imu data
  meas.measures.clear();
  struct MeasureGroup m;
  m.imu.clear();
  m.leg = leg_buffer.front();
  m.kio_time = leg_time_buffer.front();

  mtx_buffer.lock();
  /* look for and push IMU */
  if (imu_buffer.back()->header.stamp.toSec() <= m.kio_time)
    m.imu.push_back(imu_buffer.back());
  else
  {
    for (auto it = imu_buffer.rbegin(); it != imu_buffer.rend(); ++it)
    {
      if ((*it)->header.stamp.toSec() <= m.kio_time)
      {
        m.imu.push_back(*(it));
        break;
      }
    }
  }
  leg_buffer.pop_front();
  leg_time_buffer.pop_front();
  mtx_buffer.unlock();
  sig_buffer.notify_all();

  meas.measures.push_back(m);
  if (!m.imu.empty()) {
    // printf("\033[1;32m[ curr KIO case !!! ]\033[0m [upd_time: %lf]\n", m.kio_time);
  }
  meas.lio_vio_flg = KIO;
  return true;
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if ((slam_mode_ == KILVO || slam_mode_ == LIVO) && (img_buffer.empty() && img_en)) return false;
  if ( slam_mode_ == KILVO || slam_mode_ == LIVO )
  {
    EKF_STATE last_lio_vio_flg_ = meas.lio_vio_flg;
    if (last_lio_vio_flg_ != LIO) // WAIT/KIO/VIO --> LIO update [need points]
    {
      if (lid_raw_data_buffer.empty() && lidar_en) // no lidar, have img
      {
        if (slam_mode_ == KILVO)
        {
          if (kilvo_data_cache == false && LidarMeasures.last_kio_update_time >= img_time_buffer.back())
          {
            kilvo_data_cache = true;
            printf("\033[1;35m[ Multi-Data ] arrive img's time, leg data cache.\033[0m img time: %f \n", img_time_buffer.back());
            // printf("\033[1;35m[ Multi-Data ] arrive img's time, leg data cache.\033[0m img time: %f, last_kio_update_time: %f \n", 
            //         img_time_buffer.back(), LidarMeasures.last_kio_update_time);
          }
        }
        return false;
      } // else have lid have img --> [LIO update]
    }   //else last_lio_vio_flg_ == LIO, have img --> [VIO update]
  }
  else { if (lid_raw_data_buffer.empty() && lidar_en) return false;  }

  if (slam_mode_ == KILO || slam_mode_ == KILVO || slam_mode_ == ONLY_KIO){
    if (imu_buffer.empty() && imu_en) return false;
  }
  else { if (imu_buffer.empty() && imu_en) {return false;} }

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();
      ROS_INFO("check time lidar: %.6f", meas.lidar->points.back().curvature);
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
      ROS_INFO("lid frame beg: %.6f, end: %.6f", meas.lidar_frame_beg_time, meas.lidar_frame_end_time);
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;
    }

    //ROS_INFO("last_timestamp_imu: %.6f, lidar_frame_end_time: %.6f", last_timestamp_imu, meas.lidar_frame_end_time);
    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m;
    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO;
    meas.measures.push_back(m);
    lidar_pushed = false;
    return true;

    break;
  }

  case LIVO:
  {
    /*** LIO first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    case WAIT:
    case VIO:
    {
      // printf("[ last VIO case !!! ] \n");
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n", meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())           // get IMU data
      {
        if (imu_buffer.front()->header.stamp.toSec() > m.lio_time) break;
        if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n", imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);

      while (!lid_raw_data_buffer.empty())  // get LID data
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        // seg the frame, based on the img time.
        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // printf("\033[1;34m[ curr LIO case !!! ]\033[0m [upd_time: %lf]\n", m.lio_time);
      return true;
    }

    case LIO:
    {
      // printf("[ last LIO case !!! ] \n");
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      // printf("\033[1;36m[ curr VIO case !!! ]\033[0m [upd_time: %lf]\n", m.vio_time);
      return true;
    }

    case KIO:
    {
      meas.lio_vio_flg = LIO;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case KILVO:
  {
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    case WAIT:
    case KIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      if (meas.last_kio_update_time < 0.0) meas.last_kio_update_time = leg_time_buffer.front();

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = imu_buffer.back()->header.stamp.toSec();
      double kio_newest_time = leg_time_buffer.back();

      /* check img timestamp */
      if (img_capture_time < meas.last_lio_update_time + 0.00001 || 
          img_capture_time < meas.last_kio_update_time - 0.001 * 10)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        // ROS_ERROR("[ Data Cut ] img_capture_time: %lf, lio time: %lf, kio time: %lf.", img_capture_time, meas.last_lio_update_time, meas.last_kio_update_time);
        // ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }
      if (img_capture_time > lid_newest_time) 
      { 
        // ROS_WARN("img_capture_time: %lf, lid_newest_time: %lf", img_capture_time, lid_newest_time);
        kilvo_data_cache = true;
        return false;
      }

      if (!leg_buffer.empty() && (leg_time_buffer.front() < img_capture_time))
      {
        while (leg_time_buffer.front() < img_capture_time)
        {
          leg_time_buffer.pop_front();
          leg_buffer.pop_front();
          if (leg_buffer.empty() || leg_time_buffer.empty()) break;
        }
      }
      
      struct MeasureGroup m;
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      if (imu_buffer.back()->header.stamp.toSec() <= m.lio_time)  // push one imu for predicted 
        m.imu.push_back(imu_buffer.back());
      else{
        for (auto it = imu_buffer.rbegin(); it != imu_buffer.rend(); ++it)
        {
          if ((*it)->header.stamp.toSec() < m.lio_time){
            m.imu.push_back(*(it));
            break;
          }
        }
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);

      while (!lid_raw_data_buffer.empty())  // push lid data 
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        // seg lid frame
        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // printf("\033[1;34m[ curr LIO case !!! ]\033[0m [upd_time: %lf]\n", m.lio_time);
      return true;
    }

    case LIO:
    {
      // printf("[ last LIO case !!! ] \n");
      // printf("[ Data Cut ] VIO \n");
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      kilvo_data_cache = false; // cached leg frames can be released
      printf("\033[1;35m[ Multi-Data ] img update finish, leg data release.\033[0m\n");
      meas.measures.clear();

      struct MeasureGroup m;   // sequential update
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();

      mtx_buffer.lock();
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false;
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      // printf("\033[1;36m[ curr VIO case !!! ]\033[0m [upd_time: %lf]\n", m.vio_time);
      return true;
    }

    case VIO:
    {
      printf("[ last VIO case !!! ] \n");
      meas.lio_vio_flg = KIO;
      return false;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case KILO:
  {
    /* LiDAR processing in KILO mode */
    if (imu_buffer.empty()) {lid_raw_data_buffer.clear(); lid_header_time_buffer.clear();}
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (meas.last_kio_update_time < 0.0) meas.last_kio_update_time = leg_time_buffer.front();
    if (!lidar_pushed)
    {
      meas.lidar = lid_raw_data_buffer.front();
      if (meas.lidar->points.size() <= 1) { return false; }

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();
      ROS_INFO("check time lidar: %.6f", meas.lidar->points.back().curvature);
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;
    }

    if (meas.last_kio_update_time - meas.lidar_frame_end_time > 0.01) { ROS_ERROR("LiDAR is too lagging."); }
    if (leg_en && last_timestamp_leg < meas.lidar_frame_end_time)
    { // waiting leg *updated* message needs to be larger than _lidar_frame_end_time, make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }
    struct MeasureGroup m; 
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    if (imu_buffer.back()->header.stamp.toSec() <= m.lio_time)
      m.imu.push_back(imu_buffer.back());
    else
    {
      for (auto it = imu_buffer.rbegin(); it != imu_buffer.rend(); ++it)
      {
        if ((*it)->header.stamp.toSec() < m.lio_time)
        {
          m.imu.push_back(*(it));
          break;
        }
      }
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    // printf("\033[1;34m[ curr LIO case !!! ]\033[0m [upd_time: %lf]\n", m.lio_time);
    meas.lio_vio_flg = LIO;
    meas.measures.push_back(m);
    lidar_pushed = false;
    return true;
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front();
      meas.lidar_frame_beg_time = lid_header_time_buffer.front();
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
      lidar_pushed = true;             
    }
    struct MeasureGroup m;
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false;
    meas.lio_vio_flg = LO;
    meas.measures.push_back(m);
    return true;
    break;
  }

  case ONLY_KIO:
  {
    return false;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, const ros::Publisher &pubLaserCloudFullRes_RGB, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (slam_mode_ == LIVO || slam_mode_ == KILVO)
  {
    static int pub_num = 1;
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num == pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;
        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3))
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];  //r
          pointRGB.g = pixel[1];  //g
          pointRGB.b = pixel[0];  //b
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
    else
    {
      pub_num++;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO || slam_mode_ == KILVO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);  // have img
    laserCloudmsg.header.stamp = ros::Time::now();      //.fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes_RGB.publish(laserCloudmsg);
  }
  else 
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);      // 2) no img
    laserCloudmsg.header.stamp = ros::Time::now();      //.fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    static int scan_wait_num = 0;

    if (slam_mode_ == LIVO || slam_mode_ == KILVO)
      *pcl_wait_save += *laserCloudWorldRGB;
    else
      *pcl_wait_save_intensity += *pcl_w_wait_pub;
    
    scan_wait_num++;
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "log/PCD/") + to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en)
      {
        cout << "current scan saved to /PCD/" << all_points_dir << endl;
        if (slam_mode_ == LIVO || slam_mode_ == KILVO)
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);
        }
        else
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }        
        Eigen::Quaterniond q(_state.rot_end);
        fout_pcd_pos << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                     << " " << q.z() << " " << endl;
        scan_wait_num = 0;
      }
    }
  }
  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  double time = getOdometryTime();
  if (odomAftMapped.header.stamp == ros::Time().fromSec(time)) return;
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = ros::Time().fromSec(time);
  set_posestamp(odomAftMapped.pose.pose);
  odomAftMapped.twist.twist.linear.x = _state.vel_end(0);
  odomAftMapped.twist.twist.linear.y = _state.vel_end(1);
  odomAftMapped.twist.twist.linear.z = _state.vel_end(2);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped") );
  pubOdomAftMapped.publish(odomAftMapped);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  bool pub_path_downsam = true;
  static int jjj = 0;
  if (slam_mode_ == KILVO || slam_mode_ == KILO || slam_mode_ == ONLY_KIO) pub_path_downsam = true;
  else pub_path_downsam = false;

  if (pub_path_downsam == true)
  {
    jjj++;
    if(jjj % 100 == 0)
    {
      jjj = 0;
      path.poses.push_back(msg_body_pose);
      pubPath.publish(path);
    }
  }
  else
  {
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
  }
}

double LIVMapper::getOdometryTime()
{
  double time = LidarMeasures.lio_vio_flg == KIO ? LidarMeasures.measures.back().kio_time :
                LidarMeasures.lio_vio_flg == LIO ? LidarMeasures.measures.back().lio_time : LidarMeasures.measures.back().vio_time;
  if (last_update_time > 0 && time <= last_update_time) 
    time = last_update_time + 1e-5;
  last_update_time = time;
  return time;
}

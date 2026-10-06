/* 
This file is part of KILVO, the implementation of our paper.

Developed for KILVO by Jixin Gao <gaojixin99@163.com>, 2026,
for more information, see <https://github.com/JixinGao/KILVO>.

If you use this code, please cite the relevant publications as
listed on the above website.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "kio.h"

KIOManager::KIOManager()
{
	last_angvel.setZero();
	flg_contact[0] = Contact;
	flg_contact[1] = Contact;
	flg_last_contact[0] = Contact;
	flg_last_contact[1] = Contact;
	need_CalRef_sw = true;
	uncontact_idx = -1;
	curr_terrpatch.reset();
	pred_cnt_h.setZero();
	foot_data_num = 5;
	en_kio_init = false;
	en_upd_terrain = false;
}

KIOManager::~KIOManager() {};

void KIOManager::initializeFiles()
{
  if (debug_output_en == nullptr || *debug_output_en == false) return;
  fout_d_imu.open(DEBUG_FILE_DIR("d_imu.txt"), ios::out);
  fout_d_foot_vel.open(DEBUG_FILE_DIR("d_foot_vel.txt"), ios::out);
  fout_ki_time.open(DEBUG_FILE_DIR("kio_time.txt"), ios::out);
}

void KIOManager::initPublisher(ros::NodeHandle &nh)
{
  pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/pc_in_world", 100, true);
  pub_dist_foot2gnd = nh.advertise<sensor_msgs::Imu>("/realtime_plot/dist_foot2gnd", 1000, true);
  pub_contact = nh.advertise<sensor_msgs::Imu>( "/realtime_plot/contact", 100, true);
  pub_footd = nh.advertise<sensor_msgs::Imu>("/realtime_plot/foot_d", 100, true);
  if (*debug_output_en){
    debug_pub_area = nh.advertise<sensor_msgs::Imu>("/realtime_plot/area", 100, true);
    debug_pub_distK = nh.advertise<sensor_msgs::Imu>("/realtime_plot/tmp_distK", 100, true);
  }
}

void KIOManager::publish_realplot(const ros::Time &time, const Eigen::Vector2d &dist, const ContactEvent contact[2])
{
	sensor_msgs::Imu footdist_msg; 
	footdist_msg.header.stamp = time;
	footdist_msg.header.frame_id = "camera_init";
	footdist_msg.linear_acceleration.x = dist(0);
	footdist_msg.linear_acceleration.y = dist(1);

	sensor_msgs::Imu contact_msg; 
	contact_msg.header.stamp = time;
	contact_msg.header.frame_id = "camera_init";
	contact_msg.linear_acceleration.x = contact[0];
	contact_msg.linear_acceleration.y = contact[1];

	pub_dist_foot2gnd.publish(footdist_msg);
	pub_contact.publish(contact_msg);
}

void KIOManager::publish_footpos(const ros::Time &time, const std::array<V3D, 2> &foot_w)
{
	/* pub pc to rviz */
	for (int i = 0; i < 2; i++)
	{
		if (flg_contact[i] == Contact && flg_last_contact[i] != Contact)
		{
			PointCloudXYZI::Ptr pc_in_world(new PointCloudXYZI(1, 1));
			pc_in_world->points[0].x = foot_w[i](0);
			pc_in_world->points[0].y = foot_w[i](1);
			pc_in_world->points[0].z = foot_w[i](2);
			sensor_msgs::PointCloud2 pc_msg; 
			pcl::toROSMsg(*pc_in_world, pc_msg);
			pc_msg.header.stamp = time;
			pc_msg.header.frame_id = "camera_init";
			pub_pc.publish(pc_msg);
		}
	}
}

void KIOManager::footIMU2World(const V3D &foot_b_in, V3D &foot_w)
{
	V3D foot_b(foot_b_in);
	foot_w = state->rot_end * foot_b + state->pos_end;	// imu -> world
}

void KIOManager::footIMU2World(const sensor_msgs::JointState &foot_state, V3D &footL_w, V3D &footR_w)
{
	V3D footL_b, footR_b;	// cnt pos in IMU frame.
	footL_b(0) = foot_state.position[0];
	footL_b(1) = foot_state.position[1];
	footL_b(2) = foot_state.position[2];
	footR_b(0) = foot_state.position[3];
	footR_b(1) = foot_state.position[4];
	footR_b(2) = foot_state.position[5];

	// IMU frame -> world frame
	footL_w = state->rot_end * footL_b + state->pos_end;
	footR_w = state->rot_end * footR_b + state->pos_end;
}

void KIOManager::footIMU2World_vel(const V3D &foot_vel_b, V3D &foot_vel_w) {foot_vel_w = state->rot_end * foot_vel_b;}

void KIOManager::footIMU2World_vel(const sensor_msgs::JointState &foot_state, V3D &footL_w, V3D &footR_w)
{
	V3D footL_b, footR_b;
	footL_b(0) = foot_state.velocity[0];
	footL_b(1) = foot_state.velocity[1];
	footL_b(2) = foot_state.velocity[2];
	footR_b(0) = foot_state.velocity[3];
	footR_b(1) = foot_state.velocity[4];
	footR_b(2) = foot_state.velocity[5];

	// IMU frame -> world frame
	footL_w = state->rot_end * footL_b;
	footR_w = state->rot_end * footR_b;
}

void KIOManager::RecoReset(MeasureGroup &measure)
{
	/* leg recovery mechanism, superior to direct recovery */
	V3D last_angvel_(measure.imu.back()->angular_velocity.x, measure.imu.back()->angular_velocity.y, measure.imu.back()->angular_velocity.z);
	last_angvel_ -= state->bias_g;
	last_angvel = last_angvel_;
	flg_last_contact[0] = Uncontact;	// false negative is ok.
	flg_last_contact[1] = Uncontact;
	footdist_sd.clear();
	det_sta.reset(0);
	det_sta.reset(1);
	leg_reco = false;
}

bool KIOManager::check_footdist(const int &leg)
{
	bool en_raise = false;
	float base_thre = 0.015;  // 0.02, 0.025
	en_raise = (footdist_sd.back().dist2gnd[leg] > base_thre);
	return en_raise;
}

bool KIOManager::esti_plane(const std::deque<V3D> &points, Eigen::Vector4d &pca_result, Eigen::Vector3d &center)
{
	int point_size = points.size();
	Eigen::MatrixXd A(point_size, 3);
	Eigen::MatrixXd b(point_size, 1);
	A.setZero();
	b.setOnes();
	b *= -1.0;

	for (int  j = 0; j < point_size; j++)
	{
		A(j, 0) = points[j](0);
		A(j, 1) = points[j](1);
		A(j, 2) = points[j](2);
		center += points[j];
	}
	center /= point_size;

	Eigen::Matrix<double, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
	double n = normvec.norm();
	pca_result(0) = normvec(0) / n;
	pca_result(1) = normvec(1) / n;
	pca_result(2) = normvec(2) / n;
	pca_result(3) = 1.0 / n;

	double thre = 0.1;
	for (int  j = 0; j < point_size; j++)
	{
		if (fabs(pca_result(0)*points[j](0) + pca_result(1)*points[j](1) + pca_result(2)*points[j](2)
							+ pca_result(3)) > thre)
		{
			return false;
		}
	}
	return true;
}

void KIOManager::Rec_RefVar(const int &leg_, const KIO_OBS &obs_, const V3D &vel_res_, const std::vector<double> &foot_vel_b_)
{
	std::array<V3D, 2> imu_res_data;
	imu_res_data[0] = obs_.acc_vel;
	imu_res_data[1] = vel_res_;
	ref_imu_res.push_back(imu_res_data);

	V3D foot_vel_b(foot_vel_b_[(leg_)*3], foot_vel_b_[(leg_)*3+1], foot_vel_b_[(leg_)*3+2]);
	V3D foot_vel_w;
	footIMU2World_vel(foot_vel_b, foot_vel_w);
	ref_supp_foot_vel.push_back(foot_vel_w(2));
}

REF_VAR KIOManager::Cal_RefFluct()
{
	/* calculate ref variable (imu and res) */
	REF_VAR re_imu_res;
	const size_t remove_size = ref_imu_res.size() / 4;
	ref_imu_res.erase(ref_imu_res.begin(), ref_imu_res.begin() + remove_size);
	ref_imu_res.erase(ref_imu_res.end() - remove_size, ref_imu_res.end());

	size_t n = 0;
	V3D aver_imu = V3D::Zero();	V3D aver_res = V3D::Zero();
	V3D M2_imu = V3D::Zero();		V3D M2_res = V3D::Zero();
	for (auto &it : ref_imu_res)
	{
		n++;
		//imu data // cal by Welford 
		V3D delta_imu = it[0] - aver_imu;
		aver_imu += delta_imu / n;
		M2_imu += delta_imu.cwiseProduct(it[0] - aver_imu);

		//res data
		V3D delta_res = it[1] - aver_res;
		aver_res += delta_res / n;
		M2_res += delta_res.cwiseProduct(it[1] - aver_res);
	}
	re_imu_res.mean_imu = aver_imu;
	re_imu_res.mean_res = aver_res;
	re_imu_res.sigm_imu = (M2_imu/n).cwiseSqrt();
	re_imu_res.sigm_res = (M2_res/n).cwiseSqrt();
	ref_imu_res.clear();

	/* calculate ref variable (foot vel) */
	const size_t remove_size_ = ref_supp_foot_vel.size() / 4;
	ref_supp_foot_vel.erase(ref_supp_foot_vel.begin(), ref_supp_foot_vel.begin() + remove_size_);
	ref_supp_foot_vel.erase(ref_supp_foot_vel.end() - remove_size_, ref_supp_foot_vel.end());

	size_t n_ = 0;
	double aver_foot_vel = 0; double M2_foot_vel = 0;
	for (auto &it : ref_supp_foot_vel)
	{
		n_++;
		double delta_foot_vel = it - aver_foot_vel;
		aver_foot_vel += delta_foot_vel / n_;
		M2_foot_vel += delta_foot_vel * (it - aver_foot_vel);
	}

	re_imu_res.mean_foot_vel = aver_foot_vel;
	re_imu_res.sigm_foot_vel = sqrt(M2_foot_vel / n_);
	ref_supp_foot_vel.clear();

	return re_imu_res;
}

Eigen::Vector2d KIOManager::CalDist_Foot2Ground(const std::array<V3D, 2> &foot_w)
{
	Eigen::Vector2d dist_foot = Eigen::Vector2d::Zero();
  Eigen::Vector4d pca_;
  if (*slam_mode != ONLY_KIO || height2terr == 0)  pca_ = terrain_patch.back().pca_result;
  else if (height2terr == 1) pca_ = init_terrpatch.pca_result;  // ONLY_KIO
  else {
    Eigen::Vector4d sum_tmp(0,0,0,0);                           // last terrain by LiDAR points.
    for (int i = 0; i < last_terrpatch.size(); i++) { sum_tmp += last_terrpatch[i].pca_result; }
    pca_ = sum_tmp / last_terrpatch.size();
  }
	if (pca_.head<3>().dot(-state->gravity.normalized()) < 0) {pca_ = -pca_;}
	for (int leg = 0; leg < 2; leg++){
		dist_foot(leg) = pca_(0) * foot_w[leg](0) + pca_(1) * foot_w[leg](1) + pca_(2) * foot_w[leg](2) + pca_(3);
	}
	return dist_foot;
}

void KIOManager::Init_swCycle(const double &time_)
{
	/** init, cal swing cycle **/
	sw_cycle = time_ - sw_cycle;
	cout << "sw cycle = " << sw_cycle << " sec " << endl;
	double duration_first = footdist_sd.back().timestamp - footdist_sd.front().timestamp;
	while (duration_first > sw_cycle)	// the first step is usually shorter
	{
		footdist_sd.pop_front();
		duration_first = footdist_sd.back().timestamp - footdist_sd.front().timestamp;
	}
	sw_cycle = sw_cycle + scale_gait_fir2sec * sw_cycle;
	en_sw_end = true;
}

void KIOManager::ContactDetect(V3D &acc_, double &time)
{
	V3D d_imu_sw = V3D::Ones();
	/* cal IMU div */
	if (ref_sw.sigm_imu == V3D::Zero())	// first raise leg --- use the tmp ref_sw
	{
		if (!ref_imu_res.empty())		//ref_st end, ref_sw begin
		{
			auto new_data = ref_imu_res.back();
			init_ref_sw.updateData(new_data[0]);
			if (init_ref_sw.getNum() < 100) return;
			d_imu_sw =  ((acc_ - init_ref_sw.aver).cwiseQuotient(init_ref_sw.std)).cwiseAbs();
		}
	}
	else 		// following raise leg
		d_imu_sw = ((acc_ - ref_sw.mean_imu).cwiseQuotient(ref_sw.sigm_imu)).cwiseAbs();
	*d_imu = d_imu_sw;
	
	/* cal contact by dist_foot2gnd */ //judge which leg
	double curr_d = 0.6*d_imu_sw(0) + 0.6*d_imu_sw(1) + 1.8*d_imu_sw(2);
	if (curr_d > d_conta_thres) d_contact_count++;
	else d_contact_count= 0;

	if (d_contact_count > d_conta_count_thres)
	{
		if (en_sw_beg == true && en_sw_end == false)	{ Init_swCycle(time); }
		if ((det_sta.have_sta_footvel2[0] == true || det_sta.have_sta_footvel2[1] == true) && (flg_contact[0] == Uncontact || flg_contact[1] == Uncontact))
		{
			int bwd_slice = footdist_sd.size() * scale_gait_bwd; // retrospective slice
			int end_idx = (footdist_sd.size()-1) - bwd_slice;

			/* area between foot traj and ref line */
			double area_L = 0;	double area_R = 0;
			double tN = footdist_sd[end_idx].timestamp; 	  double t0 = footdist_sd.front().timestamp;
			double k_L = (footdist_sd[end_idx].dist2gnd[0] - footdist_sd.front().dist2gnd[0]) / (tN-t0);
			double k_R = (footdist_sd[end_idx].dist2gnd[1] - footdist_sd.front().dist2gnd[1]) / (tN-t0);
			for (int i = 0; i < end_idx; i++)
			{
				auto head = footdist_sd[i];
				auto tail = footdist_sd[i+1];
				double dt = tail.timestamp - head.timestamp;

				float ref_i_head = footdist_sd.front().dist2gnd[0] + k_L*(head.timestamp - t0);
				float ref_i_tail = footdist_sd.front().dist2gnd[0] + k_L*(tail.timestamp - t0);
				area_L += 0.5 * dt * ((head.dist2gnd[0] - ref_i_head) + (tail.dist2gnd[0] - ref_i_tail));

				ref_i_head = footdist_sd.front().dist2gnd[1] + k_R*(head.timestamp - t0);
				ref_i_tail = footdist_sd.front().dist2gnd[1] + k_R*(tail.timestamp - t0);
				area_R += 0.5 * dt * ((head.dist2gnd[1] - ref_i_head) + (tail.dist2gnd[1] - ref_i_tail));
			}

      if (*debug_output_en)
			{
				// for vis script
				double pub_time_end = footdist_sd[end_idx].timestamp;
				double pub_time_beg = footdist_sd.front().timestamp;
				sensor_msgs::Imu area_msg; 
				area_msg.header.stamp = ros::Time(pub_time_end);
				area_msg.header.frame_id = "camera_init";
				area_msg.linear_acceleration.x = (area_L);
				area_msg.linear_acceleration.y = (area_R);
				area_msg.linear_acceleration.z = (pub_time_end - pub_time_beg);
				debug_pub_area.publish(area_msg);
			}

			if (area_L != 0 && area_R != 0)
			{
				if (area_L >= area_R)
				{
					flg_contact[0] = Contact;
					det_sta.reset(0);
				}
				else
				{
					flg_contact[1] = Contact;
					det_sta.reset(1);
				}
			}
		}
		det_sta.count_imu[0] = 0;
		det_sta.count_imu[1] = 0;
	}
	else
	{
		for (int leg = 0; leg < 2; leg++)
		{
			if (det_sta.have_sta_imu[leg] == false)
				det_sta.check_imu(det_sta.check_count_imu, leg);	//0.05 s static time
			else{
				if (flg_contact[leg] == Uncontact && curr_d < 2.0) {d_doubleSta_count[leg]++;}
				else d_doubleSta_count[leg] = 0;
				// both contact detect
				if (d_doubleSta_count[leg] >= 1000)	//every leg 1000 frame
				{
					d_doubleSta_count[leg] = 0;
					flg_contact[leg] = Contact;
					det_sta.reset(0);
				}
			}
		}
	}
  if (*debug_output_en)  fout_d_imu << std::fixed << setprecision(8) << time << " " << curr_d << " " << acc_.transpose() << endl;
}

void KIOManager::UncontactDetect(V3D &foot_vel_w, int &leg, double &time)
{
	double d_supp_footvel_sw = 0;

	if (ref_sw.sigm_foot_vel == 0) 
		d_supp_footvel_sw = (1/scale_k_sw2st)*abs((foot_vel_w(2) - ref_st.mean_foot_vel) / ref_st.sigm_foot_vel);
	else
		d_supp_footvel_sw = abs((foot_vel_w(2) - ref_sw.mean_foot_vel) / ref_sw.sigm_foot_vel);
	if (d_supp_footvel_sw > d_raise_thres) d_raise_count[leg]++;
	else d_raise_count[leg] = 0;

	{
		sensor_msgs::Imu footvel_msg; 
		footvel_msg.header.stamp = ros::Time(time);
		footvel_msg.header.frame_id = "camera_init";
		footvel_msg.linear_acceleration.x = (d_supp_footvel_sw);
		footvel_msg.linear_acceleration.y = (leg);
		footvel_msg.linear_acceleration.z = 0;
		pub_footd.publish(footvel_msg);
	}

	if (flg_contact[leg] == Contact)
	{
		if (!en_sw_beg)	//init, cal swing cycle
		{
			if ((d_init && d_raise_count[leg] >= d_raise_count_thres) || 
				(!d_init && check_footdist(leg)))
			{
				sw_cycle = time;
				en_sw_beg = true;
				flg_contact[leg] = Uncontact;
				det_sta.have_sta_footvel2[leg] = false;
				det_sta.have_sta_footvel1[leg] = false;
				det_sta.count_footvel[leg]= 0;
			}
		}
		else
		{
			if (d_raise_count[leg] >= d_raise_count_thres && det_sta.have_sta_imu[leg] == true)
			{
				flg_contact[leg] = Uncontact;
				det_sta.have_sta_footvel2[leg] = false;
				det_sta.have_sta_footvel1[leg] = false;
				det_sta.count_footvel[leg]= 0;
			}
		}
	}
	else if (flg_contact[leg] == Uncontact)
	{
		if (d_supp_footvel_sw > d_raise_thres && det_sta.have_sta_footvel1[leg] == false && det_sta.have_sta_imu[1-leg] == true)
			det_sta.check_footvel(det_sta.check_count_vel, leg);
		else if (det_sta.have_sta_footvel1[leg] == true && d_supp_footvel_sw < d_swsta_thres)
			det_sta.have_sta_footvel2[leg] = true;
		else if (d_supp_footvel_sw < d_raise_thres)
			det_sta.count_footvel[leg]= 0;

		/**  additional contact det, for omission **/
		if (d_supp_footvel_sw < d_swsta_thres)	d_land__count[leg]++;
		else	d_land__count[leg] = 0;
		
		if (d_land__count[leg] > add_contact_det_num)
		{
      // footdist traj fitting:
			double sum_t = 0, sum_dist = 0, sum_t_dist = 0, sum_t2 = 0;
			size_t count = 0;
			size_t recent_num = (en_sw_end == false) ? 0.2*footdist_sd.size() : 0.8*footdist_sd.size();
			double t0 = footdist_sd.back().timestamp;
			for (auto it = footdist_sd.rbegin(); it != footdist_sd.rend() && count < recent_num; ++it, ++count)
			{
				if (it+1 == footdist_sd.rend()) break;
				double dist = it->dist2gnd[leg];
				double last_dist = (it+1)->dist2gnd[leg];
				if (abs(dist - last_dist) > 0.05) {break;}
				double dt   = it->timestamp - t0;
				sum_t += dt;
				sum_dist += dist;
				sum_t_dist += dist * dt;
				sum_t2 += dt * dt;
			}
			double footdist_vel_k = (count * sum_t_dist - sum_dist * sum_t) / (count * sum_t2 - sum_t * sum_t);
			if (footdist_vel_k < -0.1)	// = footdis drop vel
			{
				if (en_sw_beg == true && en_sw_end == false) { Init_swCycle(time);}
				flg_contact[leg] = Contact;
				det_sta.reset(leg);
			}
			else if (footdist_vel_k >= 0.1) d_land__count[leg] = 0; // may at the peak of leg swing
			else{
				if (d_land__count[leg] >= 100){	flg_contact[leg] = Contact; det_sta.reset(leg); d_land__count[leg] = 0;	}
			}

      if (*debug_output_en)
			{
        // for vis script
				sensor_msgs::Imu tmp_footdist_msg; 
				tmp_footdist_msg.header.stamp = ros::Time(time);
				tmp_footdist_msg.header.frame_id = "camera_init";
				tmp_footdist_msg.linear_acceleration.x = footdist_vel_k;
				tmp_footdist_msg.linear_acceleration.y = (leg);
				tmp_footdist_msg.linear_acceleration.z = 0;
				debug_pub_distK.publish(tmp_footdist_msg);
			}
		}
	}
  if (*debug_output_en) fout_d_foot_vel << std::fixed << setprecision(8) << time << " " << leg << " " << d_supp_footvel_sw << endl;
}

bool KIOManager::Get_ContactEvent(const sensor_msgs::Imu &imu, const sensor_msgs::JointState &joint)
{
  double t0 = omp_get_wtime();
	double t = joint.header.stamp.toSec();
	/* Contact Detection */
	V3D acc_(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
	acc_ = acc_ * G_m_s2 / acc_mean_Init->norm() - state->bias_a;
	ContactDetect(acc_, t);

	/* Uncontact Detection */
  V3D foot_vel_w_; foot_vel_w_.setZero();
	for (int leg = 0; leg < 2; leg++)
	{
		V3D foot_vel_w;
		V3D foot_vel_b(joint.velocity[leg*3+0], joint.velocity[leg*3+1], joint.velocity[leg*3+2]);
		footIMU2World_vel(foot_vel_b, foot_vel_w);
		UncontactDetect(foot_vel_w, leg, t);
    foot_vel_w_ = foot_vel_w;
	}
  double t1 = omp_get_wtime();
  cnt_total += t1 - t0;
	return true;
}

bool KIOManager::cache_contactData(const int &step, const std::array<V3D, 2> &curr_foot)
{
	if (curr_terrpatch.switch_num < step)
	{
		if ((flg_contact[0] != flg_last_contact[0] && flg_contact[0] == Contact) ||
				(flg_contact[1] != flg_last_contact[1] && flg_contact[1] == Contact) )
		{
			curr_terrpatch.switch_num++;
			curr_terrpatch.cnt_foot_w.clear();
			curr_terrpatch.cnt_foot_mean_w.push_back(curr_terrpatch.tmp_foot_cal);
			curr_terrpatch.tmp_foot_cal = V3D::Zero();
			// cout << "curr_terrpatch.switch_num = " << curr_terrpatch.switch_num << " step = " << step << endl;
		}
		if (curr_terrpatch.switch_num == step) {return true;}
		if (curr_terrpatch.switch_num > 0 && contact_idx != -1)
		{
			curr_terrpatch.cnt_foot_w.push_back(curr_foot[contact_idx]);
			curr_terrpatch.tmp_foot_cal += 
					(curr_foot[contact_idx] - curr_terrpatch.tmp_foot_cal) / 
					curr_terrpatch.cnt_foot_w.size();
		}
	}
	return false;
}

void KIOManager::FitPatch_byContact(std::deque<V3D>& points_gnd)
{
  Eigen::Vector4d  tmp_pca_result;	Eigen::Vector3d  tmp_center = V3D::Zero();
  if (esti_plane(points_gnd, tmp_pca_result, tmp_center))
  {
    V3D verticalNormal(0, 0, 1);
    double angle_ = 57.29578 * acos(tmp_pca_result.head<3>().dot(verticalNormal));
    if (angle_ < 30)
    {
      curr_terrpatch.reset();
      curr_terrpatch.en_build = true;
      curr_terrpatch.center = tmp_center;
      curr_terrpatch.pca_result = tmp_pca_result;
      curr_terrpatch.cnt_foot_w.swap(points_gnd);
      terrain_patch.push_back(curr_terrpatch);
    }
    else	curr_terrpatch.reset();
  }
  else	curr_terrpatch.reset();
}

void KIOManager::UdpPatch_byContact(std::deque<V3D>& points_gnd, std::deque<V3D> &tmp_foot_mean)
{
  Eigen::Vector4d  tmp_pca_result;	Eigen::Vector3d  tmp_center = V3D::Zero();
  if (esti_plane(points_gnd, tmp_pca_result, tmp_center))
  {
    V3D last_Normal = terrain_patch.back().pca_result.head<3>();
    double angle_ = 57.29578 * acos(tmp_pca_result.head<3>().dot(last_Normal));
    if (angle_ < 10)
    {
      //update terrain_patch.back()
      terrain_patch.back().en_build = true;
      terrain_patch.back().center = tmp_center;
      terrain_patch.back().pca_result = tmp_pca_result;
      terrain_patch.back().cnt_foot_mean_w.swap(tmp_foot_mean);
      terrain_patch.back().cnt_foot_w.swap(points_gnd);
    }
  }
  en_upd_terrain = false;
  curr_terrpatch.reset();		
}

bool KIOManager::SearchPoint_byLidar(std::array<V3D, 2> &foot_w, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &map)
{
  bool find = true;
  if ((flg_contact[0] != flg_last_contact[0] && flg_contact[0] == Contact) ||
      (flg_contact[1] != flg_last_contact[1] && flg_contact[1] == Contact) )
  {
    V3D footAnchor;
    if (contact_idx != -1)  footAnchor = foot_w[contact_idx];
    else  footAnchor = (foot_w[0](2) < foot_w[1](2)) ? foot_w[0] : foot_w[1];
    
    float loc_xyz[3];
    for (int j = 0; j < 3; j++){
      loc_xyz[j] = footAnchor(j) / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = map.find(position);
    if (iter != map.end()) 
    {
      VoxelOctoTree *current_octo = iter->second;
      FitPatch_byLidar(current_octo, 0);
    }
    else  {find = false; cout << " no find voxel !" << endl;}			
  }
  return find;
}

void KIOManager::FitPatch_byLidar(const VoxelOctoTree *current_octo, const int current_layer)
{
	int max_layer_ = octo_max_layer;
	if (current_octo->plane_ptr_->is_plane_)
	{
		VoxelPlane &plane = *current_octo->plane_ptr_;
		Eigen::Vector4d tmp_pca_;	V3D tmp_center;
		tmp_pca_.head<3>() = plane.normal_;
		tmp_pca_(3) = plane.d_;
		tmp_center = plane.center_;
		// cout << "\033[31m center = " << tmp_center.transpose() << "  pca = " << tmp_pca_.transpose() << "\033[0m" << endl;

		V3D ni(tmp_pca_(0), tmp_pca_(1), tmp_pca_(2));
		double cos_the = std::abs(ni.dot(state->gravity.normalized()));
		cos_the = (cos_the < 0.0) ? 0.0 : (cos_the > 1.0) ? 1.0 : cos_the;
		double th = std::acos(cos_the) * 180.0/M_PI;
		if (th < 30.0)	// if terr normal > 30. continue using the old plane.
		{
			TerrainPatch current_patch;
			current_patch.en_build = true;
			current_patch.center = tmp_center;
			current_patch.pca_result = tmp_pca_;
			terrain_patch.push_back(current_patch);
      last_terrpatch.push_back(current_patch);
      if (last_terrpatch.size() > 5) last_terrpatch.pop_front();
			return;
		}
	}
	else
	{
		if (current_layer < max_layer_)
		{
			for (size_t leafnum = 0; leafnum < 8; leafnum++)
			{
				if (current_octo->leaves_[leafnum] != nullptr)
				{
					VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
					FitPatch_byLidar(leaf_octo, current_layer + 1);
				}
			}
			return;
		}
		else return;
	}
}

void KIOManager::EstiContact(MeasureGroup &measure, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &map)
{
	if (!en_kio_init) return;
	sensor_msgs::JointState foot_state = *measure.leg;
	V3D footL_w, footR_w;
	footIMU2World(foot_state, footL_w, footR_w);
	std::array<V3D, 2> foot_w;
	foot_w[0] = footL_w;
	foot_w[1] = footR_w;

	/* Cal Dis Foot2Ground */
	if (*slam_mode == ONLY_KIO)	//use contact points
	{
    if(SearchPoint_byLidar(foot_w, map) == false){
		if (!terrain_patch.back().en_build)
		{
			/* re-build terrain patch */
			int step = 5;
			if (cache_contactData(step, foot_w))	//collect finish
			{
				std::deque<V3D> points_gnd;
				for (int i = 1; i < curr_terrpatch.cnt_foot_mean_w.size(); i++){
					points_gnd.push_back(curr_terrpatch.cnt_foot_mean_w[i]);
				}
        FitPatch_byContact(points_gnd);
			}
		}
		else
		{
			/* update terrain patch */
			if (en_upd_terrain == false)
			{
				V3D tmp_mean_foot = 0.5 * (foot_w[0] + foot_w[1]);
				V3D last_center    = terrain_patch.back().center;
				double r = sqrt((tmp_mean_foot(0)-last_center(0)) * (tmp_mean_foot(0)-last_center(0)) +
									      (tmp_mean_foot(1)-last_center(1)) * (tmp_mean_foot(1)-last_center(1)));
				if (r > 5.0)	{en_upd_terrain = true; curr_terrpatch.reset();}
			}

			if (en_upd_terrain == true)
			{
				int step = 2;
				if (cache_contactData(step, foot_w))
				{
					std::deque<V3D> tmp_foot_mean;
					for (int i = 0; i < terrain_patch.back().cnt_foot_mean_w.size(); i++){
						tmp_foot_mean.push_back(terrain_patch.back().cnt_foot_mean_w[i]);
					}
					for (int i = 1; i < curr_terrpatch.cnt_foot_mean_w.size(); i++){
						tmp_foot_mean.push_back(curr_terrpatch.cnt_foot_mean_w[i]);
					}
					std::deque<V3D> points_gnd;
					for (int i = 0; i < tmp_foot_mean.size(); i++){
						points_gnd.push_back(tmp_foot_mean[i]);
					}
          UdpPatch_byContact(points_gnd, tmp_foot_mean);			
				}
			}
		}
    }
	}
	else	{ bool find = SearchPoint_byLidar(foot_w, map); } // use LiDAR points

	Eigen::Vector2d dist_foot2ground = CalDist_Foot2Ground(foot_w);
	std::array<double, 2> dist_foot2ground_tmp{dist_foot2ground(0), dist_foot2ground(1)};
	LEG_SD leg_sd_tmp(foot_state.header.stamp.toSec(), dist_foot2ground_tmp);
	footdist_sd.push_back(leg_sd_tmp);															// [ push ] foot dist
	if (sw_cycle != 0 && en_sw_end){																// [ pop ]  foot dist
		double duration = footdist_sd.back().timestamp - footdist_sd.front().timestamp;
		while (duration > sw_cycle)
		{
			if (footdist_sd.size() <= 1) {ROS_WARN("foot dist size error."); break;}
			footdist_sd.pop_front();
			duration = footdist_sd.back().timestamp - footdist_sd.front().timestamp;
		}
	}
	else{
		while (footdist_sd.size() > 500){
			footdist_sd.pop_front();
		}
	}

	/* Estimate Contact */
	flg_last_contact[0] = flg_contact[0];
	flg_last_contact[1] = flg_contact[1];
	bool tmp = Get_ContactEvent(*measure.imu.back(), foot_state);

	/* end processing */
	if (flg_contact[0] == Contact) contact_idx = 0;
	else if (flg_contact[1] == Contact) contact_idx = 1;
	else contact_idx = -1;

	if (terrain_patch.size() > 10)  terrain_patch.pop_front();
	publish_realplot(foot_state.header.stamp, dist_foot2ground, flg_contact);
	publish_footpos(foot_state.header.stamp, foot_w);
}

void KIOManager::PredictionAdjust(const sensor_msgs::JointState &foot_state)
{
	/* contact change check & update prediction (contact foot pos)*/
	for (int leg = 0; leg < 2; leg++)
	{
		V3D foot_pos_i(foot_state.position[3*leg], foot_state.position[3*leg+1], foot_state.position[3*leg+2]);
		V3D foot_pos_w;
		if ((flg_contact[leg] != flg_last_contact[leg]) && (flg_contact[leg] == Contact))
		{
			footIMU2World(foot_pos_i, foot_pos_w);
			state_propagat->pos_cnt[leg] = foot_pos_w;				// update foot prediction
			state_propagat->cov.block<3, 3>(19 + 3*leg, 19 + 3*leg) = M3D::Identity() * INIT_COV;

			Eigen::Vector4d pca_;
      if (*slam_mode != ONLY_KIO || height2terr == 0) pca_ = terrain_patch.back().pca_result;  // from terrain_patch (deque).
      else if (height2terr == 1)  pca_ = init_terrpatch.pca_result; // from init terrain (small environment)
      else {
        Eigen::Vector4d sum_tmp(0,0,0,0);                           // from last terrain by points.
        for (int i = 0; i < last_terrpatch.size(); i++) { sum_tmp += last_terrpatch[i].pca_result; }
        pca_ = sum_tmp / last_terrpatch.size();
      }
			float r = pca_(0) * foot_pos_w(0) + pca_(1) * foot_pos_w(1) + pca_(2) * foot_pos_w(2) + pca_(3);
			pred_cnt_h(0) = foot_pos_w(2) - r*pca_(2);				// proj to terrain patch
		}			
	}
}

V3D KIOManager::StateEstimation(const KIO_OBS &obs, const int &leg)
{	
	V3D vel_res;
	Matrix<double, DIM_STATE, DIM_STATE> G, HT_Riv_H;
	G.setZero();
	HT_Riv_H.setZero();
	MD(KIO_OBSDIM, KIO_OBSDIM) R = MD(KIO_OBSDIM, KIO_OBSDIM)::Zero();
	R.diagonal() << vel_cnt_cov(0), vel_cnt_cov(1), vel_cnt_cov(2), 
							    pos_cnt_cov,    pos_cnt_cov,    0.1*pos_cnt_cov,   pos_cnt_cov;
	int max_iterations = 2;
	for (int iteration = 0; iteration < max_iterations; iteration++)
	{
		/* construct obversation */
		Eigen::Matrix<double, KIO_OBSDIM, 1>  z;
		Eigen::Matrix<double, KIO_OBSDIM, DIM_STATE> H = Eigen::Matrix<double, KIO_OBSDIM, DIM_STATE>::Zero();

		V3D ang_vel_avr;
		M3D ang_vel_across, sub_1_cross, pos_cnt_b_cross;
		ang_vel_avr = 0.5 * (last_angvel + obs.ang_vel);
		last_angvel = obs.ang_vel;
		ang_vel_across << SKEW_SYM_MATRX(ang_vel_avr);
		V3D sub_1 = ang_vel_across * obs.cnt_pos_vel[0] + obs.cnt_pos_vel[1];
		V3D v_meas = -state->rot_end * sub_1;
		vel_res = v_meas - state->vel_end;
		sub_1_cross << SKEW_SYM_MATRX(sub_1);
		pos_cnt_b_cross << SKEW_SYM_MATRX(obs.cnt_pos_vel[0]);
		V3D pos_cnt_w;
		footIMU2World(obs.cnt_pos_vel[0], pos_cnt_w);
		Eigen::Matrix<double, 1, 1> height_res;
		height_res(0, 0) =  pos_cnt_w(2) - pred_cnt_h(0);

    if (*slam_mode != ONLY_KIO || height2terr == 0)
    {
      double height_thres = 0.1;
      if (abs(height_res(0, 0)) <= height_thres && !terrain_patch.empty() && terrain_patch.back().en_build == true)
        pred_cnt_h(1) = 1;
      else{
        pred_cnt_h(1) = 0;
        if (!terrain_patch.empty())  {terrain_patch.back().en_build = false;}
        if (curr_terrpatch.en_build == true)
          curr_terrpatch.reset();
      }
    }
    else if (height2terr == 1 || height2terr == 2)  pred_cnt_h(1) = 1;
    else {ROS_ERROR("height2terr param error.");}

		/* construct Jacobian and Res */
		H.block<3, 3>(0, 3) = state->rot_end * sub_1_cross;								// cnt vel
		H.block<3, 3>(0, 7)= -Eigen::Matrix3d::Identity();								// cnt vel
		H.block<3, 3>(3, 0) = -state->rot_end * pos_cnt_b_cross;					// cnt pos
		H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();								// cnt pos
		if (leg == 0)	H.block<3, 3>(3, 19) = -Eigen::Matrix3d::Identity();// cnt pos
		else	H.block<3, 3>(3, 22) = -Eigen::Matrix3d::Identity();	
		z.block<3, 1>(0, 0) = vel_res;																		// z
		z.block<3, 1>(3, 0) = pos_cnt_w - state_propagat->pos_cnt[leg];		// z
		
		if (leg == 0) H.block<1, 1>(6, 21) = Eigen::MatrixXd::Identity(1,1);
		else H.block<1, 1>(6, 24) = Eigen::MatrixXd::Identity(1,1);
		z.block<1, 1>(6, 0) = height_res;

		/* R re-adjust */
		V3D d_res_ = V3D::Zero(); double z_coff = 1.0;
    if (*slam_mode == ONLY_KIO) z_coff = 0.1; 
		if (en_adapt_cov && ref_sw.sigm_res != V3D::Zero())
		{
			R.diagonal() << vel_cnt_cov(0), vel_cnt_cov(1), vel_cnt_cov(2), 
								      pos_cnt_cov, pos_cnt_cov, 0.1*pos_cnt_cov, z_coff*pos_cnt_cov;
			d_res_= ((vel_res - ref_sw.mean_res).cwiseQuotient(ref_sw.sigm_res)).cwiseAbs();
			d_res_max = d_res_max.cwiseMax(d_res_);
			V3D coeffi(10.0, 10.0, 10.0);
			d_res_ = d_res_.cwiseProduct(coeffi).cwiseQuotient(d_res_max);
			if (d_res_(0) < 10.0*0.1) d_res_(0) = 1.0;
			if (d_res_(1) < 10.0*0.1) d_res_(1) = 1.0;
			if (d_res_(2) < 10.0*0.1) d_res_(2) = 1.0;
			V3D R_vel = vel_cnt_cov.cwiseProduct(d_res_);
			R.diagonal().head<3>() =  R_vel;
		}
		if (pred_cnt_h(1) == 0)
			R.block<1, 1>(6, 6) = 1e6 * Eigen::Matrix<double, 1, 1>::Identity();

		HT_Riv_H = H.transpose() * R.inverse() * H;
		MD(DIM_STATE, DIM_STATE) &&K_1 = (HT_Riv_H + state->cov.inverse()).inverse();
		G = K_1 * HT_Riv_H;
		auto vec = (*state) - (*state_propagat);
		MD(DIM_STATE, 1) solution = -K_1 * H.transpose() * R.inverse() * z  - vec + G * vec;
		(*state) += solution;

		bool flg_EKF_converged = false;
		auto &&rot_add = solution.block<3, 1>(0, 0);
  	auto &&t_add = solution.block<3, 1>(3, 0);
		if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }
		if (iteration == max_iterations || flg_EKF_converged) break;
	}

	state->cov -= G * state->cov;
	return vel_res;
}

void KIOManager::ProcessLeg(MeasureGroup &measure)
{
  double t0 = omp_get_wtime();
	sensor_msgs::JointState foot_state_i = *measure.leg;
	if (!en_kio_init)
	{
		for (int  leg = 0; leg < 2; leg++)
		{
			/* 1. init prediction */
			V3D foot_pos_i(foot_state_i.position[3*leg], foot_state_i.position[3*leg+1], foot_state_i.position[3*leg+2]);
			V3D foot_pos_w;
			footIMU2World(foot_pos_i, foot_pos_w);
			state_propagat->pos_cnt[leg] = foot_pos_w;
			state_propagat->cov.block<3, 3>(19 + 3*leg, 19 + 3*leg) = M3D::Identity() * INIT_COV;
			pred_cnt_h(2) = pred_cnt_h(2) + 1;
			pred_cnt_h(0) += (foot_pos_w(2) - pred_cnt_h(0)) / pred_cnt_h(2);

			/* 2. init terrain patch at least 3 points */
			if (curr_terrpatch.cnt_foot_w.size() < 3 * foot_data_num)
			{curr_terrpatch.cnt_foot_w.push_back(foot_pos_w);}
		}
		int curr_cnt_size = curr_terrpatch.cnt_foot_w.size();
		V3D foot_fwd = 0.5 * (curr_terrpatch.cnt_foot_w.back() + curr_terrpatch.cnt_foot_w[curr_cnt_size-2]);
		foot_fwd(0) += 1.0;
		if (curr_terrpatch.cnt_foot_w.size() < 3 * foot_data_num) { curr_terrpatch.cnt_foot_w.push_back(foot_fwd); }
		/* 3. init ref variable and patch */
		if (ref_imu_res.size() > 500)
		{
			Eigen::Vector4d pca_tmp; V3D center_tmp = V3D::Zero(); 
			if (esti_plane(curr_terrpatch.cnt_foot_w, pca_tmp, center_tmp)) 
			{
				curr_terrpatch.en_build = true;
				for (int i = 0; i < 3; i++)
				{	curr_terrpatch.cnt_foot_mean_w.push_back(curr_terrpatch.cnt_foot_w[i]); }
				curr_terrpatch.center = center_tmp;
				curr_terrpatch.pca_result = pca_tmp;
				terrain_patch.push_back(curr_terrpatch);
        init_terrpatch = curr_terrpatch;
			}
			ref_st = Cal_RefFluct();
			en_kio_init = true;
		}
	}
	else
	{
		if (leg_reco == true) RecoReset(measure);
		PredictionAdjust(foot_state_i);
	}

	for (int leg = 0; leg < 2; leg++)
	{
		if (ref_imu_res.size() > 10 && uncontact_idx != -1 && flg_contact[uncontact_idx] == Contact)
		{
			ref_sw = Cal_RefFluct();
			float scale_k_sw2st_ = ref_sw.sigm_foot_vel / ref_st.sigm_foot_vel;
			need_CalRef_sw = false;
			uncontact_idx = -1;
			// cout << "scale_k_sw2st_ = " << scale_k_sw2st_ << endl;
		}

		if (flg_contact[leg] == Contact)
		{
			V3D vel_res;
			KIO_OBS obs;
			int idx = leg*3;
			V3D ang_vel_ad (measure.imu.back()->angular_velocity.x, measure.imu.back()->angular_velocity.y, measure.imu.back()->angular_velocity.z);
			V3D acc_vel_ad (measure.imu.back()->linear_acceleration.x, measure.imu.back()->linear_acceleration.y, measure.imu.back()->linear_acceleration.z);
			ang_vel_ad -= state->bias_g;
			acc_vel_ad =  acc_vel_ad * G_m_s2 / acc_mean_Init->norm() - state->bias_a;

			obs.update_time = measure.kio_time;
			obs.cnt_pos_vel[0] << foot_state_i.position[idx], foot_state_i.position[1+idx], foot_state_i.position[2+idx];//contact foot pos
			obs.cnt_pos_vel[1] << foot_state_i.velocity[idx], foot_state_i.velocity[1+idx], foot_state_i.velocity[2+idx];//contact foot vel
			obs.ang_vel = ang_vel_ad;
			obs.acc_vel = acc_vel_ad;
			vel_res = StateEstimation(obs, leg);
			
			if (need_CalRef_sw && flg_contact[1-leg] == Uncontact)	// for Ref Swing (Recording)
			{
				if (uncontact_idx == -1) uncontact_idx = 1-leg;				//record uncontact leg
				Rec_RefVar(leg, obs, vel_res, foot_state_i.velocity);
			}

			if (!en_kio_init && leg == 0)														// for Ref Stance (Recording)
				Rec_RefVar(leg, obs, vel_res, foot_state_i.velocity);
		}
	}

  double t1 = omp_get_wtime();
  if (*debug_output_en) fout_ki_time << std::fixed << setprecision(8) << ros::Time::now() << " " << (t1 - t0)*1000.0 << endl;
  tmp_total += t1 - t0;
  if (*seq_phase_flg == true)
  {
    kio_frame++;
    ave_total = ave_total * (kio_frame - 1) / kio_frame + tmp_total / kio_frame;
    ave_cnttotal = ave_cnttotal * (kio_frame - 1) / kio_frame + cnt_total / kio_frame;
    tmp_total = 0; cnt_total = 0;
    *seq_phase_flg = false;
    printf("\033[1;32m| %-29s | %-27f |\033[0m\n", "KIO Average Total Time", ave_total*1000.0);
    printf("\033[1;32m| %-29s | %-27f |\033[0m\n", "CNT DET Average Total Time", ave_cnttotal*1000.0);
  }
}
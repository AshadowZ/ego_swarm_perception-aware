#include "plan_env/grid_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0); // resolution默认是0.1，10cm？
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
  node_.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
  node_.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  if( mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.11), &GridMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_.fromSec(0);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());

  // 初始化我定义的变量
  // camera FoV params
  far_ = 4.5; // 相机可以往前看4.5米，最大的raycast长度也是4.5
  // normals of hyperplanes
  const double up_down_ang = 0.524; // 30度
  const double left_right_ang = 0.698; // 40度 
  n_top_ << 0.0, sin(M_PI_2 - up_down_ang), cos(M_PI_2 - up_down_ang);
  n_bottom_ << 0.0, -sin(M_PI_2 - up_down_ang), cos(M_PI_2 - up_down_ang);
  n_left_ << sin(M_PI_2 - left_right_ang), 0.0, cos(M_PI_2 - left_right_ang);
  n_right_ << -sin(M_PI_2 - left_right_ang), 0.0, cos(M_PI_2 - left_right_ang);
  
  // vertices of FoV assuming zero pitch

  // lefttop_ << -far_ * tan(left_ang), -far_ * sin(top_ang), far_; // 坐标系定的好奇怪阿
  // leftbottom_ << -far_ * sin(left_ang), far_ * sin(top_ang), far_;
  // righttop_ << far_ * sin(right_ang), -far_ * sin(top_ang), far_;
  // rightbottom_ << far_ * sin(right_ang), far_ * sin(top_ang), far_;

  // FOV : [80, 60] 
  lefttop_ << -far_ * tan(left_right_ang), -far_ * tan(up_down_ang), far_; // 坐标系定的好奇怪阿
  leftbottom_ << -far_ * tan(left_right_ang), far_ * tan(up_down_ang), far_;
  righttop_ << far_ * tan(left_right_ang), -far_ * tan(up_down_ang), far_;
  rightbottom_ << far_ * tan(left_right_ang), far_ * tan(up_down_ang), far_;
  
  cast_flags_ = CastFlags(1000000);
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1; // 是用于加速raycast用的变量，如何加速先不管

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id); // cache_voxel_为后面所用
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v+=skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u+=skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i]; // 遍历处理投影点

    // set flag for projected point

    if (!isInMap(pt_w))
    { 
      pt_w = closetPointInMap(pt_w, md_.camera_pos_); // 如果投影点不在地图边界内，则将其替换为地图边界内最近的点

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) // 检查光线最大长度，如果超过就将点截断为最大长度光线上的点
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1); // 这个就是障碍点了
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_); // 将投影点和相机位置输入raycast

    while (raycaster.step(ray_pt)) // 迭代计算相机位置和投影点之间光线上的点
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0); // 显然相机到投影点之间的点不是障碍

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true; // 后面代码要判断的标志位

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_; // grid的更新范围：以相机为中心的正方体

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_); // 这个就学长说的每次看到grid上有障碍就往上面加点概率
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  // 得到局部地图的边界范围
  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);
  
  // 扩展区域的边界范围
  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range
  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_; // unknown_flag_是0.01常数
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_; // 将其标记为未知
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) // 对啊
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (md_.last_occ_update_time_.toSec() < 1.0 ) md_.last_occ_update_time_ = ros::Time::now();
  
  if (!md_.occ_need_update_)
  {
    if ( md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_ )
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f", 
        ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true; // timeout为true表示里程计或深度图丢失
    }
    return;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}

/*-------------------------------------------------我定义的函数-------------------------------------------------------------*/
// 返回body to camera的变换矩阵
Eigen::Matrix4d GridMap::getCamToBody()
{
  return md_.cam2body_;
}

double GridMap::calcInformationGain(const Eigen::Vector3d& pt, const double& yaw)
{
  auto start_time = ros::Time::now();

  Eigen::Matrix3d R_wb;
  R_wb << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0; // yaw角对应的旋转矩阵R_wb

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity(); // T_wb变换矩阵表示当前的机体位姿，body to world
  T_wb.block(0, 0, 3, 3) = R_wb;
  T_wb.block(0, 3, 3, 1) = pt;  
  Eigen::Matrix4d T_bc_ = md_.cam2body_;
  Eigen::Matrix4d T_wc = T_wb * T_bc_; // T_wc表示当前的相机位姿，camera to world
  Eigen::Matrix3d R_wc = T_wc.block(0, 0, 3, 3); // 当前相机姿态
  Eigen::Vector3d t_wc = T_wc.block(0, 3, 3, 1); // 当前相机位置

  // rotate camera seperating plane normals
  vector<Eigen::Vector3d> normals = { n_top_, n_bottom_, n_left_, n_right_ }; // 用于表示相机FOV的四条法线，是单位变量
  for (auto& n : normals) // 把这四条法线转换到相机坐标系下
  {
    n = R_wc * n; 
  }  
  Eigen::Vector3i lbi, ubi; // 用于存放AABB的边界框index
  calcFovAABB(R_wc, t_wc, lbi, ubi); // 计算相机视野（FOV）在空间中的边界框（Axis-Aligned Bounding Box，AABB）

  Eigen::Vector3i pt_idx, ray_id;
  Eigen::Vector3d check_pt, ray_pt;
  const int factor = 4; // 子采样的间隔
  double gain = 0; // 信息增益：FOV中能看到的未知格子的个数
  Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - mp_.map_origin_ / mp_.resolution_; // offset只是用来把pos转换为index（raycast中）
  RayCaster raycaster; // raycaster类用于检查视线上是否有障碍物

  for (int x = lbi[0]; x <= ubi[0]; ++x) 
    for (int y = lbi[1]; y <= ubi[1]; ++y) 
      for (int z = lbi[2]; z <= ubi[2]; ++z) { // 遍历AABB中的所有点
        // subsampling
        if (!(x % factor == 0 && y % factor == 0 && z % factor == 0)) continue; // 子采样，每次跳4个
        // check visibility of unknown cells in FOV, 1: accessible, 2: blocked
        pt_idx << x, y, z;
        if (!isUnknown(pt_idx)) continue; // 检查该点是否已知，已知则直接跳过
        indexToPos(pt_idx, check_pt);
        if (!insideFoV(check_pt, pt, normals)) continue; // 检查该点是否在FOV内，不在则直接跳过

        char flag = cast_flags_.getFlag(pt_idx); // 是不是已经raycast过了，加速算法
        if (flag == 1) {  // visited visible cell, fetch visibility directly
          gain += 1;
        } else if (flag == 0) {  // unvisited cell, should raycast
          char result = 1;
          raycaster.setInput(check_pt / mp_.resolution_, pt / mp_.resolution_);
          while (raycaster.step(ray_pt)){
            ray_id(0) = ray_pt(0) + offset(0); // 这个ray_pt不能用posToIndex转啊，切记
            ray_id(1) = ray_pt(1) + offset(1);
            ray_id(2) = ray_pt(2) + offset(2);
            if(getOccupancy(ray_id) == 1){
              result = 2;
              break;
            }
          }
          if(result == 1) {
            gain += 1;
          }
          cast_flags_.setFlag(pt_idx, result);
        }   
      }

  auto end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;
  // ROS_INFO("executing time: %.4f sec", duration.toSec());

  return gain;
}

// 带轨迹权重版本的信息增益实现
// 看上去也没比不带权重的版本好上多好，反而exp会导致数值计算的问题，本来的gain是有实际意义的
// emmmmm，好像效果确实会更好一些？
double GridMap::calcInformationGain(const Eigen::Vector3d& pt, const double& yaw,
                                                                const Eigen::MatrixXd& ctrl_pts) 
{
  auto start_time = ros::Time::now();

  Eigen::Matrix3d R_wb;
  R_wb << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0; // yaw角对应的旋转矩阵R_wb

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity(); // T_wb变换矩阵表示当前的机体位姿，body to world
  T_wb.block(0, 0, 3, 3) = R_wb;
  T_wb.block(0, 3, 3, 1) = pt;  
  Eigen::Matrix4d T_bc_ = md_.cam2body_;
  Eigen::Matrix4d T_wc = T_wb * T_bc_; // T_wc表示当前的相机位姿，camera to world
  Eigen::Matrix3d R_wc = T_wc.block(0, 0, 3, 3); // 当前相机姿态
  Eigen::Vector3d t_wc = T_wc.block(0, 3, 3, 1); // 当前相机位置

  // rotate camera seperating plane normals
  vector<Eigen::Vector3d> normals = { n_top_, n_bottom_, n_left_, n_right_ }; // 用于表示相机FOV的四条法线，是单位变量
  for (auto& n : normals) // 把这四条法线转换到相机坐标系下
  {
    n = R_wc * n; 
  }  
  Eigen::Vector3i lbi, ubi; // 用于存放AABB的边界框index
  calcFovAABB(R_wc, t_wc, lbi, ubi); // 计算相机视野（FOV）在空间中的边界框（Axis-Aligned Bounding Box，AABB）

  Eigen::Vector3i pt_idx, ray_id;
  Eigen::Vector3d check_pt, ray_pt;
  const int factor = 4; // 子采样的间隔
  double gain = 0; // 信息增益：FOV中能看到的未知格子的个数
  Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - mp_.map_origin_ / mp_.resolution_; // offset只是用来把pos转换为index（raycast中）
  RayCaster raycaster; // raycaster类用于检查视线上是否有障碍物
  pair<double, double> dist12(0, 0); // 点与轨迹的横向与纵向距离
  double lambda1_ = 2.0, lambda2_ = 1.0;

  for (int x = lbi[0]; x <= ubi[0]; ++x) 
    for (int y = lbi[1]; y <= ubi[1]; ++y) 
      for (int z = lbi[2]; z <= ubi[2]; ++z) { // 遍历AABB中的所有点
        // subsampling
        if (!(x % factor == 0 && y % factor == 0 && z % factor == 0)) continue; // 子采样，每次跳4个
        // check visibility of unknown cells in FOV, 1: accessible, 2: blocked
        pt_idx << x, y, z;
        if (!isUnknown(pt_idx)) continue; // 检查该点是否已知，已知则直接跳过
        indexToPos(pt_idx, check_pt);
        if (!insideFoV(check_pt, pt, normals)) continue; // 检查该点是否在FOV内，不在则直接跳过

        char flag = cast_flags_.getFlag(pt_idx); // 是不是已经raycast过了，加速算法
        if (flag == 1) {  // visited visible cell, fetch visibility directly
          distToPathAndCurPos(check_pt, ctrl_pts, dist12, false);
          // cout << "gain of this point is : " << 100 * exp(-lambda1_ * dist12.first - lambda2_ * dist12.second) << endl;
          gain += 1000 * exp(-lambda1_ * dist12.first - lambda2_ * dist12.second);
        } else if (flag == 0) {  // unvisited cell, should raycast
          char result = 1;
          raycaster.setInput(check_pt / mp_.resolution_, pt / mp_.resolution_);
          while (raycaster.step(ray_pt)){
            ray_id(0) = ray_pt(0) + offset(0); // 这个ray_pt不能用posToIndex转啊，切记
            ray_id(1) = ray_pt(1) + offset(1);
            ray_id(2) = ray_pt(2) + offset(2);
            if(getOccupancy(ray_id) == 1){
              result = 2;
              break;
            }
          }
          if(result == 1) {
            distToPathAndCurPos(check_pt, ctrl_pts, dist12, false);
            // cout << "gain of this point is : " << exp(-lambda1_ * dist12.first - lambda2_ * dist12.second) << endl;
            gain += 1000 * exp(-lambda1_ * dist12.first - lambda2_ * dist12.second);
          }
          cast_flags_.setFlag(pt_idx, result);
        }   
      }

  // std::cout << "size of ctrl_pts: " << ctrl_pts.rows() << "rows x " << ctrl_pts.cols() << "cols" << std::endl;

  auto end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;
  ROS_INFO("executing time: %.4f sec", duration.toSec());

  return gain;
}

// 接收相机位置和yaw角，返回在该viewpoint的信息增益
// 好了，让我们接着慢慢修这个傻逼函数的bug
// emmmmm，看上去大致是work了
double GridMap::calcInfoGain(const Eigen::Vector3d& pt, const double& yaw) 
{

  // compute camera transform
  auto start_time = ros::Time::now();
  Eigen::Matrix3d R_wb;
  R_wb << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0; // yaw角对应的旋转矩阵R_wb

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity(); // T_wb变换矩阵表示当前的机体位姿，body to world
  T_wb.block(0, 0, 3, 3) = R_wb;
  T_wb.block(0, 3, 3, 1) = pt;  
  Eigen::Matrix4d T_bc_ = md_.cam2body_;
  Eigen::Matrix4d T_wc = T_wb * T_bc_; // T_wc表示当前的相机位姿，camera to world
  Eigen::Matrix3d R_wc = T_wc.block(0, 0, 3, 3); // 当前相机姿态
  Eigen::Vector3d t_wc = T_wc.block(0, 3, 3, 1); // 当前相机位置

  // rotate camera seperating plane normals
  vector<Eigen::Vector3d> normals = { n_top_, n_bottom_, n_left_, n_right_ }; // 用于表示相机FOV的四条法线，是单位变量
  for (auto& n : normals) // 把这四条法线转换到相机坐标系下
  {
    n = R_wc * n; 
  }  
  Eigen::Vector3i lbi, ubi; // 用于存放AABB的边界框index
  calcFovAABB(R_wc, t_wc, lbi, ubi); // 计算相机视野（FOV）在空间中的边界框（Axis-Aligned Bounding Box，AABB）
  // 看上去到这里应该都是对的

  Eigen::Vector3i pt_idx, ray_id;
  Eigen::Vector3d check_pt, ray_pt;
  const int factor = 4; // 子采样的间隔
  double gain = 0; // 信息增益：FOV中能看到的未知格子的个数
  Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - mp_.map_origin_ / mp_.resolution_; // offset只是用来把pos转换为index（raycast中）
  RayCaster raycaster; // raycaster类用于检查视线上是否有障碍物

  // debug变量
  int pt_num_AABB = 0, unknown_pt_num = 0;

  for (int x = lbi[0]; x <= ubi[0]; ++x) {
    for (int y = lbi[1]; y <= ubi[1]; ++y) {
      for (int z = lbi[2]; z <= ubi[2]; ++z) { // 遍历AABB中的所有点

        if (!(x % factor == 0 && y % factor == 0 && z % factor == 0)) continue; // 子采样，隔4个一看
        pt_idx << x, y, z;
         
        pt_num_AABB++; // 遍历的点个数，看上去点的个数是对的

        if (!isUnknown(pt_idx)) continue; // 检查该点是否已知，已知则直接跳过
        indexToPos(pt_idx, check_pt);
      
        if (!insideFoV(check_pt, pt, normals)) continue; // 检查该点是否在FOV内，不在则直接跳过，这一步滤了2/3的点，符合直觉
        unknown_pt_num++; // 遍历的点中未知点个数
        // 过到这里的点是FOV内的子采样的、未知的点

        bool visible = 1;
        raycaster.setInput(check_pt / mp_.resolution_, pt / mp_.resolution_);
        while (raycaster.step(ray_pt)){
          ray_id(0) = ray_pt(0) + offset(0); // 这个ray_pt不能用posToIndex转啊，切记
          ray_id(1) = ray_pt(1) + offset(1);
          ray_id(2) = ray_pt(2) + offset(2);
          if(getOccupancy(ray_id) == 1){
            visible = false;
            break;
          }
        }
        if (visible) gain += 1; // 如果我这个FOV里的点未知且可以被看到，它就会提高地图的信息增益

      }
    }
  }
  auto end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;
  // ROS_INFO("executing time: %.4f sec", duration.toSec()); // 执行时间2-4ms，raycast的点多时最高可到9ms，可尝试加速

  // std::cout << "pt_num_AABB: " << pt_num_AABB << std::endl;
  // std::cout << "unknown_pt_num: " << unknown_pt_num << std::endl;
  return gain;
}

// // 一个关于Occ函数的测试代码
// // 每次调用这个函数时返回位置周围长方体中occ的grid个数
// double GridMap::calcInfoGain(const Eigen::Vector3d& pt, const double& yaw)
// {
//   double gain = 0;
//   double count1 = 0, count2 = 0;
//   Eigen::Vector3i pt_idx, temp;
//   Eigen::Vector3d temp_d;
//   posToIndex(pt, pt_idx);
//   // cout << "min_occupancy_log_ : " << mp_.min_occupancy_log_ << endl;
//   // cout << "clamp_min_log_ : " << mp_.clamp_min_log_<< endl;
//   // cout << "clamp_max_log_ : " << mp_.clamp_max_log_<< endl;
  
//   for (int x = 0; x < 50; ++x) {
//     for (int y = 0; y < 50; ++y) { // 遍历当前平面的2500个点

//         if (!(x % 4 == 0 && y % 4 == 0)) continue;  // 子采样
//         temp << pt_idx(0) + x - 25, pt_idx(1) + y - 25, 10;
//         cout << "pos: " << temp.transpose() << endl;
//         cout << "occupancy_buffer_[pos]: " << int(md_.occupancy_buffer_[toAddress(temp)]) << endl;
//         cout << "occupancy_buffer_inflate_[pos]: " << int(md_.occupancy_buffer_inflate_[toAddress(temp)]) << endl;
//         indexToPos(temp, temp_d);
        
//         if(getOccupancy(temp_d)) count1++;
//         // if(getInflateOccupancy(temp_d)) 
//         // if(isKnownOccupied(temp))

//         if(getInflateOccupancy(temp_d)) count2++;
//         // if(getInflateOccupancy(temp_d)) 
//         // if(isKnownOccupied(temp))

//     }
//   }
//   cout << "count1: " << count1 <<"." << "count2: " << count2 << endl;
//   return gain;
// }


void GridMap::calcFovAABB(const Eigen::Matrix3d& R_wc, const Eigen::Vector3d& t_wc,
                                 Eigen::Vector3i& lb, Eigen::Vector3i& ub) 
{
  // axis-aligned bounding box(AABB) of camera FoV
  vector<Eigen::Vector3d> vertice(5);
  vertice[0] = R_wc * lefttop_ + t_wc; // 将相机FOV的四个顶点转换相机坐标系下
  vertice[1] = R_wc * leftbottom_ + t_wc;
  vertice[2] = R_wc * righttop_ + t_wc;
  vertice[3] = R_wc * rightbottom_ + t_wc;
  vertice[4] = t_wc; // 这个点是不能删的

  // cout << "the left_top is : " << vertice[0].transpose() << endl; // 地图就高两米，这个相机的视野上下有尼玛4米多？
  // cout << "the left_bottom is : " << vertice[1].transpose() << endl;
  // cout << "the right_top is : " << vertice[2].transpose() << endl;
  // cout << "the right_bottom is : " << vertice[3].transpose() << endl;

  Eigen::Vector3d lbd, ubd;
  axisAlignedBoundingBox(vertice, lbd, ubd); // lbd代表边界框的最小点，ubd代表边界框的最大点
  boundBox(lbd, ubd); // 不知道我实现的bounding box对不对，周指导那个是"bound box of explored map"

  // cout << "lbd : " << lbd.transpose() << "ubd : " << ubd.transpose() << endl;
  posToIndex(lbd, lb); // 将pos转换为index
  posToIndex(ubd, ub);
  boundIndex(lb);
  boundIndex(ub);
  // cout << "lb : " << lb.transpose() << "ub : " << ub.transpose() << endl;
}

void GridMap::axisAlignedBoundingBox(const vector<Eigen::Vector3d>& points, Eigen::Vector3d& lb,
                                            Eigen::Vector3d& ub) {
  lb = points.front(); // 初始化为点集中的第一个点
  ub = points.front();
  for (auto p : points) {
    lb = lb.array().min(p.array());
    ub = ub.array().max(p.array());
  }
}

bool GridMap::insideFoV(const Eigen::Vector3d& pw, const Eigen::Vector3d& pc,
                              const vector<Eigen::Vector3d>& normals){
  Eigen::Vector3d dir = pw - pc;
  if (dir.norm() > far_) { // 在能见范围之外
    return false;
  }
  for (auto n : normals) { // normal好像是相机FOV的四条法向量
    if (dir.dot(n) < 0.1) {
      return false;
    }
  }
  return true;
}

void GridMap::boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up){
    for (int i = 0; i < 3; ++i) {
      low[i] = max(low[i], mp_.map_min_boundary_[i]);
      up[i] = min(up[i], mp_.map_max_boundary_[i]);
    }
}

void GridMap::initCastFlag(const Eigen::Vector3d& pos) {
  Eigen::Vector3d vec(far_, far_, rightbottom_[1]);
  Eigen::Vector3i lbi, ubi;
  posToIndex(pos - vec, lbi);
  posToIndex(pos + vec, ubi);
  cast_flags_.reset(lbi, ubi);
}

// 似乎是返回某个点距离轨迹的横向距离和纵向距离
void GridMap::distToPathAndCurPos(const Eigen::Vector3d& check_pt, const Eigen::MatrixXd& ctrl_pts,
                          std::pair<double, double>& dists, bool debug = false)
{
  double min_squ = numeric_limits<double>::max();
  int idx = -1;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    Eigen::Vector3d ctrl_pt = ctrl_pts.row(i);
    double squ = (ctrl_pt - check_pt).squaredNorm();
    if (squ < min_squ) {
      min_squ = squ;
      idx = i;
    }
  }
  dists.first = sqrt(min_squ);
  dists.second = 0.0;
  for (int i = 0; i < idx; ++i) {
    dists.second += (ctrl_pts.row(i + 1) - ctrl_pts.row(i)).norm();
  }
  if (debug)
    std::cout << "pos: " << check_pt.transpose() << ", d1: " << dists.first << ", d2: " << dists.second
              << std::endl;
}
    
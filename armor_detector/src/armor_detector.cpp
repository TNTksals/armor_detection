#include "armor_detector/armor_detector.h"
#include <pluginlib/class_list_macros.h>
#include "opencv_extended/opencv_extended.h"
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(Processor, nodelet::Nodelet)

/* --- Processor --- */
Processor::Processor()
{
}

Processor::~Processor() = default;

void Processor::paramconfigCB(armor_detector::ArmorParamConfig& config, uint32_t level)
{
  brightness_threshold_ = config.brightness_threshold;
  light_min_area_ = config.light_min_area;
  light_contour_min_solidity_ = config.light_contour_min_solidity;
  light_max_ratio_ = config.light_max_ratio;
  light_max_angle_diff_ = config.light_max_angle_diff;
  light_max_height_diff_ratio_ = config.light_max_height_diff_ratio;
  light_max_y_diff_ratio_ = config.light_max_y_diff_ratio;
  light_min_x_diff_ratio_ = config.light_min_x_diff_ratio;
  armor_min_aspect_ratio_ = config.armor_min_aspect_ratio;
  armor_max_aspect_ratio_ = config.armor_max_aspect_ratio;
}

void Processor::onInit()
{
  ros::NodeHandle & nh = getPrivateNodeHandle();
  initialize(nh);
}

void Processor::initialize(ros::NodeHandle& nh)
{
  nh_ = ros::NodeHandle(nh, "armor_detector");
  param_cfg_srv_ = new dynamic_reconfigure::Server<armor_detector::ArmorParamConfig>(ros::NodeHandle(nh_, "armor_condition"));
  param_cfg_cb_ = boost::bind(&Processor::paramconfigCB, this, _1, _2);
  param_cfg_srv_->setCallback(param_cfg_cb_);
  it_ = std::make_shared<image_transport::ImageTransport>(nh_);
  cam_sub_ = it_->subscribeCamera("/galaxy_camera/image_raw", 1, &Processor::onFrameCb, this);
  image_pub_ = it_->advertise("/image_process/output_image", 1);
}

void Processor::onFrameCb(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &info)
{
  cv_image_ = cv_bridge::toCvCopy(img, "bgr8");
  cam_info_ = info;
  this->detectArmor(cv_image_->image, cam_info_);
  this->publishImage();
}

void Processor::publishImage() const
{
  image_pub_.publish(cv_image_->toImageMsg());
}



/* --- LightDescriptor --- */
LightDescriptor::LightDescriptor()
{
}

LightDescriptor::LightDescriptor(const cv::RotatedRect &light_rect)
{
  height_ = light_rect.size.height;
  width_ = light_rect.size.width;
  area_ = light_rect.size.area();
  angle_ = light_rect.angle;
  center_ = light_rect.center;
}

LightDescriptor::~LightDescriptor() = default;

const LightDescriptor &LightDescriptor::operator=(const LightDescriptor &light_descriptor)
{
  this->height_ = light_descriptor.height_;
  this->width_ = light_descriptor.width_;
  this->area_ = light_descriptor.area_;
  this->angle_ = light_descriptor.angle_;
  this->center_ = light_descriptor.center_;
  return *this;
}

cv::RotatedRect LightDescriptor::rect() const
{
  return cv::RotatedRect(center_, cv::Size2f(width_, height_), angle_);
}



/* --- ArmorDescriptor --- */
ArmorDescriptor::ArmorDescriptor()
{
}

ArmorDescriptor::~ArmorDescriptor() = default;

ArmorDescriptor::ArmorDescriptor(const LightDescriptor& left_light, const LightDescriptor& right_light)
{
  using namespace cv;
  light_pairs_[0] = left_light.rect();
  light_pairs_[1] = right_light.rect();

  Size left_light_size(static_cast<int>(light_pairs_[0].size.width), static_cast<int>(light_pairs_[1].size.height * 2));
  Size right_light_size(static_cast<int>(light_pairs_[1].size.width), static_cast<int>(light_pairs_[1].size.height * 2));
  RotatedRect left_light_rect(light_pairs_[0].center, left_light_size, light_pairs_[0].angle);
  RotatedRect right_light_rect(light_pairs_[1].center, right_light_size, light_pairs_[1].angle);
  // 获取装甲板左边顶点
  Point2f pts_left[4];
  left_light_rect.points(pts_left);
  Point2f upper_left = pts_left[2];
  Point2f lower_left = pts_left[3];
  // 获取装甲板右边顶点
  Point2f pts_right[4];
  right_light_rect.points(pts_right);
  Point2f upper_right = pts_right[1];
  Point2f lower_right = pts_right[0];
  // 保存装甲板顶点
  armor_vertex_.resize(4);
  armor_vertex_[0] = upper_left;
  armor_vertex_[1] = upper_right;
  armor_vertex_[2] = lower_right;
  armor_vertex_[3] = lower_left;
}



void Processor::detectArmor(const cv::Mat3b & src_img, const sensor_msgs::CameraInfoConstPtr &cam_info)
{
  using namespace cv;
  using std::vector;

  src_img_ = src_img;
  vector<Mat1b> color_channels;
  split(src_img_, color_channels);
  gray_img_ = color_channels[0] - color_channels[2];
  threshold(gray_img_, thresh_img_, brightness_threshold_, 255, THRESH_BINARY | cv::THRESH_OTSU);

  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
  dilate(thresh_img_, thresh_img_, kernel);

  vector<vector<Point>> light_contours;
  findContours(thresh_img_, light_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<LightDescriptor> light_infos;
  for (const auto & contour: light_contours)
  {
    // 筛掉面积太小的轮廓
    float light_contour_area = contourArea(contour);
    if (light_contour_area < light_min_area_)
      continue;
    // 椭圆拟合生成相应的旋转矩形
    RotatedRect light_rect = fitEllipse(contour);
    // 矫正旋转矩形
    cvex::adjustRect(light_rect);
    // 宽长比，实心度筛选灯条
    if (light_rect.size.width / light_rect.size.height > light_max_ratio_ ||
        light_contour_area / light_rect.size.area() < light_contour_min_solidity_)
        continue;
    // 保存灯条
    light_infos.emplace_back(LightDescriptor(light_rect));
  }

  // Lambda函数, 按灯条中心x从小到大排序
  sort(light_infos.begin(), light_infos.end(),
       [](const LightDescriptor &light_descriptor1, const LightDescriptor &light_descriptor2)
       {
         return light_descriptor1.center_.x < light_descriptor2.center_.x;
       });

  size_t i;
  size_t j;
  unsigned int light_infos_size = light_infos.size();
  for (i = 0; i < light_infos_size; i++)
  {
    for (j = i + 1; j < light_infos_size; j++)
    {
      const LightDescriptor &left_light = light_infos[i];
      const LightDescriptor &right_light = light_infos[j];
      // 左灯和右灯的角度差
      float angle_diff = abs(left_light.angle_ - right_light.angle_);
      // 左灯和右灯的长度差之比
      float length_diff = abs(left_light.height_ - right_light.height_) / max(left_light.height_, right_light.height_);

      if (angle_diff > light_max_angle_diff_ ||
          length_diff > light_max_height_diff_ratio_)
          continue;

      // 左右灯条中心距离
      float light_distance = cvex::distance(left_light.center_, right_light.center_);
      // 左右灯条长度的均值
      float mean_length = (left_light.height_ + right_light.height_) / 2;
      // y差值的比率
      float y_diff = abs(left_light.center_.y - right_light.center_.y);
      float y_diff_ratio = y_diff / mean_length;
      // x差值的比率
      float x_diff = abs(left_light.center_.x - right_light.center_.x);
      float x_diff_ratio = x_diff / mean_length;
      // 灯条的距离与长度的比值（也就是嫌疑装甲板长和宽的比值）
      float ratio = light_distance / mean_length;

      if (y_diff_ratio > light_max_y_diff_ratio_ ||
          x_diff_ratio < light_min_x_diff_ratio_ ||
          ratio < armor_min_aspect_ratio_ ||
          ratio > armor_max_aspect_ratio_)
          continue;

      ArmorDescriptor armor(left_light, right_light);
      armors_.emplace_back(armor);
      if (!armors_.empty())
        break;
    }
  }

  for (const auto & armor : armors_)
  {
    // 框出装甲板
    cvex::makeFrame(&src_img_, armor.armor_vertex_);
    calculateArmorPose(armor, cam_info);
  }

  armors_.clear();
}

void Processor::calculateArmorPose(const ArmorDescriptor &armor, const sensor_msgs::CameraInfoConstPtr &cam_info)
{
  using namespace cv;
  // 相机内参矩阵
  Mat camera_matrix(3, 3, CV_64F, (void *)cam_info->K.data());
  // 相机畸变矩阵
  Mat dist_coeffs(1, 5, CV_64F, (void *)cam_info->D.data());
  // 平移向量
  Mat_<double> r_vec;
  // 旋转向量
  Mat_<double> t_vec;
  // 世界坐标系中三维点的三维坐标
  std::vector<Point3f> object_points(4);
  object_points[0] = Point3f(-117.5, -63.5, 0);
  object_points[1] = Point3f(117.5, -63.5, 0);
  object_points[2] = Point3f(117.5, 63.5, 0);
  object_points[3] = Point3f(-117.5, 63.5, 0);
  // 三维点在图像中对应的像素点的二维坐标
  std::vector<Point2f> image_points(4);
  image_points[0] = armor.armor_vertex_[0];
  image_points[1] = armor.armor_vertex_[1];
  image_points[2] = armor.armor_vertex_[2];
  image_points[3] = armor.armor_vertex_[3];
  // 单目位姿估计
  solvePnP(object_points, image_points, camera_matrix, dist_coeffs, r_vec, t_vec);
  // 世界坐标系中三维点的三维坐标
  std::vector<Point3f> axis_points_3d(5);
  axis_points_3d[0] = Point3f(100, 0, 0);
  axis_points_3d[1] = Point3f(0, 100, 0);
  axis_points_3d[2] = Point3f(0, 0, 100);
  axis_points_3d[3] = Point3f(0, 0, 0);
  axis_points_3d[4] = Point3f(0, 0, -30);
  // 三维坐标点在像素坐标系中估计的坐标
  std::vector<Point2f> axis_points_2d;
  // 单目投影
  projectPoints(axis_points_3d, r_vec, t_vec, camera_matrix, dist_coeffs, axis_points_2d);
  // 绘制三维坐标轴
  line(src_img_, axis_points_2d[3], axis_points_2d[0], cv::Scalar(0, 0, 255), 2);
  putText(src_img_, "X", axis_points_2d[0], 1, 1.5, Scalar(0, 0, 255), 2);
  line(src_img_, axis_points_2d[3], axis_points_2d[1], cv::Scalar(0, 255, 0), 2);
  putText(src_img_, "Y", axis_points_2d[1], 1, 1.5, Scalar(0, 255, 0), 2);
  line(src_img_, axis_points_2d[3], axis_points_2d[2], cv::Scalar(255, 0, 0), 2);
  putText(src_img_, "Z", axis_points_2d[2], 1, 1.5, Scalar(255, 0, 0), 2);
  line(src_img_, axis_points_2d[3], axis_points_2d[4], cv::Scalar(0, 255, 255), 1);
  // 将旋转向量转换为旋转矩阵
  Mat_<double> r_mat;
  Rodrigues(r_vec, r_mat);
  // 转换为tf内置的矩阵类型
  tf::Matrix3x3 tf_r_mat(r_mat(0, 0), r_mat(0, 1), r_mat(0, 2),
                         r_mat(1, 0), r_mat(1, 1), r_mat(1, 2),
                         r_mat(2, 0), r_mat(2, 1), r_mat(2, 2));
  // 获取欧拉角
  double roll;
  double pitch;
  double yaw;
  tf_r_mat.getRPY(roll, pitch, yaw);
  // 获取四元数
  tf::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  // 设置旋转矩阵
  tf::Transform transform;
  transform.setRotation(quaternion);
  // 设置平移向量
  tf::Vector3 tf_t_vec;
  tf_t_vec.setX(t_vec(0, 0));
  tf_t_vec.setY(t_vec(0, 1));
  tf_t_vec.setZ(t_vec(0, 2));
  transform.setOrigin(tf_t_vec);
  // 构造tf::StampedTransform, 用于存放变换关系
  tf::StampedTransform stamped_transform(transform, ros::Time::now(), "camera_optional_frame", "target_ID:1");
  // 广播tf
  static tf::TransformBroadcaster transform_broadcaster;
  transform_broadcaster.sendTransform(stamped_transform);
}
# pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include "armor_detector/ArmorParamConfig.h"
#include <vector>
#include <array>


/*
 *   @ This class describes the info of lights, including angle level, width, length, score
 */
class LightDescriptor
{
public:
  float height_;
  float width_;
  float area_;
  float angle_;
  cv::Point2f center_;
public:
  LightDescriptor();
  LightDescriptor(const cv::RotatedRect& light_rect);
  ~LightDescriptor();
  const LightDescriptor& operator=(const LightDescriptor& light_descriptor);
  cv::RotatedRect rect() const;
};


/*
 *   @ This class describes the armor information
 */
class ArmorDescriptor
{
public:
  std::array<cv::RotatedRect, 2> light_pairs_;
  std::vector<cv::Point2f> armor_vertex_;
public:
  ArmorDescriptor();
  ArmorDescriptor(const LightDescriptor& left_light, const LightDescriptor& right_light);
  ~ArmorDescriptor();
};


/*
 *   @ This class process image and publish pose
 */
class Processor : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber cam_sub_;
  cv_bridge::CvImagePtr cv_image_;
  sensor_msgs::CameraInfoConstPtr cam_info_;
  image_transport::Publisher image_pub_;
  void onFrameCb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info);

  cv::Mat3b src_img_;
  cv::Mat1b gray_img_;
  cv::Mat1b thresh_img_;
  std::vector<ArmorDescriptor> armors_;

  // Pre-treatment
  int brightness_threshold_{};

  // Filter lights
  float light_min_area_{};
  float light_contour_min_solidity_{};
  float light_max_ratio_{};

  // Filter pairs
  float light_max_angle_diff_{};
  float light_max_height_diff_ratio_{};
  float light_max_y_diff_ratio_{};
  float light_min_x_diff_ratio_{};

  // Filter armor
  float armor_min_aspect_ratio_{};
  float armor_max_aspect_ratio_{};

public:
  Processor();
  ~Processor() override;
  void initialize(ros::NodeHandle& nh);
  void detectArmor(const cv::Mat3b & src_img, const sensor_msgs::CameraInfoConstPtr &cam_info);
  void calculateArmorPose(const ArmorDescriptor &armor, const sensor_msgs::CameraInfoConstPtr &cam_info);
  void publishImage() const;

  dynamic_reconfigure::Server<armor_detector::ArmorParamConfig> * param_cfg_srv_;
  dynamic_reconfigure::Server<armor_detector::ArmorParamConfig>::CallbackType param_cfg_cb_;
  void paramconfigCB(armor_detector::ArmorParamConfig &config, uint32_t level);

  void onInit() override;
};

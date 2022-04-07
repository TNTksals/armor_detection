#pragma once

#include <dynamic_reconfigure/server.h>
#include "armor_detector/ArmorParamConfig.h"

/*
 *   @ This struct store all parameters of armor detection
 */
struct ArmorParam
{
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

    /*
    dynamic_reconfigure::Server<armor_detector::ArmorParamConfig> * param_cfg_srv_;
    dynamic_reconfigure::Server<armor_detector::ArmorParamConfig>::CallbackType param_cfg_cb_;

    void paramconfigCB(armor_detector::ArmorParamConfig &config, uint32_t level)
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

    ArmorParam()
    {
        ros::NodeHandle nh;
        param_cfg_srv_ = new dynamic_reconfigure::Server<armor_detector::ArmorParamConfig>(ros::NodeHandle(nh, "armor_condition"));
        param_cfg_cb_ = boost::bind(&ArmorParam::paramconfigCB, this, _1, _2);
        param_cfg_srv_->setCallback(param_cfg_cb_);
    }
     */
};
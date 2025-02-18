#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <raspicam.hpp>
#include <memory>

class RaspicamPublisher : public rclcpp::Node
{
public:
  RaspicamPublisher(rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~RaspicamPublisher();

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_img_compressed;

  RASPIVID_STATE state;

  OnSetParametersCallbackHandle::SharedPtr set_parameter_callback_handle;

  void declareIntParameter(
    const std::string & name, int & current_value, int min_value, int max_value,
    std::string description = "");
  void declareFloatParameter(
    const std::string & name, double & current_value, double min_value, double max_value,
    std::string description = "");
  void declareFloatParameter(
    const std::string & name, float & current_value, double min_value, double max_value,
    std::string description = "");
  void declareBoolParameter(
    const std::string & name, int & current_value,
    std::string description = "");
  void declareParameters();
  rcl_interfaces::msg::SetParametersResult onSetParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  void onImageRaw(const uint8_t * start, const uint8_t * end);
  void onImageCompressed(const uint8_t * start, const uint8_t * end);

};

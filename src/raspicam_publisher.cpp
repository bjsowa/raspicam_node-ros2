#include <raspicam_publisher.hpp>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

RaspicamPublisher::RaspicamPublisher(rclcpp::NodeOptions options)
: Node("raspicam_node", "", options.use_intra_process_comms(true))
{
  declareParameters();

  set_parameter_callback_handle = add_on_set_parameters_callback(
    std::bind(
      &RaspicamPublisher::onSetParameter, this,
      std::placeholders::_1));

  if (state.enable_raw_pub) {
    pub_img = create_publisher<sensor_msgs::msg::Image>("image", rclcpp::QoS(1));
  }

  pub_img_compressed = create_publisher<sensor_msgs::msg::CompressedImage>(
    "image/compressed", rclcpp::QoS(1));

  init_cam(
    state,
    std::bind(
      &RaspicamPublisher::onImageRaw, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &RaspicamPublisher::onImageCompressed, this, std::placeholders::_1,
      std::placeholders::_2));

  start_capture(state);
}

RaspicamPublisher::~RaspicamPublisher()
{
  close_cam(state);
}

void RaspicamPublisher::declareIntParameter(
  std::string name, int & current_value, int min_value, int max_value, std::string description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = std::move(description);

  rcl_interfaces::msg::IntegerRange range;
  range.from_value = min_value;
  range.to_value = max_value;
  range.step = 1;

  descriptor.integer_range.emplace_back(std::move(range));

  declare_parameter(name, current_value, descriptor);
  get_parameter(name, current_value);
}

void RaspicamPublisher::declareParameters()
{
  // Read-only parameters
  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  declare_parameter("width", 320, read_only_descriptor);
  declare_parameter("height", 240, read_only_descriptor);
  declare_parameter("quality", 80, read_only_descriptor);
  declare_parameter("framerate", 90, read_only_descriptor);
  declare_parameter("publish_raw", true, read_only_descriptor);

  get_parameter("width", state.width);
  get_parameter("height", state.height);
  get_parameter("quality", state.quality);
  get_parameter("framerate", state.framerate);
  get_parameter("publish_raw", state.enable_raw_pub);

  // Dynamic parameters
  declareIntParameter(
    "sharpness", state.camera_parameters.sharpness, -100, 100, "Image sharpness");
  declareIntParameter(
    "contrast", state.camera_parameters.contrast, -100, 100, "Image contrast");
  declareIntParameter(
    "brightness", state.camera_parameters.brightness, 0, 100, "Image brightness");
  declareIntParameter(
    "saturation", state.camera_parameters.saturation, -100, 100, "Image saturation");
  RCLCPP_INFO(get_logger(), "%d", state.camera_parameters.ISO);
  declareIntParameter(
    "ISO", state.camera_parameters.ISO, 0, 1600, "Capture ISO");
  declareIntParameter(
    "exposure_compensation", state.camera_parameters.exposureCompensation, -10, 10,
    "Exposure compensation");

}

rcl_interfaces::msg::SetParametersResult RaspicamPublisher::onSetParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  for (const auto & parameter : parameters) {
    RCLCPP_INFO_STREAM(get_logger(), "Set parameter: " << parameter.get_name());
    std::string name = parameter.get_name();
    if (name == "sharpness") {
      if (raspicamcontrol_set_sharpness(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
    } else if (name == "contrast") {
      if (raspicamcontrol_set_contrast(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
    } else if (name == "brightness") {
      if (raspicamcontrol_set_brightness(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
    } else if (name == "saturation") {
      if (raspicamcontrol_set_saturation(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
    } else if (name == "ISO") {
      if (raspicamcontrol_set_ISO(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
    } else if (name == "exposure_compensation") {
      if (raspicamcontrol_set_exposure_compensation(
          state.camera_component.get(),
          parameter.as_int()) != 0)
      {
        return result;
      }
    }
  }

  result.successful = true;
  return result;
}

void RaspicamPublisher::onImageRaw(const uint8_t * start, const uint8_t * end)
{
  const auto tnow = now();

  sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
  msg->header.frame_id = "camera";
  msg->header.stamp = tnow;
  msg->encoding = "rgb8";
  msg->data.insert(msg->data.end(), start, end);
  msg->height = state.height;
  msg->width = state.width;
  pub_img->publish(std::move(msg));
}

void RaspicamPublisher::onImageCompressed(const uint8_t * start, const uint8_t * end)
{
  const auto tnow = now();

  sensor_msgs::msg::CompressedImage::UniquePtr msg(new sensor_msgs::msg::CompressedImage());
  msg->header.frame_id = "camera";
  msg->header.stamp = tnow;
  msg->format = "jpeg";
  msg->data.insert(msg->data.end(), start, end);
  pub_img_compressed->publish(std::move(msg));
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(RaspicamPublisher)

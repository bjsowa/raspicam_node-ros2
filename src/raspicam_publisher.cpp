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
  const std::string & name, int & current_value, int min_value, int max_value,
  std::string description)
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

void RaspicamPublisher::declareFloatParameter(
  const std::string & name, double & current_value, double min_value, double max_value,
  std::string description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = std::move(description);

  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min_value;
  range.to_value = max_value;
  range.step = 0.0;

  descriptor.floating_point_range.emplace_back(std::move(range));

  declare_parameter(name, current_value, descriptor);
  get_parameter(name, current_value);
}

void RaspicamPublisher::declareFloatParameter(
  const std::string & name, float & current_value, double min_value, double max_value,
  std::string description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = std::move(description);

  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min_value;
  range.to_value = max_value;
  range.step = 0.0;

  descriptor.floating_point_range.emplace_back(std::move(range));

  declare_parameter(name, static_cast<double>(current_value), descriptor);
  get_parameter(name, current_value);
}

void RaspicamPublisher::declareBoolParameter(
  const std::string & name, int & current_value, std::string description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = std::move(description);

  bool value = current_value != 0;
  declare_parameter(name, value, descriptor);
  get_parameter(name, value);
  current_value = value ? 1 : 0;
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
  declareIntParameter(
    "ISO", state.camera_parameters.ISO, 0, 1600, "Capture ISO");
  declareIntParameter(
    "exposure_compensation", state.camera_parameters.exposureCompensation, -10, 10,
    "Exposure compensation");
  declareIntParameter(
    "rotation", state.camera_parameters.rotation, 0, 359,
    "Image rotation (in degrees)");

  declareFloatParameter("roi.x", state.camera_parameters.roi.x, 0.0, 1.0);
  declareFloatParameter("roi.y", state.camera_parameters.roi.y, 0.0, 1.0);
  declareFloatParameter("roi.w", state.camera_parameters.roi.w, 0.0, 1.0);
  declareFloatParameter("roi.h", state.camera_parameters.roi.h, 0.0, 1.0);
  declareFloatParameter("analog_gain", state.camera_parameters.analog_gain, 0.0, 15.0);
  declareFloatParameter("digital_gain", state.camera_parameters.analog_gain, 0.0, 64.0);

  declareBoolParameter(
    "video_stabilization", state.camera_parameters.videoStabilisation,
    "Video stabilization");
  declareBoolParameter(
    "hflip", state.camera_parameters.hflip,
    "Flip image horizontally");
  declareBoolParameter(
    "vflip", state.camera_parameters.vflip,
    "Flip image vertically");
}

rcl_interfaces::msg::SetParametersResult RaspicamPublisher::onSetParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  for (const auto & parameter : parameters) {
    RCLCPP_INFO_STREAM(get_logger(), "Set parameter: " << parameter.get_name());
    std::string name = parameter.get_name();
    auto roi = state.camera_parameters.roi;
    bool roi_changed = false;

    if (name == "sharpness") {
      if (raspicamcontrol_set_sharpness(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
      state.camera_parameters.sharpness = parameter.as_int();
    } else if (name == "contrast") {
      if (raspicamcontrol_set_contrast(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
      state.camera_parameters.contrast = parameter.as_int();
    } else if (name == "brightness") {
      if (raspicamcontrol_set_brightness(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
      state.camera_parameters.brightness = parameter.as_int();
    } else if (name == "saturation") {
      if (raspicamcontrol_set_saturation(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
      state.camera_parameters.saturation = parameter.as_int();
    } else if (name == "ISO") {
      if (raspicamcontrol_set_ISO(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
      state.camera_parameters.ISO = parameter.as_int();
    } else if (name == "exposure_compensation") {
      if (raspicamcontrol_set_exposure_compensation(
          state.camera_component.get(), parameter.as_int()) != 0)
      {
        return result;
      }
      state.camera_parameters.exposureCompensation = parameter.as_int();
    } else if (name == "rotation") {
      if (raspicamcontrol_set_rotation(state.camera_component.get(), parameter.as_int()) != 0) {
        return result;
      }
      state.camera_parameters.rotation = -parameter.as_int();
    } else if (name == "roi.x") {
      roi.x = parameter.as_double();
      roi_changed = true;
    } else if (name == "roi.y") {
      roi.y = parameter.as_double();
      roi_changed = true;
    } else if (name == "roi.w") {
      roi.w = parameter.as_double();
      roi_changed = true;
    } else if (name == "roi.h") {
      roi.h = parameter.as_double();
      roi_changed = true;
    } else if (name == "analog_gain") {
      if (raspicamcontrol_set_gains(
          state.camera_component.get(), parameter.as_double(),
          state.camera_parameters.digital_gain) != 0)
      {
        return result;
      }
      state.camera_parameters.analog_gain = parameter.as_double();
    } else if (name == "digital_gain") {
      if (raspicamcontrol_set_gains(
          state.camera_component.get(), state.camera_parameters.analog_gain,
          parameter.as_double()) != 0)
      {
        return result;
      }
      state.camera_parameters.digital_gain = parameter.as_double();
    } else if (name == "video_stabilization") {
      if (raspicamcontrol_set_video_stabilisation(
          state.camera_component.get(),
          parameter.as_bool() ? 1 : 0) != 0)
      {
        return result;
      }
      state.camera_parameters.videoStabilisation = parameter.as_bool() ? 1 : 0;
    } else if (name == "hflip") {
      if (raspicamcontrol_set_flips(
          state.camera_component.get(), parameter.as_bool() ? 1 : 0,
          state.camera_parameters.vflip) != 0)
      {
        return result;
      }
      state.camera_parameters.hflip = parameter.as_bool() ? 1 : 0;
    } else if (name == "vflip") {
      if (raspicamcontrol_set_flips(
          state.camera_component.get(), state.camera_parameters.hflip,
          parameter.as_bool() ? 1 : 0) != 0)
      {
        return result;
      }
      state.camera_parameters.vflip = parameter.as_bool() ? 1 : 0;
    }

    if (roi_changed) {
      if (raspicamcontrol_set_ROI(state.camera_component.get(), roi) != 0) {
        return result;
      }
      state.camera_parameters.roi = roi;
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

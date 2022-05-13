#include <raspicam_publisher.hpp>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

RaspicamPublisher::RaspicamPublisher(rclcpp::NodeOptions options)
: Node("raspicam_node", "", options.use_intra_process_comms(true))
{
  callback_handle = add_on_set_parameters_callback(
    std::bind(
      &RaspicamPublisher::onSetParameter, this,
      std::placeholders::_1));

  declareParameters();
  updateParameters();

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


void RaspicamPublisher::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  descriptor.description = "Width of the image";
  descriptor.read_only = true;


  declare_parameter("width", 320, descriptor);
  declare_parameter("height", 240);
  declare_parameter("quality", 80);
  declare_parameter("framerate", 90);
  declare_parameter("publish_raw", true);

}


void RaspicamPublisher::updateParameters()
{
  get_parameter("width", state.width);
  get_parameter("height", state.height);
  get_parameter("quality", state.quality);
  get_parameter("framerate", state.framerate);
  get_parameter("publish_raw", state.enable_raw_pub);
}

rcl_interfaces::msg::SetParametersResult RaspicamPublisher::onSetParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & parameter : parameters) {
    RCLCPP_INFO_STREAM(get_logger(), "Set parameter: " << parameter.get_name());
  }

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

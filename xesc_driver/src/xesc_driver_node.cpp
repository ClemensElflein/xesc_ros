#include "xesc_driver/xesc_driver_node.h"
#include <rmw/qos_profiles.h>

using namespace xesc_driver;
using std::placeholders::_1;

XescDriverNode::XescDriverNode() : rclcpp::Node("XescDriver")
{
    declare_parameter("xesc_type", rclcpp::PARAMETER_STRING);

    xesc_driver_ptr = new xesc_driver::XescDriver(*this);

    state_publisher_ = create_publisher<xesc_msgs::msg::XescStateStamped>(
        "~/sensors/core", rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data));
    vel_sub_ = create_subscription<std_msgs::msg::Float32>("~/duty_cycle", 10, std::bind(&XescDriverNode::velReceived, this, _1));
}

void XescDriverNode::velReceived(const std_msgs::msg::Float32& msg)
{
    if(!xesc_driver_ptr)
        return;

    xesc_driver_ptr->setDutyCycle(msg.data);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XescDriverNode>());
    rclcpp::shutdown();
    return 0;
}

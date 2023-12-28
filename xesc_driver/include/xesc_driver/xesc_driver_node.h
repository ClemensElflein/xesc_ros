//
// Created by clemens on 03.07.22.
//

#ifndef SRC_XESC_DRIVER_NODE_H
#define SRC_XESC_DRIVER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <xesc_msgs/msg/xesc_state_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include "xesc_driver.h"

namespace xesc_driver  {
    class XescDriverNode: public rclcpp::Node {
    public:
        XescDriverNode();
    private:
        void velReceived(const std_msgs::msg::Float32 &msg);
        xesc_driver::XescDriver* xesc_driver_ptr = nullptr;
        rclcpp::Publisher<xesc_msgs::msg::XescStateStamped>::SharedPtr state_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_;
    };
}

#endif //SRC_XESC_DRIVER_NODE_H

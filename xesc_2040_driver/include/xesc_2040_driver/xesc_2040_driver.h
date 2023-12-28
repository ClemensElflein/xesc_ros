//
// Created by clemens on 03.07.22.
//

#ifndef SRC_XESC_2040_DRIVER_H
#define SRC_XESC_2040_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <xesc_interface/xesc_interface.h>
#include "xesc_2040_interface.h"


namespace xesc_2040_driver  {
    class Xesc2040Driver: public xesc_interface::XescInterface {
    public:
        Xesc2040Driver(rclcpp::Node &nh);
        void getStatus(xesc_msgs::msg::XescStateStamped &state) override;

        void getStatusBlocking(xesc_msgs::msg::XescStateStamped &state) override;

        void setDutyCycle(float duty_cycle) override;

        void stop() override;

    private:
        const rclcpp::Node *node_;
        void error_func(const std::string &s);
        xesc_2040_driver::Xesc2040StatusStruct status{};
        xesc_2040_driver::Xesc2040Interface* xesc_interface = nullptr;
    };
}

#endif //SRC_XESC_2040_DRIVER_H

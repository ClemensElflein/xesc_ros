#include "xesc_driver/xesc_driver.h"

xesc_driver::XescDriver::XescDriver(rclcpp::Node &nh) {
    RCLCPP_INFO_STREAM(nh.get_logger(),"Starting xesc driver");

    // Check the xesc_type field and create the appropriate driver for it
    std::string xesc_type;
    if(!nh.get_parameter("xesc_type", xesc_type)) {
        RCLCPP_ERROR_STREAM(nh.get_logger(),"Error: xesc_type not set.");
        rclcpp::shutdown();
        xesc_driver = nullptr;
        return;
    }

    if(xesc_type == "xesc_2040") {
        xesc_driver = new xesc_2040_driver::Xesc2040Driver(nh);
    } else if(xesc_type == "xesc_mini") {
        xesc_driver = new vesc_driver::VescDriver(nh);
    } else {
        RCLCPP_ERROR_STREAM(nh.get_logger(),"Error: xesc_type invalid. Type was: " << xesc_type);
        rclcpp::shutdown();
        xesc_driver = nullptr;
        return;
    }
}

void xesc_driver::XescDriver::getStatus(xesc_msgs::msg::XescStateStamped &state_msg) {
    if (!xesc_driver)
        return;
    xesc_driver->getStatus(state_msg);
}

void xesc_driver::XescDriver::getStatusBlocking(xesc_msgs::msg::XescStateStamped &state_msg) {
    if (!xesc_driver)
        return;
    xesc_driver->getStatusBlocking(state_msg);
}

void xesc_driver::XescDriver::stop() {
    if (!xesc_driver)
        return;
    xesc_driver->stop();
}

void xesc_driver::XescDriver::setDutyCycle(float duty_cycle) {
    if (!xesc_driver)
        return;
    xesc_driver->setDutyCycle(duty_cycle);
}

xesc_driver::XescDriver::~XescDriver() {
    if (xesc_driver) {
        delete xesc_driver;
        xesc_driver = nullptr;
    }
}

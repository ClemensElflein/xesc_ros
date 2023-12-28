#include "xesc_2040_driver/xesc_2040_driver.h"


void xesc_2040_driver::Xesc2040Driver::error_func(const std::string &s) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),s);
}


xesc_2040_driver::Xesc2040Driver::Xesc2040Driver(rclcpp::Node &nh) : node_(&nh) {
    RCLCPP_INFO_STREAM(node_->get_logger(),"Starting xesc 2040 driver");

    nh.declare_parameter("serial_port", rclcpp::PARAMETER_STRING);
    for(int i = 0; i < 8; i++)
    {
        nh.declare_parameter(std::string("hall_table_")+std::to_string(i), rclcpp::PARAMETER_INTEGER);
    }
    nh.declare_parameter("motor_current_limit", rclcpp::PARAMETER_DOUBLE);
    nh.declare_parameter("acceleration", rclcpp::PARAMETER_DOUBLE);
    nh.declare_parameter("has_motor_temp", rclcpp::PARAMETER_BOOL);
    nh.declare_parameter("min_motor_temp", rclcpp::PARAMETER_DOUBLE);
    nh.declare_parameter("max_motor_temp", rclcpp::PARAMETER_DOUBLE);
    nh.declare_parameter("min_pcb_temp", rclcpp::PARAMETER_DOUBLE);
    nh.declare_parameter("max_pcb_temp", rclcpp::PARAMETER_DOUBLE);


    xesc_interface = new xesc_2040_driver::Xesc2040Interface([this](auto && PH1) { error_func(std::forward<decltype(PH1)>(PH1)); });

    uint8_t hall_table[8];
    float motor_current_limit;
    float acceleration;
    bool has_motor_temp;
    float min_motor_temp;
    float max_motor_temp;
    float min_pcb_temp;
    float max_pcb_temp;
    std::string serial_port;

    if(!node_->get_parameter("serial_port", serial_port)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter serial_port.");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter serial_port.");
    }

    for(int i = 0; i < 8; i++) {
        int tmp;
        if (!node_->get_parameter(std::string("hall_table_")+std::to_string(i), tmp)) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter "<< std::string("hall_table_")+std::to_string(i));
            throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter "+ std::string("hall_table_")+std::to_string(i));
        }
        hall_table[i] = tmp;
    }

    if(!node_->get_parameter("motor_current_limit",motor_current_limit)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter motor_current_limit");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter motor_current_limit");
    }
    if(!node_->get_parameter("acceleration",acceleration)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter acceleration");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter acceleration");
    }
    if(!node_->get_parameter("has_motor_temp",has_motor_temp)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter has_motor_temp");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter has_motor_temp");
    }
    if(!node_->get_parameter("min_motor_temp",min_motor_temp)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter min_motor_temp");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter min_motor_temp");
    }
    if(!node_->get_parameter("max_motor_temp",max_motor_temp)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter max_motor_temp");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter max_motor_temp");
    }
    if(!node_->get_parameter("min_pcb_temp",min_pcb_temp)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter min_pcb_temp");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter min_pcb_temp");
    }
    if(!node_->get_parameter("max_pcb_temp",max_pcb_temp)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"You need to provide parameter max_pcb_temp");
        throw rclcpp::exceptions::InvalidParametersException("You need to provide parameter max_pcb_temp");
    }

    xesc_interface->update_settings(hall_table,
                                    motor_current_limit,
                                    acceleration,
                                    has_motor_temp,
                                    min_motor_temp,
                                    max_motor_temp,
                                    min_pcb_temp,
                                    max_pcb_temp);

    xesc_interface->start(serial_port);
}

void xesc_2040_driver::Xesc2040Driver::getStatus(xesc_msgs::msg::XescStateStamped &state_msg) {
    if(!xesc_interface)
        return;
    xesc_interface->get_status(&status);

    state_msg.header.stamp = node_->now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.voltage_input = status.voltage_input;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.temperature_motor = status.temperature_motor;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.direction = status.direction;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_2040_driver::Xesc2040Driver::getStatusBlocking(xesc_msgs::msg::XescStateStamped &state_msg) {
    if(!xesc_interface)
        return;
    xesc_interface->wait_for_status(&status);

    state_msg.header.stamp = node_->now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.voltage_input = status.voltage_input;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.temperature_motor = status.temperature_motor;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.direction = status.direction;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_2040_driver::Xesc2040Driver::stop() {
    RCLCPP_INFO_STREAM(node_->get_logger(),"stopping XESC2040 driver");
    xesc_interface->stop();

    delete xesc_interface;
}

void xesc_2040_driver::Xesc2040Driver::setDutyCycle(float duty_cycle) {
    if(xesc_interface) {
        xesc_interface->setDutyCycle(duty_cycle);
    }
}

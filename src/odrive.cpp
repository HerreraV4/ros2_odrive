#include "rclcpp/rclcpp.hpp"
#include "ros_odrive_msgs/msg/odrvctrl.hpp"
#include "ros_odrive_msgs/msg/odrvmsg.hpp"
#include "ros_odrive/odrive.hpp"
#include <jsoncpp/json/json.h>


using namespace std;

odrive *od;

void msgCallback(const ros_odrive_msgs::msg::Odrvctrl::ConstSharedPtr &msg)
{
    std::string cmd;
    uint8_t u8val;
    uint32_t u32val;
    float fval;
    odrive_endpoint *endpoint = NULL;
    Json::Value odrive_json;

    if (msg->axis == 0) {
        cmd = "axis0";
    }
    else if (msg->axis == 1){
        cmd = "axis1";
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("odrive"),"* Invalid axis value in message!");
    return;
    }

    if ((msg->target < 0) || (msg->target >= MAX_NR_OF_TARGETS)) {
        RCLCPP_ERROR(rclcpp::get_logger("odrive"),"* Invalid target value in message!");
        return;
    }

    endpoint = od->endpoint.at(msg->target);
    odrive_json = od->json.at(msg->target);

    switch (msg->command) {
        case (CMD_AXIS_RESET):
            // Reset errors
            u32val = 0;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".motor.error"), u32val);
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".encoder.error"), u32val);
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".controller.error"), u32val);
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".error"), u32val);
            RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 0. RESET");
            break;

	case (CMD_AXIS_IDLE):
            // Set channel to Idle
            u32val = AXIS_STATE_IDLE;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".requested_state"), u32val);
            RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 1. IDLE");
             break;

	case (CMD_AXIS_CLOSED_LOOP):
            // Enable Closed Loop Control
            u32val = AXIS_STATE_CLOSED_LOOP_CONTROL;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".requested_state"), u32val);
        RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 2. CLOSED LOOP");
        break;

        case (CMD_AXIS_SET_VELOCITY):
            // Set velocity
            fval = msg->fval;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".controller.input_vel"), fval);
            RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 3. SET VELOCITY");
            break;


	case (CMD_AXIS_SET_VELOCITY_DUAL):
            // Set velocity on both axis
            fval = msg->fval;
            writeOdriveData(endpoint, odrive_json,
                    "axis0.controller.input_vel", fval);
            fval = msg->fval2;
            writeOdriveData(endpoint, odrive_json,
                    "axis1.controller.input_vel", fval);
            RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 4. SET VELOCITY DUAL");
            break;

        case (CMD_REBOOT):
            execOdriveFunc(endpoint, odrive_json, string("reboot"));
            RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 5. REBOOT");
            break;

    // Set torque
        case (CMD_AXIS_SET_TORQUE):
            fval = msg->fval;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".controller.input_torque"), fval);
            RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 6. TORQUE");
            break;

        default:
            RCLCPP_ERROR(rclcpp::get_logger("odrive"),"* Invalid command type in message!");
            RCLCPP_INFO(rclcpp::get_logger("odrive"),"CASO 7---");
            return;
    }
}

/**
 *
 * Publise odrive message to ROS
 * @param endpoint odrive enumarated endpoint
 * @param odrive_json target json
 * @param odrive_pub ROS publisher
 * return ODRIVE_OK in success
 *
 */
int publishMessage(odrive_endpoint *endpoint, Json::Value odrive_json, rclcpp::Publisher<ros_odrive_msgs::msg::Odrvmsg>::SharedPtr &odrive_pub)
{
    uint16_t u16val;
    uint32_t u32val;
    float fval;
    ros_odrive_msgs::msg::Odrvmsg msg;

    // Collect data
    readOdriveData(endpoint, odrive_json, string("vbus_voltage"), fval);
    msg.vbus = fval;
    readOdriveData(endpoint, odrive_json, string("axis0.error"), u32val);
    msg.error0 = u32val;
    readOdriveData(endpoint, odrive_json, string("axis1.error"), u32val);
    msg.error1 = u32val;
    readOdriveData(endpoint, odrive_json, string("axis0.current_state"), u32val);
    msg.state0 = u32val;
    readOdriveData(endpoint, odrive_json, string("axis1.current_state"), u32val);
    msg.state1 = u32val;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.encoder.vel_estimate"), fval);
    msg.vel0 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.encoder.vel_estimate"), fval);
    msg.vel1 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.encoder.pos_estimate"), fval);
    msg.pos0 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.encoder.pos_estimate"), fval);
    msg.pos1 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.motor.current_meas_phB"), fval);
    msg.curr0b = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.motor.current_meas_phC"), fval);
    msg.curr0c = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.motor.current_meas_phB"), fval);
    msg.curr1b = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.motor.current_meas_phC"), fval);
    msg.curr1c = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.fet_thermistor.temperature"), fval);
    msg.temp0 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.fet_thermistor.temperature"), fval);
    msg.temp1 = fval;

    // Publish message
    odrive_pub->publish(msg);

    return ODRIVE_OK;
}


int main(int argc, char **argv) {
    std::string od_sn;
    std::string od_cfg;

    RCLCPP_INFO(rclcpp::get_logger("odrive"),"Starting ODrive...");

    od = new odrive();

    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("odrive");
    rclcpp::Rate r(1);

    nh->declare_parameter<std::string>("od_sn", "0x00000000");
    nh->get_parameter("od_sn", od_sn);

    nh->declare_parameter<std::string>("od_cfg", "");
    nh->get_parameter("od_cfg", od_cfg);

    if (!(nh->get_parameter("od_sn", od_sn))) {
        RCLCPP_ERROR(rclcpp::get_logger("odrive"),"Failed to get sn parameter %s!", od_sn.c_str());
        return 1;
    }

    std::stringstream ssn(od_sn);
    while(ssn.good()) {
        string substr;
        getline(ssn, substr, ',');
        od->target_sn.push_back(substr);
    }
    
    std::stringstream scfg(od_cfg);
    while(scfg.good()) {
        string substr;
        getline(scfg, substr, ',');
        od->target_cfg.push_back(substr);
    }

    if (od->target_sn.size() != od->target_cfg.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("odrive"),"* Configuration parameters do not match Serial Numbers list!");
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("odrive"),"%d odrive instances:", (int)od->target_sn.size());
    for(int i = 0; i < od->target_sn.size() ; i++) {
        RCLCPP_INFO(rclcpp::get_logger("odrive"),"- Instance %d: SN %s - cfg %s",
            i, od->target_sn.at(i).c_str(), od->target_cfg.at(i).c_str());
        
        
        od->odrive_pub.push_back(
    	nh->create_publisher<ros_odrive_msgs::msg::Odrvmsg>("odrive_msg_" + od->target_sn.at(i), 100));
        
        od->odrive_sub.push_back(
    	nh->create_subscription<ros_odrive_msgs::msg::Odrvctrl>("odrive_ctrl_" + od->target_sn.at(i), rclcpp::SystemDefaultsQoS(), msgCallback));
        
        // Get odrive endpoint instance
        od->endpoint.push_back(new odrive_endpoint());

        // Enumarate Odrive target
        if (od->endpoint.at(i)->init(stoull(od->target_sn.at(i), 0, 16))) {
            RCLCPP_ERROR(rclcpp::get_logger("odrive"),"* Device not found!");
            return 1;
        }

        // Read JSON from target
        Json::Value odrive_json;
        if (getJson(od->endpoint.at(i), &odrive_json)) {
            return 1;
        }

        od->json.push_back(odrive_json);

        // Process configuration file
        updateTargetConfig(od->endpoint.at(i), od->json.at(i), od->target_cfg.at(i));
    }
    
    RCLCPP_INFO(rclcpp::get_logger("odrive"),"Starting idle loop");
    
    auto timer_callback = [nh]()
    {
        for (int i = 0; i < od->target_sn.size(); i++) {
            publishMessage(od->endpoint.at(i), od->json.at(i), od->odrive_pub.at(i));
            execOdriveFunc(od->endpoint.at(i), od->json.at(i), "axis0.watchdog_feed");
            execOdriveFunc(od->endpoint.at(i), od->json.at(i), "axis1.watchdog_feed");
        }
    };

    auto timer = nh->create_wall_timer(1s, timer_callback);

    rclcpp::spin(nh);
     
    for(int i = 0; i < od->target_sn.size() ; i++) {
        od->endpoint.at(i)->remove();
        delete od->endpoint.at(i);
    }
    
    rclcpp::shutdown();
    return 0;
}



#ifndef ODRIVE_HPP_
#define ODRIVE_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include "ros_odrive_msgs/msg/odrvmsg.hpp"
#include "ros_odrive_msgs/msg/odrvctrl.hpp"
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "ros_odrive/odrive_endpoint.hpp"
#include "ros_odrive/odrive_utils.hpp"
#include "ros_odrive/odrive_enums.hpp"
#include <jsoncpp/json/json.h>

#define ODRIVE_OK    0
#define ODRIVE_ERROR 1

#define MAX_NR_OF_TARGETS 16

using namespace std;

// Listener commands
enum commands {
    CMD_AXIS_RESET,
    CMD_AXIS_IDLE,
    CMD_AXIS_CLOSED_LOOP,
    CMD_AXIS_SET_VELOCITY,
    CMD_AXIS_SET_VELOCITY_DUAL,
    CMD_REBOOT,
    CMD_AXIS_SET_TORQUE
};


class odrive{
    private:
        void msgCallback(const ros_odrive_msgs::msg::Odrvctrl::ConstSharedPtr &msg);
    public:
        vector<string> target_sn;
        vector<string> target_cfg;
        std::vector<std::shared_ptr<rclcpp::Publisher<ros_odrive_msgs::msg::Odrvmsg>>> odrive_pub;
        std::vector<std::shared_ptr<rclcpp::Subscription<ros_odrive_msgs::msg::Odrvctrl>>> odrive_sub;

        vector<odrive_endpoint *> endpoint;
        vector<Json::Value> json;
};


/*
class odrive {
public:
    std::vector<std::string> target_sn;
    std::vector<std::string> target_cfg;
    std::vector<std::shared_ptr<rclcpp::Publisher<ros_odrive_msgs::msg::Odrvmsg>>> odrive_pub;
    std::vector<std::shared_ptr<rclcpp::Subscription<ros_odrive_msgs::msg::Odrvctrl>>> odrive_sub;
    std::vector<std::unique_ptr<odrive_endpoint>> endpoint;
    std::vector<Json::Value> json;

    void msgCallback(const ros_odrive_msgs::msg::Odrvctrl::SharedPtr &msg);

    odrive() = default;

    ~odrive() {
        for (auto &ep : endpoint) {
            if (ep) {
                ep->remove();
            }
        }
    }
};
*/
#endif

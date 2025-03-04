#include "khi_ux150_hardware/khi_ux150_hardware.hpp"
#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace khi_ux150_hardware
{

    KHI_UX150_HARDWARE_ hardware_interface::CallbackReturn khi_ux150_hardware::KhiUX150Hardware::on_init(const hardware_interface::HardwareInfo &info)
    {
    info_ = info;
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    hw_command_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_state_position_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_state_velocity_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("HardwareInterface"), "Joint'%s' has %zu command interfaces found. 1 expected",
                         joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("UnrealInterface"), "Joint'%s' has %s command interface. %s expected",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("HardwareInterface"), "Joint'%s' has %zu state interfaces found. 2 expected",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("UnrealInterface"), "Joint'%s' has %s state interface. %s expected",
                         joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("UnrealInterface"), "Joint'%s' has %s command interface. %s expected",
                         joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
    }

    KHI_UX150_HARDWARE_ hardware_interface::CallbackReturn KhiUX150Hardware::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        for (uint i = 0; i < hw_state_position_.size(); i++)
        {
            hw_state_position_[i] = 0;
            hw_state_velocity_[i] = 0;
            hw_command_[i] = 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Successfully configured");

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    KHI_UX150_HARDWARE_ hardware_interface::CallbackReturn KhiUX150Hardware::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        if(telnetConnect("192.168.0.2",23) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Failed to connect to the controller");
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }    
    
    KHI_UX150_HARDWARE_ hardware_interface::CallbackReturn KhiUX150Hardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        telnetDisconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    KHI_UX150_HARDWARE_ std::vector<hardware_interface::StateInterface> KhiUX150Hardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,hardware_interface::HW_IF_POSITION, &hw_state_position_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]));
        }
        return state_interfaces;
    }

    KHI_UX150_HARDWARE_ std::vector<hardware_interface::CommandInterface> KhiUX150Hardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interface;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interface.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,hardware_interface::HW_IF_POSITION, &hw_command_[i]));
        }
        return command_interface;
    }

    KHI_UX150_HARDWARE_ hardware_interface::return_type KhiUX150Hardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::string tmp_msg;
        std::vector<std::string> msg;
        telnetWrite("here");
        tmp_msg = telnetRead();
        boost::split(msg,tmp_msg, boost::is_any_of(" "));
        for(int i = 0; i < msg.size(); i++){
            hw_state_position_[i]= M_PI/180 * std::stod(msg[i]);
        }
        return hardware_interface::return_type::OK;
    }

    KHI_UX150_HARDWARE_ hardware_interface::return_type KhiUX150Hardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::vector<float> degree_hw_commands(sizeof(hw_command_));
        for (size_t i = 0; i < sizeof(hw_command_); i++)
        {
            degree_hw_commands[i] = 180.0/M_PI * hw_command_[i];
        }
        
        std::string command = "do jmove [" + std::to_string(degree_hw_commands[0]) + ", " +
                                            std::to_string(degree_hw_commands[1]) + ", " +
                                            std::to_string(degree_hw_commands[2]) + ", " +
                                            std::to_string(degree_hw_commands[3]) + ", " +
                                            std::to_string(degree_hw_commands[4]) + ", " +
                                            std::to_string(degree_hw_commands[5]) + "]";
        telnetWrite(command);
        return hardware_interface::return_type::OK;
    }

    int KhiUX150Hardware::telnetConnect(const std::string &ip, int port)
    {
        server_ip_ = ip.c_str();
        server_port_ = port;
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) {
            std::cerr << "Could not create socket: " << strerror(errno) << std::endl;
            return -1;
        }

        struct sockaddr_in server_address;
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(server_port_);

        // Convert IP address from text to binary form
        if (inet_pton(AF_INET, server_ip_, &server_address.sin_addr) <= 0) {
            std::cerr << "Invalid address or address not supported: " << server_ip_ << std::endl;
            return -1;
        }

        // Connect to the server
        if (connect(sock_, (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
            std::cerr << "Connection failed: " << strerror(errno) << std::endl;
            close(sock_);
            return -1;
        }

        is_connected_ = true;
        std::cout << "Connected to Telnet server at " << server_ip_ << ":" << server_port_ << std::endl;
        return 0;
    }

    void KhiUX150Hardware::telnetDisconnect()
    {
        if (is_connected_) {
            close(sock_);
            is_connected_ = false;
            std::cout << "Disconnected from server." << std::endl;
        }
    }

    std::string KhiUX150Hardware::telnetRead()
    {
        if (!is_connected_) {
            std::cerr << "Not connected to a server." << std::endl;
            return "";
        }

        char buffer[512];
        std::string response;
        int bytes_received;
        while ((bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0)) > 0) {
            buffer[bytes_received] = '\0'; // Null-terminate the buffer
            response += buffer;
        }

        if (bytes_received < 0) {
            std::cerr << "Read error: " << strerror(errno) << std::endl;
        }

        return response;
    }
    int KhiUX150Hardware::telnetWrite(const std::string &telnet_command)
    {
        if (!is_connected_) {
            std::cerr << "Not connected to a server." << std::endl;
            return -1;
        }
        std::string cmd = telnet_command + "\r\n"; // Append carriage return and newline
        if (send(sock_, cmd.c_str(), cmd.length(), 0) < 0) {
            std::cerr << "Failed to send command: " << strerror(errno) << std::endl;
            return -1;
        }
        return 0;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(khi_ux150_hardware::KhiUX150Hardware, hardware_interface::SystemInterface)
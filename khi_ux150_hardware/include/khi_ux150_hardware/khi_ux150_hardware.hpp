#ifndef KHI_UX150_HARDWARE_
#define KHI_UX150_HARDWARE_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace khi_ux150_hardware
{
class KhiUX150Hardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(KhiUX150Hardware)

    KHI_UX150_HARDWARE_
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    
    KHI_UX150_HARDWARE_
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    
    KHI_UX150_HARDWARE_
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    
    KHI_UX150_HARDWARE_
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    KHI_UX150_HARDWARE_
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    
    KHI_UX150_HARDWARE_
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    KHI_UX150_HARDWARE_
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    KHI_UX150_HARDWARE_
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    hardware_interface::HardwareInfo info_;
    std::vector<double> hw_command_;
    std::vector<double> hw_state_position_;
    std::vector<double> hw_state_velocity_;
    const char* server_ip_;
    int server_port_;
    int sock_;
    bool is_connected_;
    int telnetConnect(const std::string &ip, int port);
    void telnetDisconnect();
    std::string telnetRead();
    int telnetWrite(const std::string &telnet_command);
};
}// namespace khi_ux150_hardware
#endif // KHI_UX150_HARDWARE
#include "wheebot_firmware/wheebot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace wheebot_firmware
{

WheebotInterface::WheebotInterface()
{
}

WheebotInterface::~WheebotInterface()
{
    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("WheebotInterface"), "Something went wrong while closing connection with port " << port_);
        }
    }
}

CallbackReturn WheebotInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch (const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("WheebotInterface"), "No Serial Port provided! Aborting");
        return CallbackReturn::FAILURE;
    }

    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    last_run_ = rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> WheebotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> WheebotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }

    return command_interfaces;
}


CallbackReturn WheebotInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("WheebotInterface"), "Starting robot hardware ...");

    // Reset commands and states
    velocity_commands_ = { 0.0, 0.0 };
    position_states_ = { 0.0, 0.0 };
    velocity_states_ = { 0.0, 0.0 };

    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("WheebotInterface"), "Something went wrong while interacting with port " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("WheebotInterface"), "Hardware started, ready to take commands");
    return CallbackReturn::SUCCESS;
}


CallbackReturn WheebotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("WheebotInterface"), "Stopping robot hardware ...");

    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("WheebotInterface"), "Something went wrong while closing connection with port " << port_);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("WheebotInterface"), "Hardware stopped");
    return CallbackReturn::SUCCESS;
}


hardware_interface::return_type WheebotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Interpret the string
  if(arduino_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;
    int multiplier = 1;
    while(std::getline(ss, res, ','))
    {
      multiplier = res.at(1) == 'p' ? 1 : -1;

      if(res.at(0) == 'r')
      {
        velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(0) += velocity_states_.at(0) * dt;
      }
      else if(res.at(0) == 'l')
      {
        velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      }
    }
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type WheebotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::stringstream message_stream;

    int right_vel = velocity_commands_.at(0) * 255;
    int left_vel = velocity_commands_.at(1) * 255;

    int inAxisXVal = (right_vel + left_vel)/2;
    int inAxisYVal = (left_vel - right_vel)/2;

    // safety
    if(inAxisXVal > 255) inAxisXVal = 255;
    if(inAxisXVal < -255) inAxisXVal = -255;
    if(inAxisYVal > 255) inAxisYVal = 255;
    if(inAxisYVal < -255) inAxisYVal = -255;

    message_stream << inAxisXVal << "," << inAxisYVal << ",\n";

    try
    {
        arduino_.Write(message_stream.str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("WheebotInterface"), "Sent message " << message_stream.str() << " to the port " << port_);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("WheebotInterface"), "Sent message " << velocity_commands_.at(0) << " | " << velocity_commands_.at(1) << " to the port " << port_);
    }
    catch (...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("WheebotInterface"), "Something went wrong while sending the message " << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}
}  // namespace wheebot_firmware

PLUGINLIB_EXPORT_CLASS(wheebot_firmware::WheebotInterface, hardware_interface::SystemInterface)
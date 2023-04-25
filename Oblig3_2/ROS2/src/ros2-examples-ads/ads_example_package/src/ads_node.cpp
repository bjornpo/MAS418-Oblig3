#include <iostream>
#include <string>
#include "AdsLib.h"
#include "AdsVariable.h"

#include <thread>
#include <chrono>

#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crane_interfaces/msg/crane_reference.hpp" //CHANGE
#include "sensor_msgs/msg/joint_state.hpp"

namespace craneads {

    struct AdsVariables
    {
        AdsVariables() = delete;

        explicit AdsVariables(AdsDevice& route)
            : activateMotion{route, "MAIN.bActivateMotion"}
            , velocityReference{route, "MAIN.fVelRef"}
            , positionReference{route, "MAIN.fPosRef"}
            , positionMeasurement{route, "MAIN.fPosMeas"}
            , pressureMeasurement{route, "MAIN.fPres1Meas"}
            , pressureMeasurement2{route, "MAIN.fPres2Meas"}
        {
            // Do nothing.
        }

        AdsVariable<bool> activateMotion;
        AdsVariable<double> velocityReference;
        AdsVariable<double> positionReference;
        AdsVariable<double> positionMeasurement;
        AdsVariable<double> pressureMeasurement;
        AdsVariable<double> pressureMeasurement2;
    };

    class AdsHandler
    {
    public:
        explicit AdsHandler(const AmsNetId remoteNetId, const std::string remoteIpV4)
            : remoteNetId_(remoteNetId)
            , remoteIpV4_(remoteIpV4)
            , route_{remoteIpV4_, remoteNetId_, AMSPORT_R0_PLC_TC3}
            , ads_(route_) { }

        AdsHandler() : AdsHandler({127, 0, 0, 1,  1, 1}, "127.0.0.1") { }


        void activateMotion()
        {
            ads_.activateMotion = true;
        }

        void deactivateMotion()
        {
            ads_.activateMotion = false;
        }

        void setVelocityReference(double value)
        {
            ads_.velocityReference = value;
        }

        void setPositionReference(double value)
        {
            ads_.velocityReference = value;
        }

        double getPositionMeasurement()
        {
            return ads_.positionMeasurement;
        }
	
	double getPressureMeasurement1()
        {
            return ads_.pressureMeasurement;
        }
	
	double getPressureMeasurement2()
        {
            return ads_.pressureMeasurement2;
        }
        
        void printState()
        {
            const auto state = route_.GetState();
            std::cout << "ADS state: "
                      << std::dec << static_cast<uint16_t>(state.ads)
                      << " devState: "
                      << std::dec << static_cast<uint16_t>(state.device);
        }

        ~AdsHandler() { }

    private:
        const AmsNetId remoteNetId_;
        const std::string remoteIpV4_;
        AdsDevice route_;
        AdsVariables ads_;
    };

}


class sinCurve
{
  public:
    float sinOut()
    {
      time = time + 0.001;
      angle = hight*sin(time)+offset;
      
      return angle;
    };
  
  private:
    float time = 0.0;
    float angle = 0.0;
    float hight = 0.2;
    float offset = 0.2;
};

sinCurve sinGenerator;

void handler(float * paraList)
{
  auto angleRef = 0.3;//sinGenerator.sinOut();

  std::cout << "Example ROS2 ADS node starting up.." << std::endl;

  // Real lab PLC IP.
  const AmsNetId remoteNetId {192,168, 0, 12, 1, 1 };
  const std::string remoteIpV4 = "192.168.0.12";

  // Connecting to testbed computer.
  //const AmsNetId remoteNetId { 192, 168, 56, 1, 1, 1 };
  //const std::string remoteIpV4 = "192.168.56.1";

  std::cout << "  Create AdsHandler.. ";
  craneads::AdsHandler adsHandler(remoteNetId, remoteIpV4);
  std::cout << "  OK" << std::endl;

  //adsHandler.deactivateMotion();

  //adsHandler.printState();

  //adsHandler.setVelocityReference(3.2);
  //std::this_thread::sleep_for (std::chrono::seconds(5));

  adsHandler.setPositionReference(angleRef);

  adsHandler.activateMotion();
  //for(uint8_t n = 0; ; ++n)
  //{
    //adsHandler.setPositionReference(static_cast<double>(n) / 255.0);
    paraList[0] = -1.0 * adsHandler.getPositionMeasurement();
    paraList[1] = adsHandler.getPressureMeasurement1();
    paraList[2] = adsHandler.getPressureMeasurement2();
    std::cout << "Position measurement from ADS: " << adsHandler.getPositionMeasurement() << std::endl;
    //std::this_thread::sleep_for (std::chrono::seconds(0.1));
  //}

  return;
}

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("crane_controll"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10); // CHANGE
    timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::timer_callback, this));
    
     // Forsøk
    publisher2_ = this->create_publisher<crane_interfaces::msg::CraneReference>("pressure_states", 11); // CHANGE
    timer2_ = this->create_wall_timer(5ms, std::bind(&MinimalPublisher::timer_callback, this));
    
  }

private:
  void timer_callback()
  {
    float feedback[3] = {0.0, 0.0, 0.0};
    handler(feedback);
    auto message = sensor_msgs::msg::JointState();		// CHANGE
    message.name = {"base_to_crane_boom"};					// CHANGE
    message.position = {feedback[0]};
    //message.velocity = {0.01}; 
    message.header.stamp = this->get_clock()->now();
    //message.crane_cylinder_velocity_reference = std::to_float(count_++); // CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.position[0]);	// CHANGE'
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.velocity[0]);	// CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.name[0].c_str());	// CHANGE
    publisher_->publish(message);
    
    //time = time + 0.01;
    //angle = hight*sin(time);
    
     //Forsøk
    auto message2 = crane_interfaces::msg::CraneReference();
    message2.name = {"Pressure Messuremnts"};
    message2.crane_pressure_messurement_1 = {feedback[1]};
    message2.crane_pressure_messurement_2 = {feedback[2]};
    
    publisher2_->publish(message2);
    
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;	// CHANGe
  rclcpp::Publisher<crane_interfaces::msg::CraneReference>::SharedPtr publisher2_;	// Forsøk
  size_t count_;
  float time = 0.0;
  float angle = 0.0;
  float hight = 0.4;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

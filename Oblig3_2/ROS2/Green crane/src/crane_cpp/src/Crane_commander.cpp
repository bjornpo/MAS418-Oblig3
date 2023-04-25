#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "crane_interfaces/msg/crane_reference.hpp"


namespace crane {
   namespace config {
      using namespace std::chrono_literals;
      const auto dt = 500ms;
   }
}


class CraneReferencePublisher : public rclcpp::Node
{
  public:
    CraneReferencePublisher()
    : Node("crane_commander")
    , step_(0)
    , craneCylinderVelocityReference_(0.0)
    {
      using namespace std::chrono_literals;
      

      publisher_ = this->create_publisher<crane_interfaces::msg::CraneReference>("craneCylinderVelocityReference", 5);   
      timer_ = this->create_wall_timer(crane::config::dt, std::bind(&CraneReferencePublisher::update_reference, this));
    }

  private:
    void update_reference()
    {
      auto message = crane_interfaces::msg::CraneReference();
      message.crane_cylinder_velocity_reference = step_++;
      RCLCPP_INFO(this->get_logger(), "Cylinder reference: '%f'", message.crane_cylinder_velocity_reference);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<crane_interfaces::msg::CraneReference>::SharedPtr publisher_;
    size_t step_;
    double craneCylinderVelocityReference_;
};


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CraneReferencePublisher>());
  rclcpp::shutdown();
  
  return 0;
}




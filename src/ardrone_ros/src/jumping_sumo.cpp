#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#include <stdlib.h>
#include <curses.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>

#define JS_IP_ADDRESS "192.168.2.1"
#define JS_DISCOVERY_PORT 44444

class MinimalPublisher : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  // Create attributes just to try library
  ARDISCOVERY_Device_t *device = NULL;
  ARCONTROLLER_Device_t *deviceController = NULL;
  eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
  eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_DEVICE_STATE_MAX;

public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

    device = ARDISCOVERY_Device_New(&errorDiscovery);

    if (errorDiscovery == ARDISCOVERY_OK)
    {
      // create a JumpingSumo discovery device (ARDISCOVERY_PRODUCT_JS)
      errorDiscovery = ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_JS, "JS", JS_IP_ADDRESS, JS_DISCOVERY_PORT);
    }
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
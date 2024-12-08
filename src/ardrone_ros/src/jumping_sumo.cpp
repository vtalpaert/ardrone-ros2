#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
}

#define JS_IP_ADDRESS "192.168.2.1"
#define JS_DISCOVERY_PORT 44444
#define TAG "JumpingSumo"

class JumpingSumoNode : public rclcpp::Node {
private:
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    
    // Drone SDK objects
    ARDISCOVERY_Device_t *device_;
    ARCONTROLLER_Device_t *deviceController_;
    eARCONTROLLER_ERROR error_;
    eARCONTROLLER_DEVICE_STATE deviceState_;
    ARSAL_Sem_t stateSem_;

    bool initDrone() {
        eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

        // Create discovery device
        device_ = ARDISCOVERY_Device_New(&errorDiscovery);
        if (errorDiscovery != ARDISCOVERY_OK) {
            RCLCPP_ERROR(get_logger(), "Failed to create discovery device");
            return false;
        }

        // Initialize WiFi connection
        errorDiscovery = ARDISCOVERY_Device_InitWifi(device_, 
            ARDISCOVERY_PRODUCT_JS, "JS", JS_IP_ADDRESS, JS_DISCOVERY_PORT);
        if (errorDiscovery != ARDISCOVERY_OK) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize WiFi connection");
            return false;
        }

        // Create device controller
        deviceController_ = ARCONTROLLER_Device_New(device_, &error_);
        if (error_ != ARCONTROLLER_OK) {
            RCLCPP_ERROR(get_logger(), "Failed to create device controller");
            return false;
        }

        // Initialize semaphore
        ARSAL_Sem_Init(&stateSem_, 0, 0);

        // Add callbacks
        error_ = ARCONTROLLER_Device_AddStateChangedCallback(
            deviceController_, stateChanged, this);
        if (error_ != ARCONTROLLER_OK) return false;

        error_ = ARCONTROLLER_Device_SetVideoStreamCallbacks(
            deviceController_, decoderConfigCallback, didReceiveFrameCallback, NULL, this);
        if (error_ != ARCONTROLLER_OK) return false;

        // Start the device controller
        error_ = ARCONTROLLER_Device_Start(deviceController_);
        if (error_ != ARCONTROLLER_OK) return false;

        // Wait for device
        ARSAL_Sem_Wait(&stateSem_);
        
        deviceState_ = ARCONTROLLER_Device_GetState(deviceController_, &error_);
        if ((error_ != ARCONTROLLER_OK) || (deviceState_ != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
            RCLCPP_ERROR(get_logger(), "Failed to start device controller");
            return false;
        }

        // Enable video stream
        error_ = deviceController_->jumpingSumo->sendMediaStreamingVideoEnable(
            deviceController_->jumpingSumo, 1);
        if (error_ != ARCONTROLLER_OK) return false;

        return true;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!deviceController_) return;

        // Convert ROS velocity commands to drone commands
        // Forward/backward speed
        float speed = msg->linear.x * 100.0f;  // Scale to percentage
        speed = std::min(std::max(speed, -100.0f), 100.0f);
        
        // Angular velocity for turning
        float turn = msg->angular.z * 100.0f;  // Scale to percentage
        turn = std::min(std::max(turn, -100.0f), 100.0f);

        // Send commands to drone
        error_ = deviceController_->jumpingSumo->setPilotingPCMDFlag(
            deviceController_->jumpingSumo, 1);
        error_ = deviceController_->jumpingSumo->setPilotingPCMDSpeed(
            deviceController_->jumpingSumo, static_cast<int8_t>(speed));
        error_ = deviceController_->jumpingSumo->setPilotingPCMDTurn(
            deviceController_->jumpingSumo, static_cast<int8_t>(turn));
    }

    // Callback when device state changes
    static void stateChanged(eARCONTROLLER_DEVICE_STATE newState, 
                           eARCONTROLLER_ERROR /*error*/, void* customData) {
        JumpingSumoNode* node = static_cast<JumpingSumoNode*>(customData);
        switch (newState) {
            case ARCONTROLLER_DEVICE_STATE_RUNNING:
                ARSAL_Sem_Post(&node->stateSem_);
                break;
            case ARCONTROLLER_DEVICE_STATE_STOPPED:
                ARSAL_Sem_Post(&node->stateSem_);
                break;
            default:
                break;
        }
    }

    // Callback for video stream configuration
    static eARCONTROLLER_ERROR decoderConfigCallback(
        ARCONTROLLER_Stream_Codec_t codec, void *customData) {
        JumpingSumoNode* node = static_cast<JumpingSumoNode*>(customData);
        
        RCLCPP_INFO(node->get_logger(), "Video codec configuration received:");
        RCLCPP_INFO(node->get_logger(), "  Type: %d", codec.type);
        RCLCPP_INFO(node->get_logger(), "  Width: %d", codec.width);
        RCLCPP_INFO(node->get_logger(), "  Height: %d", codec.height);
        RCLCPP_INFO(node->get_logger(), "  Frame rate: %d", codec.framerate);
        
        return ARCONTROLLER_OK;
    }

    // Callback when video frame is received
    static eARCONTROLLER_ERROR didReceiveFrameCallback(
        ARCONTROLLER_Frame_t *frame, void *customData) {
        JumpingSumoNode* node = static_cast<JumpingSumoNode*>(customData);
        
        if (frame && frame->data && frame->used) {
            static int frame_count = 0;
            if (frame_count++ % 30 == 0) { // Log every 30 frames
                RCLCPP_DEBUG(node->get_logger(), 
                    "Received frame: size=%d isIFrame=%d",
                    frame->used, frame->isIFrame);
            }

            // The frame data is in MJPEG format, we need to decode it
            auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
            img_msg->header.stamp = node->now();
            img_msg->header.frame_id = "camera_frame";
            
            // Use actual frame dimensions from codec config
            img_msg->height = 480;  // Should match codec.height
            img_msg->width = 640;   // Should match codec.width
            img_msg->encoding = "jpeg";  // Raw MJPEG data
            img_msg->step = frame->used;
            img_msg->data.resize(frame->used);
            
            // Copy the raw MJPEG data
            std::memcpy(img_msg->data.data(), frame->data, frame->used);
            
            node->image_pub_->publish(std::move(img_msg));
        }
        return ARCONTROLLER_OK;
    }

public:
    JumpingSumoNode() : Node("jumping_sumo") {
        // Initialize ROS interfaces
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "jumpingsumo/cmd_vel", 10,
            std::bind(&JumpingSumoNode::cmdVelCallback, this, std::placeholders::_1));
            
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "jumpingsumo/raw_image", 10);

        // Initialize drone connection
        if (!initDrone()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize drone connection");
            throw std::runtime_error("Drone initialization failed");
        }
        
        RCLCPP_INFO(get_logger(), "JumpingSumo node initialized successfully");
    }

    ~JumpingSumoNode() {
        if (deviceController_ != NULL) {
            deviceState_ = ARCONTROLLER_Device_GetState(deviceController_, &error_);
            if ((error_ == ARCONTROLLER_OK) && 
                (deviceState_ != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
                ARCONTROLLER_Device_Stop(deviceController_);
                ARSAL_Sem_Wait(&stateSem_);
            }
            ARCONTROLLER_Device_Delete(&deviceController_);
        }
        
        if (device_ != NULL) {
            ARDISCOVERY_Device_Delete(&device_);
        }
        
        ARSAL_Sem_Destroy(&stateSem_);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<JumpingSumoNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("jumping_sumo"), 
                    "Node crashed with exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cinttypes> // for PRIu64, PRId64

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(17, 0, 0)
        #include <cv_bridge/cv_bridge.hpp>
#else
        #include <cv_bridge/cv_bridge.h>
#endif
#include <opencv4/opencv2/opencv.hpp>

extern "C"
{
#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
}

#define JS_IP_ADDRESS "192.168.2.1"
#define JS_DISCOVERY_PORT 44444
#define TAG "JumpingSumo"

class JumpingSumoNode : public rclcpp::Node
{
private:
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

    // Drone SDK objects
    ARDISCOVERY_Device_t *device_;
    ARCONTROLLER_Device_t *deviceController_;
    eARCONTROLLER_ERROR error_;
    eARCONTROLLER_DEVICE_STATE deviceState_;
    ARSAL_Sem_t stateSem_;

    bool initDrone()
    {
        eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

        // Create discovery device
        device_ = ARDISCOVERY_Device_New(&errorDiscovery);
        if (errorDiscovery != ARDISCOVERY_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create discovery device");
            return false;
        }

        // Initialize WiFi connection
        errorDiscovery = ARDISCOVERY_Device_InitWifi(device_,
                                                     ARDISCOVERY_PRODUCT_JS, "JS", JS_IP_ADDRESS, JS_DISCOVERY_PORT);
        if (errorDiscovery != ARDISCOVERY_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize WiFi connection");
            return false;
        }

        // Create device controller
        deviceController_ = ARCONTROLLER_Device_New(device_, &error_);
        if (error_ != ARCONTROLLER_OK)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create device controller");
            return false;
        }

        // Initialize semaphore
        ARSAL_Sem_Init(&stateSem_, 0, 0);

        // Add callbacks
        error_ = ARCONTROLLER_Device_AddStateChangedCallback(
            deviceController_, stateChanged, this);
        if (error_ != ARCONTROLLER_OK)
            return false;

        error_ = ARCONTROLLER_Device_SetVideoStreamCallbacks(
            deviceController_, decoderConfigCallback, didReceiveFrameCallback, NULL, this);
        if (error_ != ARCONTROLLER_OK)
            return false;

        // Register for sensor state changes
        error_ = ARCONTROLLER_Device_AddCommandReceivedCallback(
            deviceController_,
            commandReceivedCallback,
            this);
        if (error_ != ARCONTROLLER_OK)
            return false;

        // Start the device controller
        error_ = ARCONTROLLER_Device_Start(deviceController_);
        if (error_ != ARCONTROLLER_OK)
            return false;

        // Wait for device
        ARSAL_Sem_Wait(&stateSem_);

        deviceState_ = ARCONTROLLER_Device_GetState(deviceController_, &error_);
        if ((error_ != ARCONTROLLER_OK) || (deviceState_ != ARCONTROLLER_DEVICE_STATE_RUNNING))
        {
            RCLCPP_ERROR(get_logger(), "Failed to start device controller");
            return false;
        }

        // Enable video stream
        error_ = deviceController_->jumpingSumo->sendMediaStreamingVideoEnable(
            deviceController_->jumpingSumo, 1);
        if (error_ != ARCONTROLLER_OK)
            return false;

        return true;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!deviceController_)
            return;

        // Convert ROS velocity commands to drone commands
        // Forward/backward speed
        float speed = msg->linear.x * 100.0f; // Scale to percentage
        speed = std::min(std::max(speed, -100.0f), 100.0f);

        // Angular velocity for turning
        float turn = msg->angular.z * 100.0f; // Scale to percentage
        turn = std::min(std::max(turn, -100.0f), 100.0f);

        // Send commands to drone
        error_ = deviceController_->jumpingSumo->setPilotingPCMDFlag(
            deviceController_->jumpingSumo, 1);
        error_ = deviceController_->jumpingSumo->setPilotingPCMDSpeed(
            deviceController_->jumpingSumo, static_cast<int8_t>(speed));
        error_ = deviceController_->jumpingSumo->setPilotingPCMDTurn(
            deviceController_->jumpingSumo, static_cast<int8_t>(turn));
    }

    // Callback for command reception
    static void commandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY commandKey,
                                        ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                        void *customData)
    {
        JumpingSumoNode *node = static_cast<JumpingSumoNode *>(customData);

        // Debug: Print command key
        RCLCPP_DEBUG(node->get_logger(), "Received command key: %d", commandKey);

        // Get the command elements
        ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;
        HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
        if (element == NULL)
        {
            return;
        }

        // Process command based on key
        switch (commandKey)
        {
        // Battery state updates
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);
            if (arg != NULL)
            {
                uint8_t batteryPercentage = arg->value.U8;
                RCLCPP_INFO(node->get_logger(), "Battery: %d%%", batteryPercentage);

                // Create battery message
                auto battery_msg = std::make_unique<sensor_msgs::msg::BatteryState>();
                battery_msg->header.stamp = node->now();
                battery_msg->header.frame_id = "battery";

                // Set battery percentage (convert from 0-100 to 0.0-1.0)
                battery_msg->percentage = static_cast<float>(batteryPercentage) / 100.0f;

                // Set other fields with reasonable defaults
                battery_msg->voltage = std::numeric_limits<float>::quiet_NaN();         // Unknown
                battery_msg->current = std::numeric_limits<float>::quiet_NaN();         // Unknown
                battery_msg->charge = std::numeric_limits<float>::quiet_NaN();          // Unknown
                battery_msg->capacity = std::numeric_limits<float>::quiet_NaN();        // Unknown
                battery_msg->design_capacity = std::numeric_limits<float>::quiet_NaN(); // Unknown
                battery_msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
                battery_msg->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
                battery_msg->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
                battery_msg->present = true;

                // Publish battery state
                node->battery_pub_->publish(std::move(battery_msg));
            }
            break;
        }

        // Network quality information
        case ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_NETWORKSTATE_LINKQUALITYCHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_JUMPINGSUMO_NETWORKSTATE_LINKQUALITYCHANGED_QUALITY, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_U8)
            {
                uint8_t linkQuality = arg->value.U8;
                RCLCPP_INFO(node->get_logger(), "Network link quality: %d/5", linkQuality);
            }
            break;
        }

        // Date changed
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_CURRENTDATECHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_CURRENTDATECHANGED_DATE, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                RCLCPP_INFO(node->get_logger(), "Date changed: %s", arg->value.String);
            }
            break;
        }

        // Time changed
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_CURRENTTIMECHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_CURRENTTIMECHANGED_TIME, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                RCLCPP_INFO(node->get_logger(), "Time changed: %s", arg->value.String);
            }
            break;
        }

        // Product name changed
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTNAMECHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTNAMECHANGED_NAME, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                RCLCPP_INFO(node->get_logger(), "Product name: %s", arg->value.String);
            }
            break;
        }

        // Product serial high changed
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTSERIALHIGHCHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTSERIALHIGHCHANGED_HIGH, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                RCLCPP_INFO(node->get_logger(), "Product serial high: %s", arg->value.String);
            }
            break;
        }

        // Product serial low changed
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTSERIALLOWCHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTSERIALLOWCHANGED_LOW, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                RCLCPP_INFO(node->get_logger(), "Product serial low: %s", arg->value.String);
            }
            break;
        }

        // Product version changed
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            std::string software, hardware;

            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED_SOFTWARE, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                software = arg->value.String;
            }

            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED_HARDWARE, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                hardware = arg->value.String;
            }

            RCLCPP_INFO(node->get_logger(), "Product version - SW: %s, HW: %s",
                        software.c_str(), hardware.c_str());
            break;
        }

        // Country changed
        case ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_COUNTRYCHANGED:
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_COUNTRYCHANGED_CODE, arg);
            if (arg != NULL && arg->valueType == ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING)
            {
                RCLCPP_INFO(node->get_logger(), "Country code: %s", arg->value.String);
            }
            break;
        }

        // For all other commands, print the arguments for debugging
        default:
        {
            RCLCPP_DEBUG(node->get_logger(), "Command key: %d - Examining arguments:", commandKey);

            // Iterate through all arguments in this command
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            ARCONTROLLER_DICTIONARY_ARG_t *argTmp = NULL;

            // Iterate through all arguments
            HASH_ITER(hh, element->arguments, arg, argTmp)
            {
                if (arg != NULL)
                {
                    // Print the argument key (the key is the name used in the hash table)
                    RCLCPP_DEBUG(node->get_logger(), "  Arg key: %s", (const char *)arg->hh.key);

                    // Print the value based on its type
                    switch (arg->valueType)
                    {
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_U8:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: U8, Value: %d", arg->value.U8);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_I8:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: I8, Value: %d", arg->value.I8);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_U16:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: U16, Value: %d", arg->value.U16);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_I16:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: I16, Value: %d", arg->value.I16);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_U32:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: U32, Value: %u", arg->value.U32);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_I32:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: I32, Value: %d", arg->value.I32);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_U64:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: U64, Value: %lu", (unsigned long)arg->value.U64);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_I64:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: I64, Value: %ld", (long)arg->value.I64);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_FLOAT:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: Float, Value: %f", arg->value.Float);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_DOUBLE:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: Double, Value: %f", arg->value.Double);
                        break;
                    case ARCONTROLLER_DICTIONARY_VALUE_TYPE_STRING:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: String, Value: %s", arg->value.String);
                        break;
                    default:
                        RCLCPP_DEBUG(node->get_logger(), "    Type: Unknown (%d)", arg->valueType);
                        break;
                    }
                }
            }
            break;
        }
        }
    }

    // Callback when device state changes
    static void stateChanged(eARCONTROLLER_DEVICE_STATE newState,
                             eARCONTROLLER_ERROR /*error*/, void *customData)
    {
        JumpingSumoNode *node = static_cast<JumpingSumoNode *>(customData);
        switch (newState)
        {
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
        ARCONTROLLER_Stream_Codec_t codec, void *customData)
    {
        JumpingSumoNode *node = static_cast<JumpingSumoNode *>(customData);

        RCLCPP_INFO(node->get_logger(), "Video codec configuration received:");

        // Human readable codec type
        const char *codec_type = "Unknown";
        switch (codec.type)
        {
        case ARCONTROLLER_STREAM_CODEC_TYPE_H264:
            codec_type = "H.264";
            break;
        case ARCONTROLLER_STREAM_CODEC_TYPE_MJPEG:
            codec_type = "MJPEG";
            break;
        case ARCONTROLLER_STREAM_CODEC_TYPE_PCM16LE:
            codec_type = "PCM16LE (Audio)";
            break;
        default:
            break;
        }
        RCLCPP_INFO(node->get_logger(), "  Codec Type: %s (enum: %d)", codec_type, codec.type);

        // Codec specific parameters
        if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264)
        {
            RCLCPP_INFO(node->get_logger(), "  H.264 Parameters:");
            RCLCPP_INFO(node->get_logger(), "    SPS Buffer Size: %d bytes", codec.parameters.h264parameters.spsSize);
            RCLCPP_INFO(node->get_logger(), "    PPS Buffer Size: %d bytes", codec.parameters.h264parameters.ppsSize);
            RCLCPP_INFO(node->get_logger(), "    MP4 Compliant: %s",
                        codec.parameters.h264parameters.isMP4Compliant ? "Yes" : "No");
        }

        if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_DEFAULT)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid codec type received");
            return ARCONTROLLER_ERROR_STREAM;
        }
        return ARCONTROLLER_OK;
    }

    // Callback when video frame is received
    static eARCONTROLLER_ERROR didReceiveFrameCallback(
        ARCONTROLLER_Frame_t *frame, void *customData)
    {
        JumpingSumoNode *node = static_cast<JumpingSumoNode *>(customData);

        if (frame && frame->data && frame->used)
        {
            // Create image message
            auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
            img_msg->header.stamp = node->now();
            img_msg->header.frame_id = "camera_frame";

            // Set image properties for bgr8 format
            img_msg->height = 480;
            img_msg->width = 640;
            img_msg->encoding = "bgr8";
            img_msg->step = img_msg->width * 3; // 3 bytes per pixel for BGR
            img_msg->data.resize(img_msg->height * img_msg->step);

            try
            {
                // Decode frame based on codec type
                cv::Mat decoded_frame;

                // Create a cv::Mat from the raw frame data
                std::vector<uint8_t> frame_data(frame->data, frame->data + frame->used);
                decoded_frame = cv::imdecode(frame_data, cv::IMREAD_COLOR);

                if (decoded_frame.empty())
                {
                    RCLCPP_ERROR(node->get_logger(), "Failed to decode video frame");
                    return ARCONTROLLER_ERROR_STREAM;
                }

                // Convert OpenCV image to ROS message
                auto cv_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", decoded_frame);
                img_msg = std::make_unique<sensor_msgs::msg::Image>(*cv_ptr.toImageMsg());
                img_msg->header.stamp = node->now();
                img_msg->header.frame_id = "camera_frame";
            }
            catch (const cv::Exception &e)
            {
                RCLCPP_ERROR(node->get_logger(), "OpenCV error: %s", e.what());
                return ARCONTROLLER_ERROR_STREAM;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node->get_logger(), "Error processing frame: %s", e.what());
                return ARCONTROLLER_ERROR_STREAM;
            }

            node->image_pub_->publish(std::move(img_msg));
        }
        return ARCONTROLLER_OK;
    }

public:
    JumpingSumoNode() : Node("jumping_sumo")
    {
        // Initialize ROS interfaces
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "jumpingsumo/cmd_vel", 10,
            std::bind(&JumpingSumoNode::cmdVelCallback, this, std::placeholders::_1));

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "jumpingsumo/image_raw", 10);

        battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(
            "jumpingsumo/battery", 10);

        // Initialize drone connection
        if (!initDrone())
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize drone connection");
            throw std::runtime_error("Drone initialization failed");
        }

        RCLCPP_INFO(get_logger(), "JumpingSumo node initialized successfully");
    }

    ~JumpingSumoNode()
    {
        if (deviceController_ != NULL)
        {
            deviceState_ = ARCONTROLLER_Device_GetState(deviceController_, &error_);
            if ((error_ == ARCONTROLLER_OK) &&
                (deviceState_ != ARCONTROLLER_DEVICE_STATE_STOPPED))
            {
                ARCONTROLLER_Device_Stop(deviceController_);
                ARSAL_Sem_Wait(&stateSem_);
            }
            ARCONTROLLER_Device_Delete(&deviceController_);
        }

        if (device_ != NULL)
        {
            ARDISCOVERY_Device_Delete(&device_);
        }

        ARSAL_Sem_Destroy(&stateSem_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<JumpingSumoNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("jumping_sumo"),
                     "Node crashed with exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

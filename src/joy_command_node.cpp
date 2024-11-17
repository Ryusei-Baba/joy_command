#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

class JoyServiceNode : public rclcpp::Node
{
public:
    JoyServiceNode() : Node("joy_command_node"), button_state_(0)
    {
        // Joyメッセージのサブスクリプションを作成
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyServiceNode::joy_callback, this, std::placeholders::_1));

        // Triggerサービスのクライアントを作成
        client_ = this->create_client<std_srvs::srv::Trigger>("/waypoint_manager2/next_wp");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[1] != button_state_)
        {
            button_state_ = msg->buttons[1];
            if (button_state_ == 1)
            {
                call_service();
            }
        }
    }

    void call_service()
    {
        if (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        RCLCPP_INFO(this->get_logger(), "send next_wp");
        client_->async_send_request(request);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    int button_state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

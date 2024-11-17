#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

class JoyServiceNode : public rclcpp::Node
{
public:
    JoyServiceNode()
        : Node("joy_service_node"),
          previous_button_state_(0)
    {
        // サブスクライバの設定
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyServiceNode::joyCallback, this, std::placeholders::_1));

        // クライアントの設定
        service_client_ = this->create_client<std_srvs::srv::Trigger>("/waypoint_manager2/next_wp");

        RCLCPP_INFO(this->get_logger(), "JoyServiceNode initialized");
    }

private:
    // サブスクライバのコールバック
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        int current_button_state = msg->buttons[1]; // ボタン[1]の状態を取得

        if (current_button_state != previous_button_state_) {
            previous_button_state_ = current_button_state;

            if (current_button_state == 1) { // ボタンが押されたとき
                sendTriggerRequest();
            }
        }
    }

    // サービスリクエストの送信
    void sendTriggerRequest()
    {
        if (!service_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service '/waypoint_manager2/next_wp' not available");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // 非同期リクエストの送信
        auto future = service_client_->async_send_request(request);

        // 結果の処理（コールバックを使用）
        future.wait(); // 結果を待つ（ブロッキング）
        try {
            auto response = future.get(); // 結果を取得
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", response->message.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_;
    int previous_button_state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>

class JoyServiceNode : public rclcpp::Node
{
public:
    JoyServiceNode()
        : Node("joy_service_node"),
          button_pressed_(false)
    {
        // サブスクライバの設定
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyServiceNode::joyCallback, this, std::placeholders::_1));

        // クライアントの設定
        service_client_ = this->create_client<std_srvs::srv::Trigger>("/waypoint_manager2/next_wp");

        // タイマーの設定（サービス呼び出し間隔を制御）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000), // 5000msごとにサービスを送信
            std::bind(&JoyServiceNode::sendTriggerRequest, this));

        RCLCPP_INFO(this->get_logger(), "JoyServiceNode initialized");
    }

private:
    // サブスクライバのコールバック
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ボタン[1]の状態を取得
        button_pressed_ = (msg->buttons[1] == 1);
    }

    // サービスリクエストの送信
    void sendTriggerRequest()
    {
        // ボタンが押されていない場合は送信しない
        if (!button_pressed_) {
            return;
        }

        if (!service_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service '/waypoint_manager2/next_wp' not available");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // 非同期リクエストの送信
        auto future = service_client_->async_send_request(request);

        // 結果の処理（ログの出力のみ）
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
    rclcpp::TimerBase::SharedPtr timer_; // タイマーでサービス呼び出しを制御
    bool button_pressed_; // ボタン[1]が押されているかの状態
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

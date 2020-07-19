#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber()
            : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<MinimalSubscriber>();
    executor.add_node(node);
    while (rclcpp::ok()) {
        using namespace std::chrono_literals;
        executor.spin_some(100ms);
        std::this_thread::sleep_for(500ms);
    }
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"


class MyNode: public rclcpp::Node {
public:
    MyNode(): Node("cpp_test") {
        RCLCPP_INFO(this->get_logger(), "HELLO CPP NODE");
        timer = this->create_wall_timer(std::chrono::seconds(1), 
                                        std::bind(&MyNode::timerCallback, this));
    }

private:
    void timerCallback() {
        this->counter++;
        RCLCPP_INFO(this->get_logger(), "HELLO %d", this->counter);
    }

    rclcpp::TimerBase::SharedPtr timer;

    int counter = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
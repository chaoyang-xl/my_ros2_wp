#include "rclcpp/rclcpp.hpp"

//colcon build --packages-select my_cpp_pkg
class MyNode : public rclcpp::Node{

public:
    MyNode():Node("my_first_cpp_node"){
        RCLCPP_INFO(this->get_logger(),"Hello");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timer_callback, this)
        );
    }
private:
    void timer_callback(){
        RCLCPP_INFO(this->get_logger(),"Hello timer %d", counter_);
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_=0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // auto node = rclcpp::Node::make_shared("my_first_cpp_node");
    auto node = std::make_shared<MyNode>();
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello world");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
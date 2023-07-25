#include <memory> // use by shared pointer
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit Talker(const std::string & node_name, bool intraProcessesComms = false) 
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intraProcessesComms))
        {
            
        }

        void publish()
        {
            static size_t count = 0;
            auto msg = std::make_unique<std_msgs::msg::String>();
            msg->data = "Lifecycle talker says Hello "+ std::to_string(++count);

            if(!publisher->is_activated()) { RCLCPP_INFO(get_logger(), "Lifecycle Publisher currently inactive"); }
            else { RCLCPP_INFO(get_logger(), "Lifecycle Publisher currently active. Publishing [%s] ", msg->data.c_str()); }
            publisher->publish(std::move(msg));
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
        {
            publisher = this->create_publisher<std_msgs::msg::String>("my_messages",10);
            timer_ = this->create_wall_timer(1s, std::bind(&Talker::publish, this));

            RCLCPP_INFO(get_logger(), "on_configure() called");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
        {
            publisher->on_activate();
            std::this_thread::sleep_for(2s);

            RCLCPP_INFO(get_logger(), "on_activate() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;   
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
        {
            publisher->on_deactivate();

            RCLCPP_INFO(get_logger(), "on_deactivate() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;   
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
        {
            publisher.reset();
            timer_.reset();

            RCLCPP_INFO(get_logger(), "on_cleanup() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; 
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(get_logger(), "on_shutdown() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; 
        }

    private:
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher;
        std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<Talker> lc_talker_node = std::make_shared<Talker>("lc_talker");
    executor.add_node(lc_talker_node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
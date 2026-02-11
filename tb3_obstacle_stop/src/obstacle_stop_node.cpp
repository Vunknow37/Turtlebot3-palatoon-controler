#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

using namespace std;

class ObstacleStopNode : public rclcpp::Node
{
public:
    ObstacleStopNode() : Node("obstacle_stop_node")
    {
        // Parameters
        this->declare_parameter("safe_distance", 0.5);
        this->declare_parameter("detect_angle_deg", 180.0);
        this->declare_parameter("forward_speed", 0.2);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            bind(&ObstacleStopNode::scanCallback, this, placeholders::_1));

        timer_ = this->create_wall_timer(
            chrono::milliseconds(100),
            bind(&ObstacleStopNode::controlLoop, this));

        obstacle_detected_ = false;

        RCLCPP_INFO(this->get_logger(), "Obstacle Stop Node Started!");
    }

    ~ObstacleStopNode()
    {
        stopRobot();
        RCLCPP_WARN(this->get_logger(), "Node stopped. Robot forced to stop.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        detect_angle_deg_ = this->get_parameter("detect_angle_deg").as_double();

        // Convert to radians
        double detect_angle_rad = (detect_angle_deg_ * M_PI) / 180.0;

        float closest = msg->range_max;

        int total = msg->ranges.size();

        for (int i = 0; i < total; i++)
        {
            float angle = msg->angle_min + i * msg->angle_increment;

            // Check only within desired front angle range
            if (angle >= -detect_angle_rad / 2.0 && angle <= detect_angle_rad / 2.0)
            {
                float r = msg->ranges[i];

                if (std::isfinite(r) && r > msg->range_min && r < msg->range_max)
                {
                    if (r < closest)
                        closest = r;
                }
            }
        }

        obstacle_detected_ = (closest < safe_distance_);
    }

    void controlLoop()
    {
        forward_speed_ = this->get_parameter("forward_speed").as_double();

        geometry_msgs::msg::Twist cmd;

        if (obstacle_detected_)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Obstacle detected! STOPPING!");
        }
        else
        {
            cmd.linear.x = forward_speed_;
            cmd.angular.z = 0.0;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Moving forward...");
        }

        cmd_pub_->publish(cmd);
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;

        for (int i = 0; i < 5; i++)
        {
            cmd_pub_->publish(cmd);
            rclcpp::sleep_for(chrono::milliseconds(100));
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool obstacle_detected_;

    double safe_distance_;
    double detect_angle_deg_;
    double forward_speed_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ObstacleStopNode>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace simple_tf_kinematics_ns
{

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics(const std::string &name);

private:
    double last_x_;
    double x_increment_;
    int rotations_counter_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;

    rclcpp::TimerBase::SharedPtr timer_;
    void update_transform();
};

SimpleTfKinematics::SimpleTfKinematics(const std::string &name)
    : Node(name), last_x_(0.0), x_increment_(0.01), rotations_counter_(0)
{
    //  Static broadcaster (base_footprint -> base_link)
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "base_footprint";
    static_transform_stamped_.child_frame_id = "camera";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.2;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    // 2️ Dynamic broadcaster (odom -> base_footprint)
    dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(100ms, std::bind(&SimpleTfKinematics::update_transform, this));
}

void SimpleTfKinematics::update_transform()
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    dynamic_transform_stamped.header.stamp = get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "odom";
    dynamic_transform_stamped.child_frame_id = "base_footprint";

    last_x_ += x_increment_;
    dynamic_transform_stamped.transform.translation.x = last_x_;
    dynamic_transform_stamped.transform.translation.y = 0.0;
    dynamic_transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0.1 * rotations_counter_++);
    dynamic_transform_stamped.transform.rotation = tf2::toMsg(q);

    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped);
}

} // namespace simple_tf_kinematics_ns

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simple_tf_kinematics_ns::SimpleTfKinematics>("simple_tf_kinematics_node"));
    rclcpp::shutdown();
    return 0;
}

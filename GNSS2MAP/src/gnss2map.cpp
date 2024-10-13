#include "gnss2map/gnss2map.hpp"

Gnss_to_map::Gnss_to_map()
: Node("gnss_to_map")
{
    map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gnss2map", 1);
    
    // Subscribe to the NavSatFix topic
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fix", rclcpp::QoS{1}, std::bind(&Gnss_to_map::navsat_callback, this, std::placeholders::_1));

    target_frame = this->declare_parameter<std::string>("target_frame", "map");
    gnss_frame = this->declare_parameter<std::string>("gnss_frame", "gnss");
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void Gnss_to_map::navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg) {
    // Extract GPS coordinates
    const double latitude = navsat_msg->latitude;
    const double longitude = navsat_msg->longitude;
    const double altitude = navsat_msg->altitude;

    // Convert GPS coordinates to UTM
    geographic_msgs::msg::GeoPoint gps_msg;
    gps_msg.latitude = latitude;
    gps_msg.longitude = longitude;
    gps_msg.altitude = altitude;

    geodesy::UTMPoint utm;
    geodesy::fromMsg(gps_msg, utm);

    // Create and populate the PoseWithCovarianceStamped message
    gnss2map_msg.header.stamp = this->now();
    gnss2map_msg.header.frame_id = target_frame;

    // Set position and covariance
    gnss2map_msg.pose.pose.position.x = fmod(utm.easting, UTM2MGRS);
    gnss2map_msg.pose.pose.position.y = fmod(utm.northing, UTM2MGRS);
    gnss2map_msg.pose.pose.position.z = altitude;

    // Set a default covariance (example values)
    gnss2map_msg.pose.covariance[0] = 0.1;  // X position covariance
    gnss2map_msg.pose.covariance[7] = 0.1;  // Y position covariance
    gnss2map_msg.pose.covariance[14] = 0.1; // Z position covariance
    gnss2map_msg.pose.covariance[21] = 0.1; // X-Y correlation

    // Publish the PoseWithCovarianceStamped message
    map_pose_pub_->publish(gnss2map_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gnss_to_map>());
    rclcpp::shutdown();
    return 0;
}

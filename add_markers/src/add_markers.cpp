#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker CreateMarker(double x, double y, int32_t action, std::size_t id = 0)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the marker type.
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    return marker;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    double pickup_x = 0;
    double pickup_y = 0;
    double dropoff_x = 0;
    double dropoff_y = 0;

    while (ros::ok())
    {
        if (ros::param::get("pickup_x", pickup_x) && ros::param::get("pickup_y", pickup_y) &&
            ros::param::get("dropoff_x", dropoff_x) && ros::param::get("dropoff_y", dropoff_y))
        {
            std::cout << "Got pickup and dropoff locations:\n pickup: " << pickup_x << ", " << pickup_y << "\n dropoff: "
                      << dropoff_x << ", " << dropoff_y << "\nStart showing markers now" << std::endl;
            break;
        }
        else
        {
            r.sleep();
        }
    }

    marker_pub.publish(CreateMarker(pickup_x, pickup_y, visualization_msgs::Marker::ADD));
    ros::Duration(5).sleep();
    marker_pub.publish(CreateMarker(pickup_x, pickup_y, visualization_msgs::Marker::DELETE));
    ros::Duration(5).sleep();
    marker_pub.publish(CreateMarker(dropoff_x, dropoff_y, visualization_msgs::Marker::ADD));

    ros::spin();
}
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
class fake_baselink
{
private:
  float fake_yaw=0;
  geometry_msgs::Twist fake_twist;
  geometry_msgs::Twist base_twist;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  geometry_msgs::PoseStamped robot_pose_stamped;
  tf::StampedTransform fake2base;
  tf::StampedTransform map_to_base_footprint;
  ros::NodeHandle nh;
  ros::Rate loop_rate;
  bool GetGlobalRobotPose(const tf::TransformListener& tf_listener,
                        const std::string& target_frame,
                        geometry_msgs::PoseStamped& robot_global_pose);
  bool UpdateTransform(const tf::TransformListener& tf_listener,
                     const std::string& target_frame,
                     const std::string& source_frame,
                     const ros::Time& source_time,
                     tf::StampedTransform& target_to_source_transform,
                     const ros::Time& target_time,
                     const ros::Duration& timeout);
  void transformTwist(const geometry_msgs::Twist& in, geometry_msgs::Twist& output_v,const tf::Transform& transform);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  public:
  fake_baselink():fake_yaw(0),loop_rate(70)
  {
    ros::Subscriber cmd_vel_sub =nh.subscribe("/cmd_vel_fake", 10, &fake_baselink::cmdVelCallback,this);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber path_sub=nh.subscribe("/move_base/TebLocalPlannerROS/local_plan",100,&fake_baselink::pathCallback,this);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q;
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link_fake"));
     while (ros::ok())
    {
        // Publish the accumulated twist as base_link_fake

        // Broadcast the transform between base_link and base_link_fake
        q.setRPY(0,0,fake_yaw);
        transform.setRotation(q);
        ros::Time send_time=ros::Time::now();
        br.sendTransform(tf::StampedTransform(transform, send_time, "base_footprint", "base_link_fake"));
        
        UpdateTransform(listener, "base_footprint", "base_link_fake", send_time,fake2base, send_time, ros::Duration(0.5));
        transformTwist(fake_twist, base_twist, fake2base);
        
        // Publish the twist
        // std::cout<<"fv "<<fake_twist.linear.x<<" "<<fake_twist.linear.y<<std::endl;
        // std::cout<<"v "<<base_twist.linear.x<<" "<<base_twist.linear.y<<std::endl;
        cmd_vel_pub.publish(base_twist);
        loop_rate.sleep();
        ros::spinOnce();
    }
  }
};


void fake_baselink::pathCallback(const nav_msgs::Path::ConstPtr& msg){
    UpdateTransform(listener, "base_footprint", "map", ros::Time::now(),map_to_base_footprint, ros::Time(0), ros::Duration(0.5));
    tf::Quaternion quaternion;
    int path_size=msg->poses.size();
    std::cout << "path_size: " << path_size << std::endl;
    std::cout << "Yaw in map frame: " << tf::getYaw(msg->poses[path_size/3].pose.orientation) << std::endl;
    tf::quaternionMsgToTF(msg->poses[path_size/3].pose.orientation, quaternion);
    
    // Convert the quaternion from the map frame to the base_footprint frame
    tf::Quaternion af_quaternion = map_to_base_footprint * quaternion;
    
    // Get the yaw angle from the quaternion
    fake_yaw = tf::getYaw(af_quaternion);
    
    std::cout << "Yaw in base_footprint frame: " << fake_yaw << std::endl;
}

bool fake_baselink::GetGlobalRobotPose(const tf::TransformListener& tf_listener,
                        const std::string& target_frame,
                        geometry_msgs::PoseStamped& robot_global_pose){
  tf::Stamped<tf::Pose> robot_pose_tf;
  robot_pose_tf.setIdentity();
  robot_pose_tf.frame_id_ = "base_link";
  robot_pose_tf.stamp_ = ros::Time();

  tf::Stamped<tf::Pose> robot_global_pose_tf;
  try{
    tf_listener.transformPose( target_frame, robot_pose_tf, robot_global_pose_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Failed to transform robot pose: %s", ex.what());
    return false;
  }
  tf::poseStampedTFToMsg(robot_global_pose_tf, robot_global_pose);
  return true;
}
bool fake_baselink::UpdateTransform(const tf::TransformListener& tf_listener,
                     const std::string& target_frame,
                     const std::string& source_frame,
                     const ros::Time& source_time,
                     tf::StampedTransform& target_to_source_transform,
                     const ros::Time& target_time = ros::Time::now(),
                     const ros::Duration& timeout = ros::Duration(0.5)){
  try{
    tf_listener.waitForTransform(target_frame, target_time,
                                  source_frame, source_time,
                                  source_frame, timeout);
    tf_listener.lookupTransform(target_frame, target_time,
                                 source_frame, source_time,
                                 source_frame, target_to_source_transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Failed to update transform: %s", ex.what());
    return false;
  }

  return true;


}
void fake_baselink::transformTwist(const geometry_msgs::Twist& in, geometry_msgs::Twist& output_v,const tf::Transform& transform)
{
    // Convert the linear velocity
    tf::Vector3 linear(in.linear.x, in.linear.y, in.linear.z);
    tf::Vector3 af_linear = transform* linear;

    // Convert the angular velocity
    tf::Vector3 angular(in.angular.x, in.angular.y, in.angular.z);
    tf::Vector3 af_angular = transform * angular;
    output_v.linear.x = af_linear.x();
    output_v.linear.y = af_linear.y();
    output_v.linear.z = af_linear.z();
    output_v.angular.x = af_angular.x();
    output_v.angular.y = af_angular.y();
    output_v.angular.z = af_angular.z();

}
void fake_baselink::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // fake_yaw += msg->angular.z;
    fake_twist.linear.x=msg->linear.x;
    fake_twist.linear.y=msg->linear.y;
    fake_twist.linear.z=0;
    fake_twist.angular.x=0;
    fake_twist.angular.y=0;
    fake_twist.angular.z=0;
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "fake_baselink");
    fake_baselink fake_baselink_;
    return 0;
}


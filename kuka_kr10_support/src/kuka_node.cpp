#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cmath>

static const std::string PLANNING_GROUP_NAME = "manipulator";
static int nLinks;
static ros::Publisher display_publisher;
static ros::Publisher marker_pub;
static visualization_msgs::Marker link_list;
static visualization_msgs::Marker nodes;
static ros::Rate * r;
static geometry_msgs::Point testbed_offset;
static moveit::planning_interface::MoveGroup * move_group;
static moveit::planning_interface::PlanningSceneInterface * planning_scene_interface;  
static float cylinder_radius = 0.001;


//Takes in framefab point and returns it transformed to the coordinates of our testbed
geometry_msgs::Point transformPoint(geometry_msgs::Point pwf_point) {
   pwf_point.x += testbed_offset.x;
   pwf_point.y += testbed_offset.y;
   pwf_point.z += testbed_offset.z; //TODO likely need to find lowest link and offset all by some amount
   return pwf_point;
}


static std::vector<moveit_msgs::CollisionObject> collision_objects;

moveit_msgs::CollisionObject makeCollisionCylinder(geometry_msgs::Point start, geometry_msgs::Point end,std::string id) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.id = id;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  shape_msgs::SolidPrimitive link_cylinder;
  link_cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  link_cylinder.dimensions.resize(2);
    
  Eigen::Vector3d eStart, eEnd;
  tf::pointMsgToEigen(start, eStart);
  tf::pointMsgToEigen(end, eEnd);
  double height = (eStart-eEnd).lpNorm<2>();
  link_cylinder.dimensions[0] = height;
  link_cylinder.dimensions[1] = cylinder_radius;

  Eigen::Vector3d axis = eStart - eEnd;
  axis.normalize();
  Eigen::Vector3d zVec(0.0,0.0,1.0);
  Eigen::Vector3d xVec = axis.cross(zVec);
  xVec.normalize();
  double theta = axis.dot(zVec);
  double angle = -1.0 * acos(theta);
 
  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(xVec, tf_right_axis_vector);
  Eigen::Quaterniond q;
  // Create quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle);

  // Convert back to Eigen
  tf::quaternionTFToEigen(tf_q, q);
  q.normalize();
  Eigen::Affine3d pose;
  q.normalize();
  pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
  Eigen::Vector3d origin;
  tf::pointMsgToEigen(testbed_offset, origin);
  pose.translation() = origin + eStart;
  geometry_msgs::Pose linkPose;
  tf::poseEigenToMsg(pose , linkPose);
  collision_object.primitive_poses.push_back(linkPose);
  // Drawing frame w/ no collision
  //nodes.points.push_back(start);
  //nodes.points.push_back(end);
  //marker_pub.publish(nodes);
  //marker_pub.publish(link_list);
   
  collision_object.primitives.push_back(link_cylinder);
  
  return collision_object;
}

void drawAll() {

  planning_scene_interface->addCollisionObjects(collision_objects);
  r->sleep();
}
//Callback function for when framefab panel publishes links to draw and plan for
void frameCallback(geometry_msgs::PoseArray msg){
  std::string id = "link" + nLinks;
  nLinks++;
  geometry_msgs::Point start = msg.poses[0].position;
  geometry_msgs::Point end = msg.poses[1].position;

  moveit_msgs::CollisionObject collision_object = makeCollisionCylinder(start, end, id);
  collision_objects.push_back(collision_object);
  drawAll();
}    




//Initialize the LINE_LIST Marker display parameters
void initLinklist(){
  link_list.header.frame_id = "world";
  link_list.action = visualization_msgs::Marker::ADD;
  link_list.pose.orientation.w = 1.0;
  link_list.id = 0; 
  link_list.type = visualization_msgs::Marker::LINE_LIST;
  link_list.color.b = 1.0;
  link_list.color.a = 1.0;
  link_list.scale.x = 0.01;
  
  nodes.header.frame_id = "world";
  nodes.action = visualization_msgs::Marker::ADD;
  nodes.pose.orientation.w = 1.0;
  nodes.id = 1; 
  nodes.type = visualization_msgs::Marker::POINTS;
  nodes.color.g = 1.0f;
  nodes.color.a = 1.0;
  nodes.scale.x = 0.01;
  nodes.scale.y = 0.01;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kuka_node");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber frame_sub = node_handle.subscribe("framelinks", 0 , &frameCallback);
  //marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
  /* This sleep is ONLY to allow Rviz to come up */
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
  // initLinklist();
  r = new ros::Rate(20.0);
  r->sleep();
  
  testbed_offset.x = 0.5;
  testbed_offset.y = -0.5;
  testbed_offset.z = 0.33;
  nLinks = 0;  

  move_group = new moveit::planning_interface::MoveGroup("manipulator"); 
  // Publisher for visualizing plans in Rviz.
  display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  while(ros::ok()) {
    ros::spinOnce();
  } 
  ros::shutdown();
  return 0;
}

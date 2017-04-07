#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

static const std::string PLANNING_GROUP_NAME = "manipulator";

static ros::Publisher display_publisher;
static ros::Publisher marker_pub;
static visualization_msgs::Marker link_list;
static visualization_msgs::Marker nodes;
static ros::Rate * r;
static geometry_msgs::Point testbed_offset;
static moveit::planning_interface::MoveGroup * move_group;
static moveit::planning_interface::PlanningSceneInterface * planning_scene_interface;  
static moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

//Takes in framefab point and returns it transformed to the coordinates of our testbed
geometry_msgs::Point transformPoint(geometry_msgs::Point pwf_point) {
   pwf_point.x += testbed_offset.x;
   pwf_point.y += testbed_offset.y;
   pwf_point.z += testbed_offset.z; //TODO likely need to find lowest link and offset all by some amount
   return pwf_point;
}

float get_height(geometry_msgs::Point a, geometry_msgs::Point b) {
  return std::sqrt( a.x*b.x + a.y*b.y + a.z*b.z); 
}

//Callback function for when framefab panel publishes links to draw and plan for
void frameCallback(geometry_msgs::PoseArray msg){
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  shape_msgs::SolidPrimitive link_cylinder;
  link_cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  link_cylinder.dimensions.resize(2);
  link_cylinder.dimensions[0] = get_height(msg.poses[0].position, msg.poses[1].position);
  link_cylinder.dimensions[1] = 0.01;
 
  // Drawing frame w/ no collision
  link_list.header.stamp = ros::Time::now();
  geometry_msgs::Point start = transformPoint(msg.poses[0].position);
  geometry_msgs::Point end = transformPoint( msg.poses[1].position);
  link_list.points.push_back(start);
  link_list.points.push_back(end);  
  nodes.points.push_back(start);
  nodes.points.push_back(end);
  marker_pub.publish(nodes);
  marker_pub.publish(link_list);
   
  geometry_msgs::Pose p;
  p.position = testbed_offset;
  p.orientation.w = 1.0;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_object.primitives.push_back(link_cylinder);
  collision_object.primitive_poses.push_back(p);
  collision_objects.push_back(collision_object);
  std::cout << "Still not dead" << std::endl;
  planning_scene_interface->addCollisionObjects(collision_objects);
  r->sleep();
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
  marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
  /* This sleep is ONLY to allow Rviz to come up */
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
  initLinklist();
  r = new ros::Rate(20.0);
  r->sleep();
  
  testbed_offset.x = 1.0;
  testbed_offset.y = 0.0;
  testbed_offset.z = 0.33;
 
  move_group = new moveit::planning_interface::MoveGroup("manipulator"); 
  // Publisher for visualizing plans in Rviz.
  display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  while(ros::ok()) {
    ros::spinOnce();
  } 
  ros::shutdown();
  return 0;
}

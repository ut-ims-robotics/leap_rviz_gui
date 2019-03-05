#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <leap_motion/Human.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

#include <stdlib.h>
#include <std_msgs/Empty.h>

#include "leap_motion/leap.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <rviz_animated_view_controller/CameraTrajectory.h>

#include <leap_rviz_gui/rviz_visual.h>



Visuals* manager;

//Callbacks

//Functions

//hand 
//0 = left
//1 = right
bool hand_marker_collision(visualization_msgs::Marker marker, int hand){
	
	//getting info about correct hand
	geometry_msgs::PointStamped hand_point_stamped;
	if (hand == 0){
		if (manager->getUse_leap_hands()){
			if (manager->getLeft().is_present){
				hand_point_stamped.point = manager->getLeft().palm_center;
			}else{
				return false;
			}		
		}else {
			hand_point_stamped.point = manager->getMarker("custom_left").pose.position;
		}
	}else{
		
		if (manager->getUse_leap_hands()){
			if (manager->getRight().is_present){
				hand_point_stamped.point = manager->getRight().palm_center;
			}else{
				return false;
			}		
		}else {
			hand_point_stamped.point = manager->getMarker("custom_right").pose.position;
		}
	}
	
	//checing the shape of the object
	if (marker.type == visualization_msgs::Marker::SPHERE){
		double d_hand2marker;
		d_hand2marker = sqrt(pow((marker.pose.position.x - hand_point_stamped.point.x),2) + 
						pow((marker.pose.position.y - hand_point_stamped.point.y),2) + 
						pow((marker.pose.position.z - hand_point_stamped.point.z),2));

		if (d_hand2marker < (marker.scale.x/2)){
			return true;
		}

	}else if (marker.type == visualization_msgs::Marker::CUBE){
		if (((marker.pose.position.x + marker.scale.x/2) > hand_point_stamped.point.x) &&
			((marker.pose.position.x - marker.scale.x/2) < hand_point_stamped.point.x) &&
			((marker.pose.position.y + marker.scale.y/2) > hand_point_stamped.point.y) &&
			((marker.pose.position.y - marker.scale.y/2) < hand_point_stamped.point.y) &&
			((marker.pose.position.z + marker.scale.z/2) > hand_point_stamped.point.z) &&
			((marker.pose.position.z - marker.scale.z/2) < hand_point_stamped.point.z)){
				return true;
			}
	}
	
	
	return false;
}

geometry_msgs::Point tf_world_2_hand(geometry_msgs::Point p){
	// tf from world to handtf_l

	tf2_ros::Buffer tfBuffer;
	geometry_msgs::TransformStamped hand_to_world;
	geometry_msgs::PointStamped temp_point_stamped;
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	try{
		hand_to_world = tfBuffer.lookupTransform("world", "leap_hands", ros::Time(0), ros::Duration(1.0) );
		temp_point_stamped.point = p;
		tf2::doTransform(temp_point_stamped, temp_point_stamped, hand_to_world);
	}catch (tf2::LookupException e){
		ROS_ERROR_STREAM("Lets wait a bit" << e.what());
	}


	

	return temp_point_stamped.point;
}


//callbacks
void tmrLeapCallback(const ros::TimerEvent&){
    manager->framePublish();
}


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "visual_manager");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	manager = new Visuals();

	ros::Timer timer_leap_h_frame = node_handle.createTimer(ros::Duration(0.05), tmrLeapCallback);

	//Setup
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup* joint_model_group = 
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	//Visualization
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();
	
	visual_tools.loadRemoteControl();

	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 1.75;

	visual_tools.trigger();

	//TF variables
	tf2_ros::Buffer tfBuffer;
	geometry_msgs::TransformStamped hand_to_world;
	
	ros::Publisher pub_goal_ee_link = 
    node_handle.advertise<geometry_msgs::PoseStamped>("/rviz/moveit/move_marker/goal_ee_link", 1);
	
	ros::Publisher pub_update_goal_pose = 
    node_handle.advertise<std_msgs::Empty>("/rviz/moveit/update_goal_state", 1);
	std_msgs::Empty empty_cmd_msg;

	// geometry_msgs::PoseStamped pose;
	pub_update_goal_pose.publish(manager->getEmpty_msg());

	
	pub_goal_ee_link.publish(manager->getGoal_state_pose());
	pub_update_goal_pose.publish(empty_cmd_msg);
	

	
  	tf::TransformListener tf_listener;

	double dx = 0.025;	

	while(ros::ok()){		
		manager->publishMarkers();

		// manager->frameStart();
		switch(manager->getMode()){
			case 0: //normal mode
				
				
				//adjust camera
				if (manager->getAdjust_camera()){
					manager->publishCam();
					manager->checkCamStatus();
				}
				
				//goal_state_pose update
				if (manager->getRight().is_present){
					geometry_msgs::PoseStamped temp_pose_stmp = manager->getGoal_state_pose();
					temp_pose_stmp.pose.position = tf_world_2_hand(temp_pose_stmp.pose.position);

					pub_goal_ee_link.publish(temp_pose_stmp);
			
				}

			
			
				//controll if left hand is touching the memu object
			
				if (manager->getActivate_menu()) {
					if (hand_marker_collision(manager->getMarker("plan"), 0)){
						
						if (manager->getRight().is_present){
							
							geometry_msgs::Pose right_hand_palm_pose;
							right_hand_palm_pose.position = tf_world_2_hand(manager->getRight().palm_center);
							right_hand_palm_pose.orientation.w = 1;
							
							//robot planning
							move_group.setPoseTarget(right_hand_palm_pose);
						
							moveit::planning_interface::MoveGroupInterface::Plan my_plan;
							
							geometry_msgs::PoseStamped markerArray_pose_stamped_new;
							visual_tools.publishAxisLabeled(markerArray_pose_stamped_new.pose, "pose1");
							visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
							visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
							visual_tools.trigger();

							move_group.plan(my_plan);
						}
					}else if (hand_marker_collision(manager->getMarker("execute"), 0)){
						
						if (manager->getRight().is_present){
							

							geometry_msgs::Pose right_hand_palm_pose;
							right_hand_palm_pose.position = tf_world_2_hand(manager->getRight().palm_center);
							right_hand_palm_pose.orientation.w = 1;
							
							//robot planning
							move_group.setPoseTarget(right_hand_palm_pose);
						
							moveit::planning_interface::MoveGroupInterface::Plan my_plan;

							geometry_msgs::PoseStamped markerArray_pose_stamped_new;
							visual_tools.publishAxisLabeled(markerArray_pose_stamped_new.pose, "pose1");
							visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
							visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
							visual_tools.trigger();

							move_group.move();
						}
					}
					
					manager->close_menu();
					
				}else{
					if(hand_marker_collision(manager->getMarker("menu"), 0)){
						manager->setActivate_menu(true);

					}else{
						manager->setActivate_menu(false);
					}


				}

				if (hand_marker_collision(manager->getMarker("scale"), 0)){
					manager->changeScale();
				}else{
					if (manager->getSet_new_gain()){
						manager->setGainAdd(manager->getNew_gain());
						manager->setSet_new_gain(false);
					}
				}

				if (hand_marker_collision(manager->getMarker("camera"), 1)){
					manager->setMode(1);
				}
			


				break;
			case 1: //camera controlling mode
				if (manager->getAdjust_camera()){
					manager->publishCam();
					manager->checkCamStatus();	
				}else {
					//rviz_animated_view_controller::CameraTrajectory cam_tra = manager->getCam_tra();
					// rviz_animated_view_controller::CameraMovement cam_mov;
					// cam_mov = manager->getCam_tra().trajectory.pop_back();
					
  //if present missing
					if (hand_marker_collision(manager->getMarker("view_left"), 0) ||
						hand_marker_collision(manager->getMarker("view_left"), 1)){
						
						manager->setNewCamTra(0, dx);
						manager->setAdjust_camera(true);

						ROS_INFO_STREAM("Touching left");

					}else if (hand_marker_collision(manager->getMarker("view_right"), 0) ||
						hand_marker_collision(manager->getMarker("view_right"), 1)){
						
						manager->setNewCamTra(1, dx);
						manager->setAdjust_camera(true);

						//manager->frameChange();

						ROS_INFO_STREAM("Touching right");

					}else if (hand_marker_collision(manager->getMarker("view_up"), 0) ||
						hand_marker_collision(manager->getMarker("view_up"), 1)){
						
						manager->setNewCamTra(2, dx);
						manager->setAdjust_camera(true);

						//manager->frameChange();

						ROS_INFO_STREAM("Touching up");

					}else if (hand_marker_collision(manager->getMarker("view_down"), 0) ||
						hand_marker_collision(manager->getMarker("view_down"), 1)){
						
						manager->setNewCamTra(3, dx);
						manager->setAdjust_camera(true);

						//manager->frameChange();

						ROS_INFO_STREAM("Touching down");

					} 


					
				}

				

				break;
			default :
				manager->setMode(0);
		}
	}
}
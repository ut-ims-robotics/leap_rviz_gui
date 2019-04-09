
// ROS includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include <vector>
#include <map>
#include <leap_motion/Human.h>

#include <iomanip>
#include <tf/transform_broadcaster.h>


class Visuals
{
public:
  // ___ CONSTRUCTOR ___
  Visuals()
  { 
    generalMarkerInit();
    publisherInit();
    subscriberInit();
    cameraViewControllInit();
    adjust_camera_ = true;
    activate_menu_ = false;
    use_leap_hands_ = false;
    set_new_gain_ = false;
    
    new_gain_ = 1;
    mode_ = 0;
    gain_x_ = ORG_GAIN_X_;
    gain_y_ = ORG_GAIN_Y_;
    gain_z_ = ORG_GAIN_Z_;

    leap_frame_pos_x_ = 0.0;
    leap_frame_pos_y_ = 0.0;
    leap_frame_pos_z_ = 0.7;

    body_frame_rot_roll_ = 0;
    body_frame_rot_pitch_ = 0;
    body_frame_rot_yaw_ = M_PI;

    number_of_view_points_ = 10000;
  };


//getters, setters

leap_motion::Hand getLeft(){
  return left_;
}

leap_motion::Hand getRight(){
  return right_;
}


int getMode(){
  return mode_;
}

void setMode(int n){
  mode_ = n;
}

double getNew_gain(){
  return new_gain_;
}

bool getActivate_menu(){
  return activate_menu_;
}

void setActivate_menu(bool b){
  activate_menu_ = b;
}

bool getUse_leap_hands(){
  return use_leap_hands_;
}

bool getSet_new_gain(){
  return set_new_gain_;
}

void setSet_new_gain(bool b){
  set_new_gain_ = b;
}

bool getAdjust_camera(){
  return adjust_camera_;
}

void setAdjust_camera(bool b){
  adjust_camera_ = b;
}

std_msgs::Empty getEmpty_msg(){
  return empty_msg_;
}

geometry_msgs::PoseStamped getGoal_state_pose(){
  return goal_state_pose_;
}

visualization_msgs::Marker getMarker(const std::string& m){
  return marker_vec_[marker_map_[m]];
}

visualization_msgs::Marker getCustomRight(){
  int n = marker_map_["custom_right"];
  return marker_vec_[n];
}


//FUNCTIONS
void setGainAdd(double n){
	gain_x_ = ORG_GAIN_X_ + n;
  gain_z_ = ORG_GAIN_Z_ + n;

	if (gain_y_ < 3){
	    gain_y_ = ORG_GAIN_Y_ + n;
	}
}

void resetGain(){
	gain_x_ = ORG_GAIN_X_;
  gain_z_ = ORG_GAIN_Z_;
  gain_y_ = ORG_GAIN_Y_;
  
  int n_scale = marker_map_["scale"];
  marker_vec_[n_scale].pose.position.z = DIST_MENU_Z_;

}

std::string gainFormater(double gain){
  std::ostringstream streamObj;
  streamObj << std::fixed;
  streamObj << std::setprecision(2);
  streamObj << gain;
  return streamObj.str();
}

void publishMarkers(){
  int n;


  switch (mode_){
    case 0:
      if (activate_menu_) {
        n = marker_map_["plan"];
        pub_vec_[n].publish(marker_vec_[n]);
        n = marker_map_["plan_text"];
        pub_vec_[n].publish(marker_vec_[n]);
        
        n = marker_map_["execute"];
        pub_vec_[n].publish(marker_vec_[n]);
        n = marker_map_["execute_text"];
        pub_vec_[n].publish(marker_vec_[n]);

      }else{
        n = marker_map_["menu"];
        pub_vec_[n].publish(marker_vec_[n]);
        n = marker_map_["menu_text"];
        pub_vec_[n].publish(marker_vec_[n]);
      }
      
      n = marker_map_["scale"];
      pub_vec_[n].publish(marker_vec_[n]);
      
      n = marker_map_["scale_arrow_up"];
      pub_vec_[n].publish(marker_vec_[n]);
      n = marker_map_["scale_arrow_down"];
      pub_vec_[n].publish(marker_vec_[n]);
      
      n = marker_map_["scale_text"];
      pub_vec_[n].publish(marker_vec_[n]);
      
      n = marker_map_["gain_text"];
      //marker_vec_[n].text = gainFormater(gain_x_ - ORG_GAIN_X_ + 1);
      marker_vec_[n].text = gainFormater(gain_x_);
      pub_vec_[n].publish(marker_vec_[n]);




      n = marker_map_["camera"];
      pub_vec_[n].publish(marker_vec_[n]);
      n = marker_map_["camera_text"];
      pub_vec_[n].publish(marker_vec_[n]);

      n = marker_map_["instructions"];
      pub_vec_[n].publish(marker_vec_[n]);  

      break;


    case 1:
      
      n = marker_map_["view_left"];
      pub_vec_[n].publish(marker_vec_[n]);
      n = marker_map_["view_right"];
      pub_vec_[n].publish(marker_vec_[n]);
      n = marker_map_["view_down"];
      pub_vec_[n].publish(marker_vec_[n]);
      n = marker_map_["view_up"];
      pub_vec_[n].publish(marker_vec_[n]);
      n = marker_map_["back"];
      pub_vec_[n].publish(marker_vec_[n]);
      n = marker_map_["back_text"];
      pub_vec_[n].publish(marker_vec_[n]);

      n = marker_map_["instructions"];
      pub_vec_[n].publish(marker_vec_[n]);

      break;
    default:
      mode_ = 0;

    
  }
  
}

void publishCam(){
  pub_rviz_animated_.publish(cam_tra_);
}

// void replaceMarker(const std::string& name, visualization_msgs::Marker new_marker){
//   marker_vec_[marker_map_[name]] = new_marker;
// }

bool close_menu(){
	
  //check if left hand is present
  //if not close the menu
	if (!left_.is_present) {
		activate_menu_ = false;
		return true;
	}


	geometry_msgs::PointStamped left_point_stamped;
	left_point_stamped.point = left_.palm_center;
  
  //calc dist between the hand and obj
	double dist_x = abs(getMarker("menu").pose.position.x - left_point_stamped.point.x);
	double dist_y = abs(getMarker("menu").pose.position.y - left_point_stamped.point.y);
	
	
	if (dist_x > MENU_CLOSING_DIST_ || dist_y > MENU_CLOSING_DIST_){
		activate_menu_ = false;
    // ROS_INFO_STREAM("-----------------------");
    // ROS_INFO_STREAM(dist_x);
    // ROS_INFO_STREAM(dist_y);
    // ROS_INFO_STREAM("-----------------------");
    
		return true;
	}

	return false;
	
}

void changeScale(){
	float_t dif_org;
	int n_scale = marker_map_["scale"];
	if (use_leap_hands_) {
		dif_org = -(left_.palm_center.z - SCALE_START_POS_);
    marker_vec_[n_scale].pose.position.z = left_.palm_center.z; 
		
	}else{
    int n_left = marker_map_["custom_left"];
		dif_org = -(marker_vec_[n_left].pose.position.z - SCALE_START_POS_);
		marker_vec_[n_scale].pose.position.z = marker_vec_[n_left].pose.position.z; 		
	}
	
  new_gain_ = dif_org * 20.0;
  set_new_gain_ = true;
  //ROS_INFO_STREAM(marker_vec_[n_scale].pose.position.z);
  pub_vec_[n_scale].publish(marker_vec_[n_scale]);
}

void checkCamStatus(){

	double dist = sqrt(pow((cam_current_pose_.position.x - cam_tra_.trajectory[0].eye.point.x),2) + 
					   pow((cam_current_pose_.position.y - cam_tra_.trajectory[0].eye.point.y),2) + 
					   pow((cam_current_pose_.position.z - cam_tra_.trajectory[0].eye.point.z),2));	

	if (dist < CAM_MOV_TOLERANCE_){
		adjust_camera_ = false;
	}

}

void handFramePublish(){
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion q;
  q.setRPY(M_PI, 0, M_PI);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "leap_hands"));
  
}


void framePublish(){
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(leap_frame_pos_x_, leap_frame_pos_y_, leap_frame_pos_z_));
  tf::Quaternion q;
  q.setRPY(body_frame_rot_roll_, body_frame_rot_pitch_, body_frame_rot_yaw_);
  transform.setRotation(q);

  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "body"));
  
  handFramePublish();
  
}


/*
dir
0 -> left
1 -> right
2 -> up
3 -> down
*/
void setNewCamTra(int dir){
  rviz_animated_view_controller::CameraMovement cam_mov;

  cam_mov = cam_tra_.trajectory[0];
  double dist = 2.5;

  geometry_msgs::TransformStamped ts;
  geometry_msgs::Transform transform;

  switch (dir){
    case 0:
      body_frame_rot_yaw_ += M_PI / number_of_view_points_;
      body_frame_rot_yaw_ = fmod(body_frame_rot_yaw_, 2 * M_PI);
      framePublish();

      break;
    
    case 1: 
      body_frame_rot_yaw_ -= M_PI / number_of_view_points_;
      body_frame_rot_yaw_ = fmod(body_frame_rot_yaw_, 2 * M_PI);
      framePublish();
      
      break;
    
    case 2:
      body_frame_rot_roll_ += M_PI / number_of_view_points_;
      body_frame_rot_roll_ = fmod(body_frame_rot_roll_, 2 * M_PI);
      framePublish();

      break;
    case 3:
      body_frame_rot_roll_ -= M_PI / number_of_view_points_;
      body_frame_rot_roll_ = fmod(body_frame_rot_roll_, 2 * M_PI);
      framePublish();

      break;
    
  }

  cam_tra_.trajectory[0] = cam_mov;
}


geometry_msgs::Point transformBetweenFrames(geometry_msgs::Point p, const std::string& frameFrom, const std::string& frameTo){
	// tf from handframe to world

	tf2_ros::Buffer tfBuffer;
	geometry_msgs::TransformStamped trans;
	geometry_msgs::PointStamped temp_point_stamped;
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	try{
		trans = tfBuffer.lookupTransform(frameTo, frameFrom, ros::Time(0), ros::Duration(1.0) );
		temp_point_stamped.point = p;
		tf2::doTransform(temp_point_stamped, temp_point_stamped, trans);
	}catch (tf2::LookupException e){
		ROS_ERROR_STREAM("Lets wait a bit" << e.what());
	}

	return temp_point_stamped.point;
}


//CALLBACKS
void leapFilCallback(const leap_motion::Human& msg){
	left_ = msg.left_hand;
	right_ = msg.right_hand;
  int i = 0;
  
  if (msg.left_hand.is_present){
        int n = marker_map_["custom_left"];
        visualization_msgs::Marker m = marker_vec_[n];
        
        //getting a left hand pos and multiplying it with the gain
        m.pose.position.x = left_.palm_center.x * gain_x_;
        m.pose.position.y = left_.palm_center.y * gain_y_;
        m.pose.position.z = left_.palm_center.z * gain_z_;

        //ROS_INFO_STREAM(marker.pose.position);
        
        //getting left hand orientation
        float roll = left_.roll;
        float pitch = left_.pitch;
        float yaw = left_.yaw;

        //checking orientation, Leap Motion has a problem there with turing hands around
        tf2::Quaternion q;
        if (abs(yaw) > M_PI/2){
            q.setRPY(-(roll), pitch, yaw);            
        }else{
            q.setRPY(-(roll), -(pitch), yaw);                
        }

        q.normalize();
        
        
        m.pose.orientation.w = q.w();
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
    
        marker_vec_[n] = m;
        
        pub_vec_[n].publish(marker_vec_[n]);

        n = marker_map_["left_active"];

        marker_vec_[n].pose = m.pose;
                
        pub_vec_[n].publish(marker_vec_[n]);

        //Resetting gain if left pinch
        if (left_.pinch_strength > 0.97){
          resetGain();
        }

        
  }

  if (msg.right_hand.is_present){
      int n = marker_map_["custom_right"];
      visualization_msgs::Marker m = marker_vec_[n];
        
      //getting a right hand pos and multiplying it with the gain
      m.pose.position.x = right_.palm_center.x * gain_x_;
      m.pose.position.y = right_.palm_center.y * gain_y_;
      m.pose.position.z = right_.palm_center.z * gain_z_;

      //ROS_INFO_STREAM(marker.pose.position);


      
      //getting right hand orientation
      float roll = right_.roll;
      float pitch = right_.pitch;
      float yaw = right_.yaw;

      //checking orientation, Leap Motion has a problem with turing hands around yaw
      tf2::Quaternion q;
      if (abs(yaw) > M_PI/2){
          q.setRPY(-(roll), pitch, yaw);            
      }else{
          q.setRPY(-(roll), -(pitch), yaw);                
      }

      q.normalize();
      
      
      m.pose.orientation.w = q.w();
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();
  
      marker_vec_[n] = m;
    

      pub_vec_[n].publish(marker_vec_[n]);
    
      //!!!! make goal_state/robot arm to follow right hand

      if (right_.pinch_strength > 0.97){
        goal_state_pose_.pose = m.pose;
//        goal_state_pose_.pose.orientation = q;
      }
      

      n = marker_map_["right_active"];
      marker_vec_[n].pose = m.pose;
      pub_vec_[n].publish(marker_vec_[n]);
      
  }

}

void leapVisMarkerArrayCallback(const visualization_msgs::MarkerArray& msg){
	if (use_leap_hands_){
    pub_leap_hands_.publish(msg);
	}
}

void camCurrentCallback(const geometry_msgs::Pose& msg){
	cam_current_pose_ = msg;
}



private:
  // ___ INITIALIZERS ___
  void generalMarkerInit(){

    //markers init
    //markerInit(shape, mesh, text, pos_x, pos_y, pos_z, 
    //           sca_x, sca_y, sca_z, ns, r, g, b, a)
    
    //OBJECTS AND THEIR CORRESPONDING TEXTS

    marker_vec_.push_back(markerInit(SPHERE_, "", "", DIST_MENU_X_, DIST_MENU_Y_, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "menu", 0.4f, 1.0f, 1.0f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE_, "", "", DIST_MENU_X_, DIST_MENU_Y_, DIST_MENU_Z_ + 0.1, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "plan", 1.0f, 1.0f, 0.6f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE_, "", "", DIST_MENU_X_, DIST_MENU_Y_, DIST_MENU_Z_ - 0.1, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "execute", 0.0f, 1.0f, 0.0f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE_, "", "", DIST_MENU_X_ + 0.15, DIST_MENU_Y_, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "scale", 1.0f, 1.0f, 1.0f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE_, "", "", DIST_MENU_X_ - 0.4, DIST_MENU_Y_, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "camera", 1.0f, 0.5f, 1.0f, 1.0f));

    marker_vec_.push_back(markerInit(TEXT_, "", "MENU", DIST_MENU_X_ - 0.05, DIST_MENU_Y_ - 0.1, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "menu_text", 0.4f, 1.0f, 1.0f, 1.0f));
    marker_vec_.push_back(markerInit(TEXT_, "", "PLAN", DIST_MENU_X_ - 0.05, DIST_MENU_Y_ - 0.1, DIST_MENU_Z_ + 0.1, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "plan_text", 1.0f, 1.0f, 0.6f, 1.0f));
    marker_vec_.push_back(markerInit(TEXT_, "", "EXECUTE", DIST_MENU_X_ - 0.05, DIST_MENU_Y_ - 0.1, DIST_MENU_Z_ - 0.1, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "execute_text", 0.0f, 1.0f, 0.0f, 1.0f));
    marker_vec_.push_back(markerInit(TEXT_, "", "SCALE", DIST_MENU_X_ + 0.2, DIST_MENU_Y_ - 0.1, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "scale_text", 1.0f, 1.0f, 1.0f, 1.0f));
    marker_vec_.push_back(markerInit(TEXT_, "", gainFormater(gain_x_ - ORG_GAIN_X_ + 1), DIST_MENU_X_ + 0.3, DIST_MENU_Y_ - 0.1, DIST_MENU_Z_ + 0.1, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "gain_text", 1.0f, 0.8f, 1.0f, 1.0f));
    marker_vec_.push_back(markerInit(TEXT_, "", "VIEW", DIST_MENU_X_ - 0.4, DIST_MENU_Y_ - 0.1, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "camera_text", 1.0f, 0.5f, 1.0f, 1.0f));
    

    
    marker_vec_.push_back(markerInit(TEXT_, "", INSTRUCTIONS_, 0, 2, -1, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "instructions", 1.0f, 1.0f, 0.2f, 1.0f));



    //SCALER ARROWS

    marker_vec_.push_back(arrowMarkerInit(DIST_MENU_X_ + 0.25, DIST_MENU_Y_, DIST_MENU_Z_, 
                          0.03, 0.07, 0, 0, -0.2, "scale_arrow_up", 1.0f, 1.0f, 1.0f, 1.0f));
    marker_vec_.push_back(arrowMarkerInit(DIST_MENU_X_ + 0.25, DIST_MENU_Y_, DIST_MENU_Z_, 
                          0.03, 0.07, 0, 0, 0.2, "scale_arrow_down", 1.0f, 1.0f, 1.0f, 1.0f));



    //CAMERA MOVEMENT
    
    marker_vec_.push_back(markerInit(CUBE_, "", "", 0.3, DIST_MENU_Y_, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "view_left", 1.0f, 1.0f, 0.2f, 0.4f));
    marker_vec_.push_back(markerInit(CUBE_, "", "", -0.3, DIST_MENU_Y_, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "view_right", 1.0f, 1.0f, 0.2f, 0.4f));
    marker_vec_.push_back(markerInit(CUBE_, "", "", 0, DIST_MENU_Y_, DIST_MENU_Z_ + 0.2, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "view_down", 1.0f, 1.0f, 0.2f, 0.4f));
    marker_vec_.push_back(markerInit(CUBE_, "", "", 0, DIST_MENU_Y_, DIST_MENU_Z_ - 0.2, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "view_up", 1.0f, 1.0f, 0.2f, 0.4f));
    
    marker_vec_.push_back(markerInit(SPHERE_, "", "", 0, DIST_MENU_Y_, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "back", 0.4f, 1.0f, 1.0f, 0.4f));
    marker_vec_.push_back(markerInit(TEXT_, "", "BACK", 0, DIST_MENU_Y_ - 0.1, DIST_MENU_Z_, 
                          SCALE_X_, SCALE_Y_, SCALE_Z_, "back_text", 0.4f, 1.0f, 1.0f, 1.0f));
    

    //HANDS
  
    marker_vec_.push_back(markerInit(MESH_, "package://leap_rviz_gui/stl/Black-HandLeft.stl", "", 0, 0, 0, 
                          0.01, 0.01, 0.01, "custom_left", 1.0f, 0.0f, 0.0f, 1.0f));
    marker_vec_.push_back(markerInit(MESH_, "package://leap_rviz_gui/stl/Black-HandRight.stl", "", 0, 0, 0, 
                          0.01, 0.01, 0.01, "custom_right", 0.0f, 0.0f, 1.0f, 1.0f));



    marker_vec_.push_back(markerInit(CUBE_, "", "", 0, 0, 0, 
                          SCALE_ACT_X_, SCALE_ACT_Y_, SCALE_ACT_Z_, "left_active", 1.0f, 0.0f, 0.0f, 0.3f));
    marker_vec_.push_back(markerInit(CUBE_, "", "", 0, 0, 0, 
                          SCALE_ACT_X_, SCALE_ACT_Y_, SCALE_ACT_Z_, "right_active", 0.0f, 0.0f, 1.0f, 0.3f));
    

    fillMap();

    //goal_state init
    goal_state_pose_.header.frame_id = "/base_link";
    //goal_state_pose_.pose.orientation.w = 1.0;

  }

  void publisherInit(){
    //markers publishers
    for (const auto& m : marker_vec_){
      pub_vec_.push_back(node_handle_.advertise<visualization_msgs::Marker>("/visual_manager/" + m.ns + "_marker", 1));
    }

    //leap hands publisher
    pub_leap_hands_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/visual_manager/leap_motion/visualization_marker_array", 1);
  
    //camera publisher
    pub_rviz_animated_ = node_handle_.advertise<rviz_animated_view_controller::CameraTrajectory>("/rviz/camera_trajectory", 1);

  }

  void subscriberInit(){
    //https://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
    
    //leap motion
    sub_vec_.push_back(node_handle_.subscribe("/leap_motion/visualization_marker_array", 1000, &Visuals::leapVisMarkerArrayCallback, this));
    sub_vec_.push_back(node_handle_.subscribe("/leap_motion/leap_filtered", 1000, &Visuals::leapFilCallback, this));
    
    //camera
    sub_vec_.push_back(node_handle_.subscribe("/rviz/current_camera_pose", 1000, &Visuals::camCurrentCallback, this));
    
  }

  void cameraViewControllInit(){
	
    rviz_animated_view_controller::CameraMovement cam_mov;
    cam_mov.transition_time.sec = 0.2;
    cam_mov.eye.header.frame_id = "body";
    cam_mov.eye.point.x = 0;
    cam_mov.eye.point.y = -2.5;
    cam_mov.eye.point.z = 0;
    cam_mov.focus.header.frame_id = "body";
    cam_mov.focus.point.x = 0;//0.15;
    cam_mov.focus.point.y = 0;//-0.1;
    cam_mov.focus.point.z = 0;//0.8;
    cam_mov.up.header.frame_id = "body";
    cam_mov.up.vector.z = 1;

    cam_tra_.target_frame = "body";
    cam_tra_.allow_free_yaw_axis = false;
    cam_tra_.interaction_disabled = false;
    cam_tra_.mouse_interaction_mode = 0;

    // ROS_INFO_STREAM(cam_tra);
    // ROS_INFO_STREAM(cam_mov);
    
    cam_tra_.trajectory.push_back(cam_mov);
	
  }

  // ___ HELPER FUNCTIONS ___
  visualization_msgs::Marker markerInit(uint32_t shape, const std::string& mesh, const std::string& text, float_t pos_x, float_t pos_y, float_t pos_z, 
          float_t sca_x, float_t sca_y, float_t sca_z, const char* ns, float_t r, float_t g, float_t b, float_t a){
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "leap_hands";
    //marker.header.frame_id = "body";
    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    //marker.ns = "planning_ball";
    marker.ns = ns;
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    marker.mesh_resource = mesh;
    marker.mesh_use_embedded_materials = true;

    marker.text = text;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos_x; //0.2;
    marker.pose.position.y = pos_y; //0.2;
    marker.pose.position.z = pos_z; //0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = sca_x;
    marker.scale.y = sca_y;
    marker.scale.z = sca_z;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration(0.1);


    return marker;
  }

  visualization_msgs::Marker arrowMarkerInit(float_t pos_x, float_t pos_y, float_t pos_z, 
          float_t shaft_d, float_t head_d, float_t head_l,float_t start_z,float_t end_z, const char* ns, float_t r, float_t g, float_t b, float_t a){
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "leap_hands";
    //marker.header.frame_id = "body";
    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    //marker.ns = "planning_ball";
    marker.ns = ns;
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = ARROW_;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos_x; //0.2;
    marker.pose.position.y = pos_y; //0.2;
    marker.pose.position.z = pos_z; //0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = start_z;

    marker.points.push_back(p);
    p.z = end_z;
    marker.points.push_back(p);
    

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = shaft_d;
    marker.scale.y = head_d;
    marker.scale.z = head_l;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration(0.1);


    return marker;
  }



  void fillMap(){
    for(int i = 0; i < marker_vec_.size(); i++){
      marker_map_[marker_vec_[i].ns] = i;
    }
  }






  // ___ CLASS VARIABLES AND CONSTANTS ___



  ros::NodeHandle node_handle_;
  std::map<std::string,int> marker_map_; 
  std::vector<visualization_msgs::Marker> marker_vec_;
  std::vector<ros::Publisher> pub_vec_;
  std::vector<ros::Subscriber> sub_vec_;
  ros::Publisher pub_leap_hands_;
  leap_motion::Hand left_;
  leap_motion::Hand right_;
  rviz_animated_view_controller::CameraTrajectory cam_tra_;
  ros::Publisher pub_rviz_animated_;
  geometry_msgs::Pose cam_current_pose_;
  geometry_msgs::PoseStamped goal_state_pose_;
  std_msgs::Empty empty_msg_;

  
  //tf2_ros::StaticTransformBroadcaster leap_frame_br_;
  
  float leap_frame_pos_x_;
  float leap_frame_pos_y_;
  float leap_frame_pos_z_;
  
  float body_frame_rot_roll_;
  float body_frame_rot_pitch_;
  float body_frame_rot_yaw_;
  
  /*
  mode 0 - normal, controlling robot
  mode 1 - controlling view using animated camera
  */
  int mode_;

  bool adjust_camera_;
  bool activate_menu_;
  bool use_leap_hands_;
  bool set_new_gain_;

  double new_gain_;
  double gain_x_;
  double gain_y_;
  double gain_z_;

  int number_of_view_points_;


  //CONST
  // tf::Vector3 LEAP_ORG_POS_ = tf::Vector3(0, 0, 0.7);
  // tf::Quaternion LEAP_ORG_ROT_ = tf::Quaternion(3.14, 0, 0);
  const double MENU_CLOSING_DIST_ = 0.05;
  const double ORG_GAIN_X_ = 2;
  const double ORG_GAIN_Y_ = 2;
  const double ORG_GAIN_Z_ = 1.5;

  const uint32_t SPHERE_ = visualization_msgs::Marker::SPHERE;
  const uint32_t CUBE_ = visualization_msgs::Marker::CUBE;
  const uint32_t TEXT_ = visualization_msgs::Marker::TEXT_VIEW_FACING;
  const uint32_t MESH_ = visualization_msgs::Marker::MESH_RESOURCE;
  const uint32_t ARROW_ = visualization_msgs::Marker::ARROW;
  

  const float_t DIST_MENU_X_ = 0.2;
	const float_t DIST_MENU_Y_ = 0.4;
	const float_t DIST_MENU_Z_ = 0;
  const float SCALE_ACT_X_ = 0.2;
  const float SCALE_ACT_Y_ = 0.1;
  const float SCALE_ACT_Z_ = 0.2; 
  const float_t SCALE_X_ = 0.1;
  const float_t SCALE_Y_ = 0.1;
  const float_t SCALE_Z_ = 0.1;
  const float_t SCALE_START_POS_ = DIST_MENU_Z_;
  const double CAM_MOV_TOLERANCE_ = 0.1;
  
  std::string INSTRUCTIONS_ = "Instruction:\n Using left hand (RED) user can:\n change the scale (up and down)\n open a menu and choose to plan or execute current goal pose (right hand needs to be present)\n reset scaler by pinching\n Using right hand (BLUE) user can:\n controll goal state by pinching\n change view by touching view marker";

};

// #endif

// ROS includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include <vector>
#include <map>
#include <leap_motion/Human.h>

#include "geometry_msgs/TransformStamped.h"
// #include <tf2/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
// #include <tf/static_transform_broadcaster.h>


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

    released_left_ = true;
    released_right_ = true;
    released_up_ = true;
    released_down_ = true;


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


/*dir
0 = left
1 = right
2 = up
3 = down
*/
void setReleased(int dir, bool b){
  switch(dir){
    case 0:
      released_left_ = b;
      break;
    case 1:
      released_right_ = b;
      break;
    case 2:
      released_up_ = b;
      break;
    case 3:
      released_down_ = b;
      break;
  }
}


//FUNCTIONS
void setGainAdd(double n){
	gain_x_ = ORG_GAIN_X_ + n;
  gain_z_ = ORG_GAIN_Z_ + n;

	if (gain_y_ < 3){
	    gain_y_ = ORG_GAIN_Y_ + n;
	}
}

void publishMarkers(){
  int n;

  switch (mode_){
    case 0:
      if (activate_menu_) {
        n = marker_map_["plan"];
        pub_vec_[n].publish(marker_vec_[n]);
        n = marker_map_["execute"];
        pub_vec_[n].publish(marker_vec_[n]);

      }else{
        n = marker_map_["menu"];
        pub_vec_[n].publish(marker_vec_[n]);

      }
      
      n = marker_map_["scale"];
      pub_vec_[n].publish(marker_vec_[n]);
      
      n = marker_map_["camera"];
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
      n = marker_map_["to_menu"];
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
    ROS_INFO_STREAM("-----------------------");
    ROS_INFO_STREAM(dist_x);
    ROS_INFO_STREAM(dist_y);
    ROS_INFO_STREAM("-----------------------");
    
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
  ROS_INFO_STREAM(marker_vec_[n_scale].pose.position.z);
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

// float radToDeg(float rad){
//   return rad * 180 / M_PI;
// }


void printFrameRot(){
  ROS_INFO_STREAM(body_frame_rot_roll_);
  ROS_INFO_STREAM(body_frame_rot_pitch_);
  ROS_INFO_STREAM(body_frame_rot_yaw_);
}

void handFramePublish(){
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion q;
  q.setRPY(M_PI, 0, M_PI);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "leap_hands"));
  // printFrameRot();
}


void framePublish(){
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(leap_frame_pos_x_, leap_frame_pos_y_, leap_frame_pos_z_));
  tf::Quaternion q;
  q.setRPY(body_frame_rot_roll_, body_frame_rot_pitch_, body_frame_rot_yaw_);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
  
  handFramePublish();
  // printFrameRot();
}



double calc3rdPoint(double dist, double a, double b ){

  /* 
  dist2 = (a)2 + (b)2 + (C)2
  (C)2 = dist2 - (a)2 - (b)2
  C = sqrt(dist2 - (a)2 - (b)2)
  */
  return sqrt(pow(dist, 2) - pow(a, 2) - pow(b, 2));
}

/*
dir
0 -> left
1 -> right
2 -> up
3 -> down
*/
void setNewCamTra(int dir, int n){
  rviz_animated_view_controller::CameraMovement cam_mov;

  cam_mov = cam_tra_.trajectory[0];
  double dist = 2.5;



  geometry_msgs::TransformStamped ts;
  geometry_msgs::Transform transform;

  switch (dir){
    case 0:
      body_frame_rot_yaw_ += M_PI / n;
      framePublish();
      // printFrameRot();
      
      break;
    
    case 1: 
      body_frame_rot_yaw_ -= M_PI / n;
      framePublish();
      //  printFrameRot();
      
      break;
    
    case 2:
      body_frame_rot_roll_ += M_PI / n;
      framePublish();
      //  printFrameRot();

      break;
    case 3:
      body_frame_rot_roll_ -= M_PI / n;
      framePublish();
      //  printFrameRot();

      break;
    
  }

  cam_tra_.trajectory[0] = cam_mov;
}








//CALLBACKS
void leapFilCallback(const leap_motion::Human& msg){
	left_ = msg.left_hand;
	right_ = msg.right_hand;

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
        
        // ROS_INFO_STREAM("------------------------------");
        // ROS_INFO_STREAM(roll);
        // ROS_INFO_STREAM(radToDeg(roll));
        // ROS_INFO_STREAM("------------------------------");
        
        // ROS_INFO_STREAM(rad2deg(roll));
        // // //ROS_INFO_STREAM(pitch);
        // ROS_INFO_STREAM(yaw);

        pub_vec_[n].publish(marker_vec_[n]);

        n = marker_map_["left_active"];

        marker_vec_[n].pose = m.pose;
        
        //marker_vec[n].pose.position.y = m.pose.position.y + sin(radToDeg(roll)) * distToHandC;
        //marker_vec[n].pose.position.y = m.pose.position.x + asin(radToDeg(roll)) * distToHandC;
        
        pub_vec_[n].publish(marker_vec_[n]);
  }

  if (msg.right_hand.is_present){
      int n = marker_map_["custom_right"];
      visualization_msgs::Marker m = marker_vec_[n];
        
      //getting a left hand pos and multiplying it with the gain
      m.pose.position.x = right_.palm_center.x * gain_x_;
      m.pose.position.y = right_.palm_center.y * gain_y_;
      m.pose.position.z = right_.palm_center.z * gain_z_;

      //ROS_INFO_STREAM(marker.pose.position);


      
      //getting left hand orientation
      float roll = right_.roll;
      float pitch = right_.pitch;
      float yaw = right_.yaw;

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
      
      // ROS_INFO_STREAM("------------------------------");
      // ROS_INFO_STREAM(roll);
      // ROS_INFO_STREAM(radToDeg(roll));
      // ROS_INFO_STREAM("------------------------------");
      
      // ROS_INFO_STREAM(rad2deg(roll));
      // // //ROS_INFO_STREAM(pitch);
      // ROS_INFO_STREAM(yaw);


      //!!!! make goal_state/robot arm to follow right hand
      pub_vec_[n].publish(marker_vec_[n]);
      goal_state_pose_.pose = m.pose;

      n = marker_map_["right_active"];

      marker_vec_[n].pose = m.pose;
      
      //marker_vec[n].pose.position.y = m.pose.position.y + sin(radToDeg(roll)) * distToHandC;
      //marker_vec[n].pose.position.y = m.pose.position.x + asin(radToDeg(roll)) * distToHandC;
      
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
    marker_vec_.push_back(markerInit(SPHERE, "", DIST_MENU_X_, DIST_MENU_Y_, DIST_MENU_Z_, SCALE_X_, SCALE_Y_, SCALE_Z_, "menu", 0.4f, 1.0f, 1.0f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE, "", DIST_MENU_X_, DIST_MENU_Y_, DIST_MENU_Z_ + 0.1, SCALE_X_, SCALE_Y_, SCALE_Z_, "plan", 1.0f, 1.0f, 0.6f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE, "", DIST_MENU_X_, DIST_MENU_Y_, DIST_MENU_Z_ - 0.1, SCALE_X_, SCALE_Y_, SCALE_Z_, "execute", 0.0f, 1.0f, 0.0f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE, "", DIST_MENU_X_ + 0.15, DIST_MENU_Y_, DIST_MENU_Z_, SCALE_X_, SCALE_Y_, SCALE_Z_, "scale", 1.0f, 1.0f, 1.0f, 1.0f));
    marker_vec_.push_back(markerInit(SPHERE, "", DIST_MENU_X_ - 0.4, DIST_MENU_Y_, DIST_MENU_Z_, SCALE_X_, SCALE_Y_, SCALE_Z_, "camera", 1.0f, 0.5f, 1.0f, 1.0f));

    
    marker_vec_.push_back(markerInit(CUBE, "", 0.3, DIST_MENU_Y_, DIST_MENU_Z_, SCALE_X_, SCALE_Y_, SCALE_Z_, "view_left", 1.0f, 1.0f, 0.2f, 0.4f));
    marker_vec_.push_back(markerInit(CUBE, "", -0.3, DIST_MENU_Y_, DIST_MENU_Z_, SCALE_X_, SCALE_Y_, SCALE_Z_, "view_right", 1.0f, 1.0f, 0.2f, 0.4f));
    marker_vec_.push_back(markerInit(CUBE, "", 0, DIST_MENU_Y_, DIST_MENU_Z_ + 0.2, SCALE_X_, SCALE_Y_, SCALE_Z_, "view_down", 1.0f, 1.0f, 0.2f, 0.4f));
    marker_vec_.push_back(markerInit(CUBE, "", 0, DIST_MENU_Y_, DIST_MENU_Z_ - 0.2, SCALE_X_, SCALE_Y_, SCALE_Z_, "view_up", 1.0f, 1.0f, 0.2f, 0.4f));
    
    marker_vec_.push_back(markerInit(SPHERE, "", 0, DIST_MENU_Y_, DIST_MENU_Z_, SCALE_X_, SCALE_Y_, SCALE_Z_, "to_menu", 0.4f, 1.0f, 1.0f, 0.4f));
    

  
    marker_vec_.push_back(markerInit(visualization_msgs::Marker::MESH_RESOURCE, "package://leap_rviz_gui/stl/Black-HandLeft.stl", 0, 0, 0, 0.01, 0.01, 0.01, "custom_left", 1.0f, 0.0f, 0.0f, 1.0f));
    marker_vec_.push_back(markerInit(visualization_msgs::Marker::MESH_RESOURCE, "package://leap_rviz_gui/stl/Black-HandRight.stl", 0, 0, 0, 0.01, 0.01, 0.01, "custom_right", 0.0f, 0.0f, 1.0f, 1.0f));

    marker_vec_.push_back(markerInit(CUBE, "", 0, 0, 0, SCALE_ACT_X_, SCALE_ACT_Y_, SCALE_ACT_Z_, "left_active", 1.0f, 0.0f, 0.0f, 0.3f));
    marker_vec_.push_back(markerInit(CUBE, "", 0, 0, 0, SCALE_ACT_X_, SCALE_ACT_Y_, SCALE_ACT_Z_, "right_active", 0.0f, 0.0f, 1.0f, 0.3f));
    

    fillMap();

    //goal_state init
    goal_state_pose_.header.frame_id = "/base_link";
    goal_state_pose_.pose.orientation.w = 1.0;

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
    
    // cam_tra.transition_time.secs = 3;
    // cam_tra.eye.header.frame_id = "base_link";
	
  }

  // ___ HELPER FUNCTIONS ___
  visualization_msgs::Marker markerInit(uint32_t shape, const std::string& mesh, float_t pos_x, float_t pos_y, float_t pos_z, float_t sca_x, float_t sca_y, float_t sca_z, const char* ns, float_t r, float_t g, float_t b, float_t a){
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


  const double ORG_GAIN_X_ = 2;
  const double ORG_GAIN_Y_ = 2;
  const double ORG_GAIN_Z_ = 1.5;


  bool released_left_;
  bool released_right_;
  bool released_up_;
  bool released_down_;

  //CONST
  // tf::Vector3 LEAP_ORG_POS_ = tf::Vector3(0, 0, 0.7);
  // tf::Quaternion LEAP_ORG_ROT_ = tf::Quaternion(3.14, 0, 0);
  const double MENU_CLOSING_DIST_ = 0.05;
  const uint32_t SPHERE = visualization_msgs::Marker::SPHERE;
  const uint32_t CUBE = visualization_msgs::Marker::CUBE;
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
  //const double DIST_HAND_C_ = 0.2;
};

// #endif
#include <ros/ros.h>

#include "phantom_omni/PhantomButtonEvent.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#include "workspace_transformation/setAlgorithm.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>


#ifndef PI
#define PI 3.141592
#endif


#define N_HAPTIC_JOINTS 6

using namespace Eigen;


// Global variables
double scale[3];	// NewWorkspace / BaseWorkspace

bool m_init, mi_init, s_init, s0_init, algorithm_set;
bool mi_init_changed;

Matrix<double,3,1> pm_i, pm_im1, ps_i, ps_im1, pm_0, ps_0, ps, pm, dm, ds, vm_i, p_aux, dm_aux, ds_aux;
Matrix3d Rm_i(3,3), Rs_i(3,3), Rm_0(3,3), Rs_0(3,3), Rm(3,3), Rs(3,3), Ks(3,3);
Quaternion<double> quatm_0, quatm_i, quatm, quats, quats_0, quats_i;

bool first_haptic_angle_read;
double haptic_angles[N_HAPTIC_JOINTS];

Eigen::Transform<double,3,Affine> T_camPose_S;
double origin_cam_dist;

// Indexing
Eigen::Transform<double,3,Affine> T_imInitIndex_m_M, T_0sInitIndex_s_S;
bool first_time_indexing = false;
Matrix<double,3,1> pm_i_index, dm_index, ds_index, ps_0_index;

// ws_tf_alg_bubble_technique_n_with_rotation_rate_control variables
bool normal_mode;
unsigned int n_count;
double ang_speed_mean, rot_time;
AngleAxis<double> angAx_i;
Matrix<double,3,1> axis_i, axis_0, axis_break, axis_break_plane, proj_axis_i, aux_axis;
double ang_i, ang_im1, vAng_i, ang_0, axis_norm_err;
Quaternion<double> quats_break;
std::ofstream WTdataRecord;
double ang_break, ang_aux, diffAng;
Matrix3d R_offset(3,3), R_incRot(3,3);

double smootherstep(const double t){	return (6*t*t*t*t*t - 15*t*t*t*t + 10*t*t*t);	};	// f(t) = 6*t^5 - 15*t^4 + 10*t^3

timeval past_time_, current_time_;
double time_increment_;

unsigned int algorithm_type;
// 0 - Scaling
// 1 - Ballistic tracking
// 2 - Rate Control

double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;

bool omni_white_button_pressed, omni_grey_button_pressed;
bool first_time_done;

ros::Publisher pub_OmniForceFeedback;
geometry_msgs::Vector3 force;

double euler_angle_ZYX_Z_a_old, euler_angle_ZYX_Y_a_old, euler_angle_ZYX_X_a_old;
double indexing_angle_old;
Matrix3d R_IndexingOffset;
bool init_indexing_offset;



// Alexander & Jan
Eigen::Transform<double,3,Affine>	T_HIPrH, T_VHIPrVH, T_VHrW, T_TCPrW, T_TCPrVHIP, T_CrW, T_TCPrW_detach, T_VHIPrVH_attach, T_CrVH;
bool just_attached = true;


// Jo
Eigen::Transform<double,3,Affine>	T_W_TCP, T_W_VH, T_VH_TCP, T_W_C, T_VH_C;



// Subscriber functions
void PoseMasterWSCallback(const geometry_msgs::PoseStamped& msg){
  
  pm_i << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  quatm_i.x() = msg.pose.orientation.x;
  quatm_i.y() = msg.pose.orientation.y;
  quatm_i.z() = msg.pose.orientation.z;
  quatm_i.w() = msg.pose.orientation.w;
  Rm_i = quatm_i.toRotationMatrix();
  
//   std::cout << "quatm_i:  " << quatm_i.x() << " " << quatm_i.y() << " " << quatm_i.z() << " " << quatm_i.w() << " " << std::endl;  
  
  mi_init_changed = false;
  if (!mi_init){
    pm_0 = pm_i;
    quatm_0.x() = msg.pose.orientation.x;
    quatm_0.y() = msg.pose.orientation.y;
    quatm_0.z() = msg.pose.orientation.z;
    quatm_0.w() = msg.pose.orientation.w;    
    Rm_0 = quatm_0.toRotationMatrix();
    
    mi_init = true;
    mi_init_changed = true;
    ROS_INFO("First Master Pose received");
  }
  
//   std::cout << "master pose: " << pm_i.transpose() << std::endl;
}

void BaseMasterWSCallback(const geometry_msgs::PoseStamped& msg){
  
  pm << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  quatm.x() = msg.pose.orientation.x;
  quatm.y() = msg.pose.orientation.y;
  quatm.z() = msg.pose.orientation.z;
  quatm.w() = msg.pose.orientation.w;
  Rm = quatm.toRotationMatrix();

  if (!m_init){
    m_init = true;
    ROS_INFO("Master Reference Frame initialized");
  }
}

void BaseSlaveWSCallback(const geometry_msgs::PoseStamped& msg){

  ps << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  quats.x() = msg.pose.orientation.x;
  quats.y() = msg.pose.orientation.y;
  quats.z() = msg.pose.orientation.z;
  quats.w() = msg.pose.orientation.w;  
  Rs = quats.toRotationMatrix();
  
  if (!s_init){
    if (!s0_init){
      ps_0 = ps;
      Rs_0 = Rs;      
    }
    s_init = true;
    ROS_INFO("Slave Reference Frame initialized");
  }    
}

void OriginSlaveWSCallback(const geometry_msgs::PoseStamped& msg){

  if (!s0_init){  
  
    ps_0 << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    quats_0.x() = msg.pose.orientation.x;
    quats_0.y() = msg.pose.orientation.y;
    quats_0.z() = msg.pose.orientation.z;
    quats_0.w() = msg.pose.orientation.w;    
    Rs_0 = quats_0.toRotationMatrix();

    s0_init = true;
    ROS_INFO("Slave Origin initialized");    
  }  
}


void scaleCallback(const geometry_msgs::Vector3& msg){

  scale[0] = msg.x;
  scale[1] = msg.y;
  scale[2] = msg.z;
  
//   ROS_INFO(" New Scale:  %.2f %.2f %.2f", scale[0], scale[1], scale[2]);  
}


void omniButtonCallback(const phantom_omni::PhantomButtonEvent& msg)
{
  omni_grey_button_pressed = (bool)msg.grey_button;
  omni_white_button_pressed = (bool)msg.white_button;
  
//   std::cout << "buttons white/grey:   " << omni_white_button_pressed << " / " << omni_grey_button_pressed << std::endl;
}


void HapticAnglesCallback(const sensor_msgs::JointState& msg)
{
  for (unsigned int i=0; i< N_HAPTIC_JOINTS; i++)	haptic_angles[i] = msg.position[i];  
  
  first_haptic_angle_read = true;  
}


bool algorithmSrvCallback(workspace_transformation::setAlgorithm::Request &req, 
			  workspace_transformation::setAlgorithm::Response &res){
  algorithm_type = req.algorithm_number;
  
  if ( (algorithm_type == 0) or 
       (algorithm_type == 1) or
       (algorithm_type == 2) )
    res.algorithm_number_ok = true;
  else
    res.algorithm_number_ok = true;
  
  if (!algorithm_set)	algorithm_set = true;
  
  ROS_INFO(" Workspace transformation algorithm set to id %d", algorithm_type);
  
  return true;
}

// Workspace mapping techniques
void ws_tf_alg_scaling_simple(void);
void ws_tf_alg_ballistic_tracking(void);
void ws_tf_alg_rate_control(void);
void ws_tf_alg_bubble_technique(void);
void ws_tf_alg_scaling_n_rate_control_with_button_switch(void);
void ws_tf_alg_rate_control_constant_velocity(void);
void ws_tf_alg_scaling_control_with_rate_control_constant_velocity(void);
void ws_tf_alg_workspace_drift_control(void);
void ws_tf_alg_bubble_technique_n_with_rotation_rate_control(void);
void ws_tf_alg_scaling_control_with_rate_control_constant_velocity_with_indexing(void);
void ws_tf_alg_bubble_technique_with_indexing(void);
void ws_tf_alg_posrot_bubble_technique_with_indexing(void);
void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing(void);
void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v2(void);
void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v3(void);
void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v4(void);
// Force Feedback
void force_feedback_pos_rate_control(void);


int main(int argc, char** argv){
  ros::init(argc, argv, "workspace_transformation");

  ros::NodeHandle node;
  ros::Rate loop_rate(100);


  // Topics
  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("poseSlaveWorkspace", 10);
  ros::Publisher pub_poseSlaveWSOrigin = node.advertise<geometry_msgs::PoseStamped>("poseSlaveWorkspaceOrigin", 10);  
  pub_OmniForceFeedback = node.advertise<geometry_msgs::Vector3>("set_forces", 10);

  ros::Subscriber sub_PoseMasterWS = node.subscribe("poseMasterWorkspace", 10, &PoseMasterWSCallback);
  ros::Subscriber sub_BaseMasterWS = node.subscribe("baseMasterWorkspace", 10, &BaseMasterWSCallback);
  ros::Subscriber sub_BaseSlaveWS = node.subscribe("baseSlaveWorkspace", 10, &BaseSlaveWSCallback);
  ros::Subscriber sub_OriginSlaveWS = node.subscribe("originSlaveWorkspace", 10, &OriginSlaveWSCallback); 
  ros::Subscriber sub_scale = node.subscribe("scale", 10, &scaleCallback);
  ros::Subscriber subOmniButtons = node.subscribe("button", 1000, omniButtonCallback);
  
  ros::Subscriber sub_HapticAngles = node.subscribe("angles", 10, &HapticAnglesCallback);
  
//   ros::Publisher pub_set_camera_pose = node.advertise<geometry_msgs::PoseStamped>("Viewer/Set_ActiveCamera_Pose", 1);
  ros::Publisher pub_set_camera_pose = node.advertise<geometry_msgs::PoseStamped>("Set_ActiveCamera_Pose", 1);  
  
  ros::ServiceServer service_server_algorithm = node.advertiseService("set_algorithm", algorithmSrvCallback);

  
  // INITIALIZATION ------------------------------------------------------------------------
  dm << 0,0,0;
  ds << 0,0,0;
  pm_im1 << 0,0,0;
  ps_im1 << 0,0,0;
  vm_i << 0,0,0;
  
  m_init = false;
  s_init = false;
  mi_init = false;
  s0_init = false;
  algorithm_set = false;
  
  ps_0 << 0.0, 0.0, 0.0;
  quats_0.x() = 0.0;
  quats_0.y() = 0.0;
  quats_0.z() = 0.0;
  quats_0.w() = 1.0;
  Rs_0 = quats_0.toRotationMatrix();  
  
  first_haptic_angle_read = false;
  
  // ws_tf_alg_bubble_technique_n_with_rotation_rate_control variables
  normal_mode = true;
  n_count = 0;
  ang_speed_mean = 0.0;
  rot_time = 0.0;
  ang_break = 0.0;
  R_offset = Matrix3d::Identity();
  WTdataRecord.open("/home/users/josep.a.claret/data/WTdataRecord.txt", std::ofstream::trunc);
  
  for (unsigned int i=0; i<3; i++)	scale[i] = 1.0;

  geometry_msgs::PoseStamped outputPose;  
  outputPose.pose.position.x = 0;	outputPose.pose.position.y = 0;		outputPose.pose.position.z = 0;
  outputPose.pose.orientation.x = 0;	outputPose.pose.orientation.y = 0;	outputPose.pose.orientation.z = 0;	outputPose.pose.orientation.w = 1;
  
  algorithm_type = 0;
  
  time_increment_ = 0;
  gettimeofday(&current_time_, NULL);
  
  omni_white_button_pressed = false;
  omni_grey_button_pressed = false;
  
  first_time_done = false;
  
  Xmin = -5.0;
  Ymin = -5.0;
  Zmin = 0.0;
  Xmax = 5.0;
  Ymax = 5.0;
  Zmax = 2.0;
  

  init_indexing_offset = false;
  R_IndexingOffset = Eigen::Matrix3d::Identity();
  
  
  geometry_msgs::PoseStamped cameraPose;  
  cameraPose.pose.position.x = 0;	cameraPose.pose.position.y = 0;		cameraPose.pose.position.z = 0;
  cameraPose.pose.orientation.x = 0;	cameraPose.pose.orientation.y = 0;	cameraPose.pose.orientation.z = 0;	cameraPose.pose.orientation.w = 1;
  
  origin_cam_dist = 7.0;
  
  
  
  // UPDATE -------------------------------------------------------------------------------
  while (ros::ok())
  {
    if (m_init && s_init && mi_init && algorithm_set)
    {
      past_time_ = current_time_;
      gettimeofday(&current_time_, NULL);
      time_increment_ = ( (current_time_.tv_sec*1e6 + current_time_.tv_usec) - (past_time_.tv_sec*1e6 + past_time_.tv_usec) ) / 1e6;      
      
      vm_i = (pm_i - pm_im1)/time_increment_; 
      
      if (algorithm_type == 0)		ws_tf_alg_scaling_simple();
      else if (algorithm_type == 1)	ws_tf_alg_ballistic_tracking();
      else if (algorithm_type == 2)	ws_tf_alg_rate_control();
      else if (algorithm_type == 3)	ws_tf_alg_bubble_technique();
      else if (algorithm_type == 4)	ws_tf_alg_scaling_n_rate_control_with_button_switch();
      else if (algorithm_type == 5)	ws_tf_alg_rate_control_constant_velocity();
      else if (algorithm_type == 6)	ws_tf_alg_scaling_control_with_rate_control_constant_velocity();
      else if (algorithm_type == 7)	ws_tf_alg_workspace_drift_control();
      else if (algorithm_type == 8)	ws_tf_alg_bubble_technique_n_with_rotation_rate_control();
      else if (algorithm_type == 9)	ws_tf_alg_scaling_control_with_rate_control_constant_velocity_with_indexing();
      else if (algorithm_type == 10)	ws_tf_alg_bubble_technique_with_indexing();
      else if (algorithm_type == 11)	ws_tf_alg_posrot_bubble_technique_with_indexing();
      else if (algorithm_type == 12)	ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing();
      else if (algorithm_type == 13)	ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v2();
      else if (algorithm_type == 14)	ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v3();
      else if (algorithm_type == 15)	ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v4();
      else				ws_tf_alg_scaling_simple();
      
      
// 	std::cout	<< "-- VISUALIZATION DATA ---------------------------------------------------" << std::endl;
//	std::cout	<< "alg: " << algorithm_type 
// 	std::cout	<< "    time inc: " << time_increment_*1000 << std::endl;
// 	std::cout	<< "    pm_0:     " << 1000*pm_0.transpose()<< std::endl;
// 	std::cout	<< "    pm_im1:   " << 1000*pm_im1.transpose()<< std::endl;
// 	std::cout	<< "    pm_i:     " << 1000*pm_i.transpose()<< std::endl;
// 	std::cout	<< "    ps_0:     " << 1000*ps_0.transpose()<< std::endl;
// 	std::cout	<< "    ps_im1:   " << 1000*ps_im1.transpose()<< std::endl;
// 	std::cout	<< "    ps_i:     " << 1000*ps_i.transpose()<< std::endl;
// 	std::cout	<< "    dm:       " << 1000*dm.transpose()<< std::endl;
// 	std::cout	<< "    ds:       " << 1000*ds.transpose()<< std::endl;
// 	std::cout	<< "    vm_i:     " << 1000*vm_i.transpose()<< std::endl;
      
//       std::cout << "Rm_0" << std::endl;
//       std::cout << Rm_0 << std::endl;
//       std::cout << "Rm" << std::endl;
//       std::cout << Rm << std::endl;
//       std::cout << "Rm * ds" << std::endl;
//       std::cout << Rm * ds << std::endl;      
//       std::cout << "Rs.transpose() * Rm * ds" << std::endl;
//       std::cout << Rs.transpose() * Rm * ds << std::endl;        
//       
//       std::cout << "Rm_i" << std::endl;
//       std::cout << Rm_i << std::endl; 
//       std::cout << "Rs_i" << std::endl;
//       std::cout << Rs_i << std::endl;
//       std::cout << "quat Rs_i" << std::endl;
//       std::cout << quats_i.x() << " " << quats_i.y() << " " << quats_i.z() << " " << quats_i.w() << std::endl; 


      pm_im1 = pm_i;
      ps_im1 = ps_i;

     
      // Send data ***********************************************
      //  Slave pose
      outputPose.header.stamp = ros::Time::now();
      outputPose.pose.position.x = ps_i(0,0);
      outputPose.pose.position.y = ps_i(1,0);
      outputPose.pose.position.z = ps_i(2,0);
      outputPose.pose.orientation.x = quats_i.x();
      outputPose.pose.orientation.y = quats_i.y();
      outputPose.pose.orientation.z = quats_i.z();
      outputPose.pose.orientation.w = quats_i.w();
      pub.publish(outputPose);
      
      //  Slave origin pose
      outputPose.header.stamp = ros::Time::now();
      outputPose.pose.position.x = ps_0(0,0);
      outputPose.pose.position.y = ps_0(1,0);
      outputPose.pose.position.z = ps_0(2,0);
      outputPose.pose.orientation.x = quats_0.x();
      outputPose.pose.orientation.y = quats_0.y();
      outputPose.pose.orientation.z = quats_0.z();
      outputPose.pose.orientation.w = quats_0.w();      
      pub_poseSlaveWSOrigin.publish(outputPose);
      
      if (algorithm_type == 13){
	    // Camera pose
	    cameraPose.header.stamp = ros::Time::now();
	    cameraPose.pose.position.x = T_camPose_S.translation()(0);
	    cameraPose.pose.position.y = T_camPose_S.translation()(1);
	    cameraPose.pose.position.z = T_camPose_S.translation()(2);
	    cameraPose.pose.orientation.x = Quaternion<double>(T_camPose_S.linear()).x();
	    cameraPose.pose.orientation.y = Quaternion<double>(T_camPose_S.linear()).y();
	    cameraPose.pose.orientation.z = Quaternion<double>(T_camPose_S.linear()).z();
	    cameraPose.pose.orientation.w = Quaternion<double>(T_camPose_S.linear()).w();
	    pub_set_camera_pose.publish(cameraPose);
	}
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  
  WTdataRecord.close();
  
  return 0;
}


void ws_tf_alg_scaling_simple(void){
  // ------------------------------------------------------------------------------------------------------------------------
  //	ps_i = ps_0 + Km * Rs^T * Rm * ( pm_i - pm_0 )
  
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];

  //  Position
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;
  
  ps_i = ps_0 + 10 * ds;

  //  Rotation
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i;
}


void ws_tf_alg_ballistic_tracking(void){
  // ------------------------------------------------------------------------------------------------------------------------
  //	ps_i = ps_{i-1} + Rs^T * Rm * K * |vm_i| * ( pm_i - pm_{i-1} )
    
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];
  
  vm_i = (pm_i - pm_im1)/time_increment_;
  
  // Displacement
  double km = vm_i.norm();

  dm(2) = km*(pm_i - pm_im1)(2);	// Eix X del slave BMM  
  dm(0) = km*(pm_i - pm_im1)(0);	// Eix Y del slave BMM
  dm(1) = (pm_i - pm_0)(1);		// Eix Z del slave BMM

  //  Position
  ps_i(0) = (ps_im1 + 100*Ks * Rs.transpose() * Rm * dm)(0);
  ps_i(1) = (ps_im1 + 100*Ks * Rs.transpose() * Rm * dm)(1);  
  ps_i(2) = (ps_0 + 10 * Ks * Rs.transpose() * Rm * dm)(2);
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;

  //  Rotation
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i; 
}


void ws_tf_alg_rate_control(void){
  // ------------------------------------------------------------------------------------------------------------------------
  //	ps_i = ps_{i-1} + Km * Rs^T * Rm * ( pm_i - pm_0 )
    
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];
 
  // Displacement
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;

  //  Position
  ps_i(0) = (ps_im1 + 0.25 * ds)(0);
  ps_i(1) = (ps_im1 + 0.25 * ds)(1);  
  ps_i(2) = (ps_0 + 10 * ds)(2); 
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;

  //  Rotation
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i; 
}


void ws_tf_alg_bubble_technique(void){
  ws_tf_alg_scaling_control_with_rate_control_constant_velocity();
  
  force_feedback_pos_rate_control();
}


void ws_tf_alg_scaling_n_rate_control_with_button_switch(void){
  
  // Mohamed Mamdouth et al.
  // Evaluation of a Proposed Workspace Spanning Technique for Small Haptic Device Based Manipulator Teleoperation
  // ICIRA 2012
  if (omni_white_button_pressed){
    if (!first_time_done){
      p_aux = pm_i;
      first_time_done = true;
    }
    
    Ks = MatrixXd::Zero(3,3);
    Ks(0,0) = scale[0];
    Ks(1,1) = scale[1];
    Ks(2,2) = scale[2];

    dm = pm_i - p_aux;
    ds = Ks * Rs.transpose() * Rm * dm;    
    
    double v = 0.02;
    double err = 0.0001;    
    double D = sqrt( ds(0)*ds(0) + ds(1)*ds(1) );
    
    // Workspace origin
    if (D > err){
      ps_0(0) += v * (ds(0) / D);
      ps_0(1) += v * (ds(1) / D);
    }
    
    if (ps_0(0) < Xmin)		ps_0(0) = Xmin;
    if (ps_0(0) > Xmax)		ps_0(0) = Xmax;
    if (ps_0(1) < Ymin)		ps_0(1) = Ymin; 
    if (ps_0(1) > Ymax)		ps_0(1) = Ymax;    
  }
  else{
    first_time_done = false;
  }
  
  
  //  Position  
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];
  
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;
  
  ps_i = ps_0 + 10 * ds;
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;
  
  //  Rotation
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i;  
}


void ws_tf_alg_rate_control_constant_velocity(void){
  // ------------------------------------------------------------------------------------------------------------------------
  //	ps_i = ps_{i-1} + v * (Km * Rs_T * Rm * ( pm_i - pm_0 ) ) / |Km * Rs_T * Rm * ( pm_i - pm_0 )|
   
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];
 
  // Displacement
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;

  double v = 0.02;	// cm/s
  double err = 0.05;
  
  double D = sqrt( ds(0)*ds(0) + ds(1)*ds(1) );
  
  //  Position
  if (D > err){
    ps_i(0) = ps_im1(0) + v * (ds(0) / D);
    ps_i(1) = ps_im1(1) + v * (ds(1) / D);
  }
  else{
    ps_i(0) = ps_im1(0);
    ps_i(1) = ps_im1(1);  
  }
  ps_i(2) = ps_0(2)   +                10 * ds(2); 
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;

  //  Rotation
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i; 
}


void ws_tf_alg_scaling_control_with_rate_control_constant_velocity(void){

  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = 1.0;
  Ks(1,1) = 1.0;
  Ks(2,2) = 1.0;
 
  // Displacement
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;

  double v = 0.02;	// cm/s
  double R = 0.05;
  
  double D = sqrt( ds(0)*ds(0) + ds(1)*ds(1) );
  
  //  Position
  if (D > R){
    if (!first_time_done){
      p_aux = pm_i;
      first_time_done = true;
    }
    
    dm = pm_i - p_aux;
    ds = Ks * Rs.transpose() * Rm * dm;    
    
    // Workspace origin
    if (D > R){
      ps_0(0) += v * (ds(0) / D);
      ps_0(1) += v * (ds(1) / D);
    }
    quats_0 = Rs_0;
    
    if (ps_0(0) < Xmin)		ps_0(0) = Xmin;
    if (ps_0(0) > Xmax)		ps_0(0) = Xmax;
    if (ps_0(1) < Ymin)		ps_0(1) = Ymin; 
    if (ps_0(1) > Ymax)		ps_0(1) = Ymax;
  }
  else{
    first_time_done = false;
  }
 
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;
  
  ps_i = ps_0 + 10 * ds;
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;
  
  //  Rotation
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i;  
  
  
  
//   std::cout << " VISUALIZATION " << std::endl;
//   std::cout << "  pm_0:     " << pm_0.transpose() << std::endl; 
//   std::cout << "  pm_i:     " << pm_i.transpose() << std::endl;
//   std::cout << "  ps_0:     " << ps_0.transpose() << std::endl;
//   std::cout << "  ps_i:     " << ps_i.transpose() << std::endl;
}


void ws_tf_alg_workspace_drift_control(void){

  // Force feedback
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];

  Eigen::Vector3d omni_force;
  
  double Fmax = 3.3;	// 3.3 N es la máxima força nominal
  double Kf = 20;
    
//   omni_force(0) = - Kf * (D - R) * (ds(0)/D);
//   omni_force(1) = - Kf * (D - R) * (ds(1)/D);
//   omni_force(2) = 0.0;
    
  omni_force = Rm.transpose() * Rs * omni_force;

//     std::cout << "forces ms:  " << omni_force.transpose() << std::endl;

  if (omni_force(0) < -Fmax)		omni_force(0) = -Fmax;
  if (omni_force(0) > Fmax)		omni_force(0) = Fmax;
  if (omni_force(1) < -Fmax)		omni_force(1) = -Fmax;
  if (omni_force(1) > Fmax)		omni_force(1) = Fmax;
  
  force.x = omni_force(0);
  force.y = omni_force(1);
  force.z = omni_force(2);
  pub_OmniForceFeedback.publish(force);
}


void ws_tf_alg_bubble_technique_n_with_rotation_rate_control(void){
  
  // Position workspace mapping is as in the bubble technique 
  ws_tf_alg_bubble_technique();
  
  
  // ROTATION RATE CONTROL
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i;


  //  Data
  double min_ang_speed = (3.0)*PI; // rad/s
  double axis_norm_max_error = 5.5;
  double max_time = 0.05;	// seconds

  //  Save past values
  ang_im1 = ang_i;
  
  angAx_i = quats_i;    
  ang_i = angAx_i.angle();
  axis_i = angAx_i.axis();
  
  if (normal_mode){	// NORMAL MODE
    //  Compute angle speed and axis error
    //   If first iteration
    if (mi_init_changed){
      ang_im1 = ang_i;
      axis_0 = axis_i;
      vAng_i = 0.0;
    }
    else
    {
      vAng_i = (ang_i - ang_im1)/time_increment_;
    }

    //  Compute angle velocity mean and axis error
    n_count++;
    ang_speed_mean = ( ((double)n_count-1.0)*ang_speed_mean + vAng_i )/(double)n_count;
/*    axis_norm_err = sqrt( (axis_i(0)-axis_0(0))*(axis_i(0)-axis_0(0)) + 
			  (axis_i(1)-axis_0(1))*(axis_i(1)-axis_0(1)) + 
			  (axis_i(2)-axis_0(2))*(axis_i(2)-axis_0(2)) );*/  
    
    //  Check limits
//     if ( (fabs(ang_speed_mean) > min_ang_speed) && (axis_norm_err < axis_norm_max_error) ){
    if ( fabs(ang_speed_mean) > min_ang_speed ) {      
      
      rot_time += time_increment_;
      if ( rot_time > max_time ){    // Go to OFFSET MODE
	normal_mode = false;
	axis_break_plane = axis_i;
	axis_break = quats_i.toRotationMatrix().col(0);
	if (ang_speed_mean < 0.0){
	  ang_speed_mean *= -1;
	  for (unsigned int i=0; i<3; i++)	axis_break(i) *= -1;
	}
	quats_break = quats_i;
      }
    }
    else{	// Reset values and start over again
      normal_mode = true;
      n_count = 0;
      rot_time = 0.0;
      axis_0 = axis_i;      
    }
  }
  else {	// OFFSET MODE
//     // TEST TEST TEST TEST
//     normal_mode = true;
//     n_count = 0;
//     rot_time = 0.0;
//     // Reset origin values
//     axis_0 = axis_i;
//     // TEST TEST TEST TEST

    
    // Projectem x_axis de quats_i sobre el pla amb normal axis_break
    proj_axis_i = ( axis_break_plane.cross((quats_i.toRotationMatrix().col(0))) ).cross(axis_break_plane);
    
    // Calculem l'angle entre proj_axis_i i axis_break
    ang_break = fabs( acos(axis_break.dot(proj_axis_i)/(axis_break.norm()*proj_axis_i.norm())) );
    aux_axis = axis_break_plane.cross(axis_break);
    ang_aux = fabs( acos(aux_axis.dot(proj_axis_i)/(aux_axis.norm()*proj_axis_i.norm())) );
    if (ang_aux > PI/2	)	ang_break *= -1;

    //  Compute differential rotation angle
    double K_rot = 1.0;
    double max_ang_vel = PI/2;
    diffAng = K_rot * max_ang_vel * smootherstep(ang_break/(PI/2)) * time_increment_;
    
    //  Differential rotation to be added
//     R_incRot(0,0) = R_incRot(1,1) = R_incRot(2,2) = 1.0;
//     R_incRot(0,1) = -axis_break(2)*diffAng;
//     R_incRot(1,0) =  axis_break(2)*diffAng;
//     R_incRot(0,2) =  axis_break(1)*diffAng;
//     R_incRot(2,0) = -axis_break(1)*diffAng;
//     R_incRot(1,2) = -axis_break(0)*diffAng;
//     R_incRot(2,1) =  axis_break(0)*diffAng;
    
    double sA = sin(diffAng);
    double cA = cos(diffAng);    
    R_incRot(0,0) = cA + axis_break(0)*axis_break(0)*(1-cA);
    R_incRot(1,1) = cA + axis_break(1)*axis_break(1)*(1-cA);
    R_incRot(2,2) = cA + axis_break(2)*axis_break(2)*(1-cA);
    R_incRot(0,1) = axis_break(0)*axis_break(1)*(1-cA) - axis_break(2)*sA;
    R_incRot(1,0) = axis_break(0)*axis_break(1)*(1-cA) + axis_break(2)*sA;
    R_incRot(0,2) = axis_break(0)*axis_break(2)*(1-cA) + axis_break(1)*sA;
    R_incRot(2,0) = axis_break(0)*axis_break(2)*(1-cA) - axis_break(1)*sA;
    R_incRot(1,2) = axis_break(1)*axis_break(2)*(1-cA) - axis_break(0)*sA;
    R_incRot(2,1) = axis_break(1)*axis_break(2)*(1-cA) + axis_break(0)*sA;     
    
    //  Add differential rotation
    R_offset = R_offset*R_incRot;
    
    
    if (ang_break < 0){
      // Reset values
      normal_mode = true;
      n_count = 0;
      rot_time = 0.0;
      axis_0 = axis_i;
      ang_break = 0.0;
      diffAng = 0.0;
    }
    
  }
  quats_i = quats_i.toRotationMatrix()*R_offset;
//   std::cout << "Normal Mode: " << normal_mode 
// 	    << "      angAx: " << ang_i 
// 	    << " - " << axis_i(0) << " " << axis_i(1) << " " << axis_i(2) 
// 	    << "      vAng: " << fabs(vAng_i)
// 	    << "      vAngM/AxErr: " << fabs(ang_speed_mean) << " / " << axis_norm_err 
// 	    << std::endl;
	    
  std::cout << "Normal Mode: " << normal_mode 
// 	    << "      min_ang_speed: " << min_ang_speed
	    << "      angAx: " << ang_break
	    << "      diffAng: " << diffAng
	    << "      ax norms: " << quats_i.toRotationMatrix().col(0).norm() << " " 
				  << quats_i.toRotationMatrix().col(1).norm() << " " 
				  << quats_i.toRotationMatrix().col(2).norm()
	    << std::endl;	    
    
/*  std::cout << "-----------------------------------" << std::endl;  
  std::cout << Rs_i << std::endl;  */	    
  WTdataRecord 	<< normal_mode 
		<< " " << ang_i 
		<< " " << axis_i(0) << " " << axis_i(1) << " " << axis_i(2) 
		<< " " << fabs(vAng_i)
		<< " " << fabs(ang_speed_mean) << " " << axis_norm_err 
		<< " " << n_count
		<< " " << rot_time
		<< " " << min_ang_speed
		<< std::endl;
  
}



void ws_tf_alg_scaling_control_with_rate_control_constant_velocity_with_indexing(void){
  
  // ****************************************************************************************************************
  // Inner workspace position control with rate control on the workspace outter layer
  // When omni_grey_button_pressed is TRUE then an indexing technique is applied instead of the position control
  // ****************************************************************************************************************
  

  double freq = 100;	// Should be obtained from an external variable in the ROS node
  double period = 1/freq;;
  
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];
  
  
  
  // Position -----------------------------------
  
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;  
  
  double Kp = 10;
  double Kv = 2;

  double R = 0.5;
  double D = Kp * sqrt( ds(0)*ds(0) + ds(1)*ds(1) );  
  
  
  // Indexing ************************
  double K_index = 1;
  
  if (omni_white_button_pressed && !omni_grey_button_pressed){

    if (!first_time_indexing){
      first_time_indexing = true;
      pm_i_index = pm_i;
      ps_0_index = ps_0;
    }
    
    dm_index = pm_i - pm_i_index;
    ds_index = Ks * Rs.transpose() * Rm * dm_index;
    ds_index(2) = 0.0;
    
    //  Dps_0 = K_I * Dds		<-->		ps_0_{i+1} = ps_0_{i} + K_I * ds    
    ps_0 = ps_0_index - K_index * Kp * ds_index;
    
    // Position control in axis Z
    //  Dps_0 = 0			<-->		ps_0_{i+1} = ps_0_{i}
    //  Dps_i = K_1 * Dds		<-->		ps_{i+1} = ps_0_{i} + K_1 * ds
    ps_i(2) = (ps_0 + Kp * ds)(2);
  }
  
  // No indexing *********************
  else {
    first_time_indexing = false;  
    
    // Rate control
    //  Dps_0 = K_2 * ds		<-->		ps_0_{i+1} = ps_0_{i} + K_2 * ds * u( |ds| - R ) * T
    //  Dps_i = K_1 * Dds		<-->		ps_{i+1} = ps_0_{i} + K_1 * ds  
    if (D > R){
      
      double ps_cyl_rad = sqrt( (ps_i(0)-ps_0(0))*(ps_i(0)-ps_0(0)) + (ps_i(1)-ps_0(1))*(ps_i(1)-ps_0(1)) );
      ds_aux = ( 1 - R/ps_cyl_rad ) * ( ps_i - ps_0 );
      ds_aux(2) = 0.0;
      
      // Workspace origin
      ps_0 += Kv * ds_aux * period;
    }
    
    // Position control
    //  Dps_0 = 0			<-->		ps_0_{i+1} = ps_0_{i}
    //  Dps_i = K_1 * Dds		<-->		ps_{i+1} = ps_0_{i} + K_1 * ds
    ps_i = ps_0 + Kp * ds;
  }
    
  if (ps_0(0) < Xmin)		ps_0(0) = Xmin;
  if (ps_0(0) > Xmax)		ps_0(0) = Xmax;
  if (ps_0(1) < Ymin)		ps_0(1) = Ymin; 
  if (ps_0(1) > Ymax)		ps_0(1) = Ymax; 
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax; 
  
  
  //  Rotation --------------------------------
  quats_0 = Rs_0;
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i;
}


void force_feedback_pos_rate_control(void){
  
  // ****************************************************************************************************************
  // Force feedback when in rate control area
  // ****************************************************************************************************************
  
  
  // Variables
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = 1.0;
  Ks(1,1) = 1.0;
  Ks(2,2) = 1.0;
 
  Eigen::Vector3d omni_force = Eigen::Vector3d::Zero();
  double R = 0.035;
  double Fmax = 3.3;	// 3.3 N es la máxima força nominal
  double Kf = 40;
  double Kv = 0.5;

  
  // Displacement
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;  
  
  // Omni feedback force computation
  double D = sqrt( ds(0)*ds(0) + ds(1)*ds(1) );  
  
  if (D > R)	omni_force = - Kf * (D - R) * (ds/D);	
  omni_force += - Kv*vm_i;				// Damping
  omni_force(2) = 0.0;  				// No force in vertical axis
  omni_force = Rm.transpose() * Rs * omni_force;	// To master reference frame
  
  if (omni_force(0) < -Fmax)		omni_force(0) = -Fmax;
  if (omni_force(0) > Fmax)		omni_force(0) = Fmax;
  if (omni_force(1) < -Fmax)		omni_force(1) = -Fmax;
  if (omni_force(1) > Fmax)		omni_force(1) = Fmax;  


  // Publish
  force.x = omni_force(0);
  force.y = omni_force(1);
  force.z = omni_force(2);
  pub_OmniForceFeedback.publish(force);
}

  
void ws_tf_alg_bubble_technique_with_indexing(void){
  
  ws_tf_alg_scaling_control_with_rate_control_constant_velocity_with_indexing();
  
  force_feedback_pos_rate_control();
}


void ws_tf_alg_posrot_bubble_technique_with_indexing(void){
  
  ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing();
  
  force_feedback_pos_rate_control();
}


void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing(void){
  
  // ****************************************************************************************************************
  // Inner workspace position control with rate control on the workspace outter layer
  // When omni_grey_button_pressed is TRUE then an indexing technique is applied instead of the position control
  // ****************************************************************************************************************
  

  double freq = 100;	// Should be obtained from an external variable in the ROS node
  double period = 1/freq;;
  
  
 //  Rotation -----------------------------------------------------------------------------------------------------------------------------
  
  //    Workspace origin
  //      Compute angle between plane XZ and the projection of haptic tip onto the plane XY in the master reference frame
  //        From reference Haptic to reference Master
  
  Matrix3d Rm_nonref( Rm*Rm_i*Rm.transpose() );							// Rm_i in generic haptic reference frame - change of base 
  
  Matrix<double,3,1> x_rm( Rm_nonref.col(0).normalized() );					// X Axis
  Matrix<double,3,1> x_rm_xy = Matrix<double,3,1>(x_rm(0), x_rm(1), 0).normalized();		// Projection onto plane XY
  double angle_x_xy = asin(x_rm_xy(1));
  if (x_rm_xy(1) >= 0){
    if (x_rm_xy(0) < 0)		angle_x_xy =   PI - angle_x_xy;
  }
  else{
    if (x_rm_xy(0) < 0)		angle_x_xy = - PI - angle_x_xy;
  }  
 
  double angle_x_xz = -asin(x_rm(2));
  
  double angle_y_xy = - haptic_angles[5] - PI;

  //      Rotation rate control
  const double maxAng_width = 45*(PI/180);
  const double maxAng_heigth = 45*(PI/180);
  const double vRot = 72 *(PI/180);
  if ( (fabs(angle_x_xy) > maxAng_width) && (fabs(angle_x_xz) < maxAng_heigth) ){
    Matrix3d rotRate_mat = AngleAxis<double>((angle_x_xy/fabs(angle_x_xy))*vRot*time_increment_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Rs_0 = rotRate_mat*Rs_0;
  }
    
  
  //    Slave tip
  Rs_i = Rs.transpose() * Rm * Rm_i * Rm_0.transpose() * Rm.transpose() * Rs * Rs_0;
  quats_i = Rs_i;
  
//   std::cout << " VISUALIZATION " << std::endl;
//   std::cout << "Rm_0" << std::endl;
//   std::cout << Rm_0 << std::endl;  
//   std::cout << "  Rm:       " << std::endl << Rm << std::endl;  
//   std::cout << "  Rm_i:     " << std::endl << Rm_i << std::endl;   
//   std::cout << "  Rm*Rm_i*Rm^T:  " << std::endl << Rm * Rm_i * Rm.transpose() << std::endl;
//   std::cout << "  Rs_0:     " << std::endl << Rs_0 << std::endl; 
//   std::cout << "  x_rm:  " << x_rm.transpose()<< std::endl;
//   std::cout << "  y_rm:  " << y_rm.transpose()<< std::endl;
//   std::cout << "  x_rm_xy:  " << x_rm_xy.transpose()<< std::endl;
//   std::cout << "  ang_xy_raw:   " << asin(x_rm_xy(1))*(180/PI) << std::endl;
//   std::cout << "  y_rm:       " << y_rm.transpose()<< std::endl;
//   std::cout << "  y_rm_xy:    " << y_rm_xy.transpose()<< std::endl;
//   std::cout << "  ang_x_xy:   " << angle_x_xy*(180/PI) << std::endl;
//   std::cout << "  ang_x_xz:   " << angle_x_xz*(180/PI) << std::endl;
//   std::cout << "  ang_y_xy:   " << angle_y_xy*(180/PI) << std::endl;  
  
  

  // Position ----------------------------------------------------------------------------------------------------------------------------
  
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = 1.0;
  Ks(1,1) = 1.0;
  Ks(2,2) = 1.0;
  
  Matrix3d K_s_m(Matrix3d::Identity());
  K_s_m(0,0) = scale[0];
  K_s_m(1,1) = scale[1];
  K_s_m(2,2) = scale[2];
  
  dm = pm_i - pm_0;
  ds = K_s_m * Ks * Rs.transpose() * Rm * dm;
//   ds = Ks * Rs_0 * Rs.transpose() * Rm * dm;  
  
  double Kp = 10;
  double Kv = 2;

  double R = 0.5;
  double D = Kp * sqrt( ds(0)*ds(0) + ds(1)*ds(1) );  
  
  
  // Indexing ************************
  double K_index = 1;
  
  if (omni_white_button_pressed && !omni_grey_button_pressed){

    if (!first_time_indexing){
      first_time_indexing = true;
      pm_i_index = pm_i;
      ps_0_index = ps_0;
    }
    
    dm_index = pm_i - pm_i_index;
    ds_index = Ks * Rs.transpose() * Rm * dm_index;
    ds_index(2) = 0.0;
    
    //  Dps_0 = K_I * Dds		<-->		ps_0_{i+1} = ps_0_{i} + K_I * ds    
    ps_0 = ps_0_index - K_index * Kp * ds_index;
    
    // Position control in axis Z
    //  Dps_0 = 0			<-->		ps_0_{i+1} = ps_0_{i}
    //  Dps_i = K_1 * Dds		<-->		ps_{i+1} = ps_0_{i} + K_1 * ds
    ps_i(2) = (ps_0 + Kp * ds)(2);
  }
  
  // No indexing *********************
  else {
    first_time_indexing = false;  
    
    // Rate control
    //  Dps_0 = K_2 * ds		<-->		ps_0_{i+1} = ps_0_{i} + K_2 * ds * u( |ds| - R ) * T
    //  Dps_i = K_1 * Dds		<-->		ps_{i+1} = ps_0_{i} + K_1 * ds  
    if (D > R){
      
      double ps_cyl_rad = sqrt( (ps_i(0)-ps_0(0))*(ps_i(0)-ps_0(0)) + (ps_i(1)-ps_0(1))*(ps_i(1)-ps_0(1)) );
      ds_aux = ( 1 - R/ps_cyl_rad ) * ( ps_i - ps_0 );
      ds_aux(2) = 0.0;
      
      // Workspace origin
      ps_0 += Kv * ds_aux * period;
//       ps_0 += Kv * Rs_0 * ds_aux * period;
    }
    
    // Haptic tip
    //  Dps_0 = 0			<-->		ps_0_{i+1} = ps_0_{i}
    //  Dps_i = K_1 * Dds		<-->		ps_{i+1} = ps_0_{i} + K_1 * ds
    ps_i = ps_0 + Kp * ds;
//     ps_i = ps_0 + Kp * Rs_0 * ds;    
  }
    
  if (ps_0(0) < Xmin)		ps_0(0) = Xmin;
  if (ps_0(0) > Xmax)		ps_0(0) = Xmax;
  if (ps_0(1) < Ymin)		ps_0(1) = Ymin; 
  if (ps_0(1) > Ymax)		ps_0(1) = Ymax; 
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax; 
}



void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v2(void){
  
  // ****************************************************************************************************************
  // Inner workspace position control with rate control on the workspace outter layer
  // When omni_grey_button_pressed is TRUE then an indexing technique is applied instead of the position control
  // ****************************************************************************************************************
  
  
  // MASTER reference frame --------------------------------------------------------------
  Eigen::Transform<double,3,Affine>	T_im_m_M  =	Translation3d(pm_i)	*	AngleAxisd(quatm_i);
  Eigen::Transform<double,3,Affine>	T_0m_m_M  =	Translation3d(pm_0)	*	AngleAxisd(quatm_0);
  Eigen::Transform<double,3,Affine>	T_m_W 	  =	Translation3d(pm)	*	AngleAxisd(quatm);
  Eigen::Transform<double,3,Affine>	T_im_0m_W =	T_m_W 			*	T_0m_m_M.inverse()	*	T_im_m_M	*	T_m_W.inverse();  


  // Scaling from Master Environment to Slave Environment --------------------------------
  Matrix3d K_s_m(Matrix3d::Identity());
  K_s_m(0,0) = scale[0];
  K_s_m(1,1) = scale[1];
  K_s_m(2,2) = scale[2];

  Eigen::Transform<double,3,Affine> 	T_is_0s_W =	Translation3d( K_s_m * T_im_0m_W.translation() ) 	* 	AngleAxisd( Quaterniond( T_im_0m_W.linear() ) );


  // SLAVE reference frame --------------------------------------------------------------
  Eigen::Transform<double,3,Affine>	T_s_W 	  =	Translation3d(ps)	*	AngleAxisd(quats);
  Eigen::Transform<double,3,Affine>	T_0s_s_S  =	Translation3d(ps_0)	*	AngleAxisd(quats_0);
  Eigen::Transform<double,3,Affine>	T_is_0s_S =	T_s_W.inverse()		*	T_is_0s_W	*	T_s_W;
  Eigen::Transform<double,3,Affine>	T_is_s_S;
//   Eigen::Transform<double,3,Affine>	T_is_s_S  =	T_0s_s_S		*	T_is_0s_S;


 
  // Worskspace Mapping algorithm ------------------------------------------------------


  //  Algorithm data **********************
  
  //	Position control
  const double K_position_control_Pos		=	10.0;

  //	Indexing
  const double K_indexing_Pos			=	1.0;
  const double K_indexing_Rot			=	1.0;  
  
  //	Rate control
  const double R 				=	0.5;  
  const double K_rate_control_Pos		=	20.0;
  const double maxAng_width			=	45.0 * (PI/180);
  const double maxAng_heigth			=	45.0 * (PI/180);
  const double K_rate_control_Rot		=	18.0 * (PI/180);
  
  
  //   Algorithms setting *****************

  //     Euler Angles Z Y X convention ( matrix multiplication = R_Z R_Y R_X )
  const double euler_angle_ZYX_Z		=	atan2(   T_is_0s_S(1,0) , T_is_0s_S(0,0) );
  const double euler_angle_ZYX_Y		=	atan2( - T_is_0s_S(2,0) , sqrt( T_is_0s_S(0,0)*T_is_0s_S(0,0) + T_is_0s_S(1,0)*T_is_0s_S(1,0) ) );
  const double euler_angle_ZYX_X		=	atan2(   T_is_0s_S(2,1) , T_is_0s_S(2,2) );

  //	 Indexing
  bool indexing_on				=	omni_white_button_pressed && !omni_grey_button_pressed;
    
  //	 Rate control
  //		Rotation
  bool rate_control_rotation_on			=	false;
  if ( (fabs(euler_angle_ZYX_Z) > maxAng_width) && (fabs(euler_angle_ZYX_Y) < maxAng_heigth) )		rate_control_rotation_on = true;
  //		Translation
  bool rate_control_position_on 		=	false;
  const Vector3d T_is_s_S_Pos			=	T_is_0s_S.translation();
  const double D				=	K_position_control_Pos	*	sqrt( T_is_s_S_Pos(0) * T_is_s_S_Pos(0) + T_is_s_S_Pos(1) * T_is_s_S_Pos(1) );
  if ( D > R )												rate_control_position_on = true;
  

  //   Algorithms execution ***************
  
  //	Position control ----------------
  //		Orientation
  //		Position
  T_is_0s_S.translation()			=	K_position_control_Pos	*	T_is_0s_S.translation();
  T_is_s_S = T_0s_s_S * T_is_0s_S;

  
  //	Rate control --------------------
  //		Rotation    
  if ( rate_control_rotation_on )
  {
    const double sign_angle			=	euler_angle_ZYX_Z/fabs(euler_angle_ZYX_Z);
    Matrix3d rotRate_mat			=	AngleAxisd( sign_angle * K_rate_control_Rot * time_increment_ , Eigen::Vector3d::UnitZ() ).toRotationMatrix();
    T_is_s_S.linear()				=	rotRate_mat		*	T_is_s_S.linear();
    T_0s_s_S.linear()				=	rotRate_mat		*	T_0s_s_S.linear();
//     T_is_s_S.linear()				=	T_0s_s_S.linear()	*	T_is_0s_S.linear();
  }
  //		Translation
  if ( rate_control_position_on )
  {
    Vector3d ratePosInc = ( 1 - R/D ) * T_0s_s_S.linear() * Vector3d( T_is_s_S_Pos(0), T_is_s_S_Pos(1), 0.0 );
    T_is_s_S.translation() 			+=	K_rate_control_Pos	*	time_increment_		*	ratePosInc;
    T_0s_s_S.translation()			+=	K_rate_control_Pos	*	time_increment_		*	ratePosInc;
  }
//   T_0s_s_S					=	T_is_s_S		*	T_is_0s_S.inverse();
  

//   std::cout << "DATA ----------------------------------------------------------------------" << std::endl;
//   std::cout << "  T_div:    " << std::endl<< T_is_s_S.linear().transpose() * T_0s_s_S.linear() << std::endl;
//   std::cout << "  T_0s_s_S.affine:    " << std::endl<< T_0s_s_S.affine()<< std::endl;
//   std::cout << "  T_is_0s_S.affine: " << std::endl<< T_is_0s_S.affine()<< std::endl;
//   std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;

  
  
// // //   //    Indexing ------------------------
// // //   if (!init_indexing_offset && mi_init_changed && m_init && s_init && s0_init){
// // //     std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << std::endl;
// // //     R_IndexingOffset 				=	T_is_0s_S.linear().transpose()	*	T_is_s_S.linear();
// // //     
// // //     std::cout << "  R_IndexingOffset:  " << std::endl<< R_IndexingOffset<< std::endl;  
// // //     std::cout << "  T_0s_s_S.affine:    " << std::endl<< T_0s_s_S.affine()<< std::endl;
// // //     std::cout << "  T_is_0s_S.affine: " << std::endl<< T_is_0s_S.affine()<< std::endl;
// // //     std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;      
// // //     init_indexing_offset = true;
// // //   }
// // //   
// // //   if (indexing_on)
// // //   {
// // // //     std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
// // //     //		Rotation
// // //     R_IndexingOffset 				=	T_is_0s_S.linear().transpose()	*	T_is_s_S.linear();
// // //     T_0s_s_S.linear()				=	R_IndexingOffset;
// // //     
// // //     //		Translation
// // //   }  
// // //   else
// // //   {
// // // //   std::cout << "  ------------------------------------------------------------" << std::endl;
// // // //   std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;
// // //     //		Rotation
// // //     T_is_s_S.linear()				=	T_is_0s_S.linear()	*	R_IndexingOffset;
// // // //   std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;
// // // //   std::cout << "  T_0s_s_S.affine:  " << std::endl<< T_0s_s_S.affine()<< std::endl;  
// // //     
// // //     T_0s_s_S.linear()				=	T_is_s_S.linear()	*	T_is_0s_S.linear().transpose();
// // // //   std::cout << "  T_0s_s_S.affine:  " << std::endl<< T_0s_s_S.affine()<< std::endl;  
// // //  
// // //     //		Translation
// // // 
// // //   }
// // //   
// // //   
// // //   //	Rate control --------------------
// // //   //		Rotation    
// // //   if ( rate_control_rotation_on )
// // //   {
// // //     const double sign_angle			=	euler_angle_ZYX_Z/fabs(euler_angle_ZYX_Z);
// // //     Matrix3d rotRate_mat			=	AngleAxisd( sign_angle * K_rate_control_Rot * time_increment_ , Eigen::Vector3d::UnitZ() ).toRotationMatrix();
// // //     T_is_s_S.linear()				=	rotRate_mat		*	T_is_s_S.linear();
// // //     T_0s_s_S.linear()				=	rotRate_mat		*	T_0s_s_S.linear();
// // // //     T_is_s_S.linear()				=	T_0s_s_S.linear()	*	T_is_0s_S.linear();
// // //   }
// // //   //		Translation
// // //   if ( rate_control_position_on )
// // //   {
// // //     Vector3d ratePosInc = ( 1 - R/D ) * T_0s_s_S.linear() * Vector3d( T_is_s_S_Pos(0), T_is_s_S_Pos(1), 0.0 );
// // //     T_is_s_S.translation() 			+=	K_rate_control_Pos	*	time_increment_		*	ratePosInc;
// // //     T_0s_s_S.translation()			+=	K_rate_control_Pos	*	time_increment_		*	ratePosInc;
// // //   }
// // // //   T_0s_s_S					=	T_is_s_S		*	T_is_0s_S.inverse();
  
  
  
  
  
  
  
  
  
  
  
  
  std::cout << "DATA ------------------------------" << std::endl;
  std::cout << "  T_0s_s_S.affine:    " << std::endl<< T_0s_s_S.affine()<< std::endl;
  std::cout << "  T_is_0s_S.affine: " << std::endl<< T_is_0s_S.affine()<< std::endl;
  std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;
  
  
  
  
  
  
  
  
  
 
 
 
 
// //   // Worskspace Mapping algorithm ------------------------------------------------------
// // 
// // 
// //   //  Algorithm data **********************
// //   
// //   //	Position control
// //   const double K_position_control_Pos		=	10.0;
// // 
// //   //	Indexing
// //   const double K_indexing_Pos			=	1.0;
// //   const double K_indexing_Rot			=	1.0;  
// //   
// //   //	Rate control
// //   const double R 				=	5;  
// //   const double K_rate_control_Pos		=	20.0;
// //   const double maxAng_width			=	45.0 * (PI/180);
// //   const double maxAng_heigth			=	45.0 * (PI/180);
// //   const double K_rate_control_Rot		=	18.0 * (PI/180);
// //   
// //   
// //   //   Algorithms setting *****************
// // 
// //   //     Euler Angles Z Y X convention ( matrix multiplication = R_Z R_Y R_X )
// //   const double euler_angle_ZYX_Z		=	atan2(   T_is_0s_S(1,0) , T_is_0s_S(0,0) );
// //   const double euler_angle_ZYX_Y		=	atan2( - T_is_0s_S(2,0) , sqrt( T_is_0s_S(0,0)*T_is_0s_S(0,0) + T_is_0s_S(1,0)*T_is_0s_S(1,0) ) );
// //   const double euler_angle_ZYX_X		=	atan2(   T_is_0s_S(2,1) , T_is_0s_S(2,2) );
// //  
// //   bool rate_control_position_on			=	false;
// //   bool rate_control_rotation_on			=	false;
// //   bool indexing_on = false;
// //   
// //   
// //   //	Position control ----------------
// //   //		Orientation:
// //   
// //   //		Position:	Dps_i = K_1 * Dds		<-->		ps_{i+1} - ps_0_{i} = K_1 * ds
// //   T_is_0s_S.translation()			=	K_position_control_Pos	*	T_is_0s_S.translation();
// // 
// //   
// //   if (indexing_on){
// //     
// //     // Indexing -------------------------
// //     if (!first_time_indexing){
// //       first_time_indexing = true;
// //       T_imInitIndex_m_M = T_im_m_M;
// //       T_0sInitIndex_s_S = T_0s_s_S;
// //     }
// // 
// // // TEST TEST TEST TEST TEST TEST TEST 
// // //     Eigen::Transform<double,3,Affine> T_im_imInitIndex_m 	= 	Translation3d( T_im_m_M.translation() - T_imInitIndex_m_M.translation() ) 
// // // 										* AngleAxisd(Quaterniond(T_im_m_M.linear() * T_imInitIndex_m_M.linear().transpose()));
// // //     Eigen::Transform<double,3,Affine> T_im_imInitIndex_W   	= 	T_m_W		*	T_im_imInitIndex_m 	*	 T_m_W.inverse();
// // //     Eigen::Transform<double,3,Affine> T_is_isInitIndex_0s	=	Translation3d( K_s_m * T_im_imInitIndex_W.translation() ) 
// // // 										* AngleAxisd( Quaterniond(T_im_imInitIndex_W.linear()) );
// // //     Eigen::Transform<double,3,Affine> T_is_isInitIndex_S	=	T_0s_s_S 	*	T_is_isInitIndex_0s	*	T_0s_s_S.inverse();    
// // // TEST TEST TEST TEST TEST TEST TEST 
// //     
// //     
// //     //		Position
// //     Eigen::Vector3d	v_im_imInitIndex_m	=	T_im_m_M.translation()	-	T_imInitIndex_m_M.translation();
// //     Eigen::Vector3d	v_im_imInitIndex_W	=	T_m_W.linear()		*	v_im_imInitIndex_m;
// // 
// //     Eigen::Vector3d	v_is_isInitIndex_0s	=	K_s_m			*	v_im_imInitIndex_W;
// //     Eigen::Vector3d	v_is_isInitIndex_S	=	T_0s_s_S.linear() 	*	v_is_isInitIndex_0s;
// //     T_0s_s_S.translation()(0)			=	(T_0sInitIndex_s_S.translation() - K_indexing_Pos * K_position_control_Pos * v_is_isInitIndex_S)(0);
// //     T_0s_s_S.translation()(1)			=	(T_0sInitIndex_s_S.translation() - K_indexing_Pos * K_position_control_Pos * v_is_isInitIndex_S)(1);
// //     
// // 
// //     //		Orientation
// //     Eigen::Matrix3d	R_im_imInitIndex_m	=	T_im_m_M.linear()	*	T_imInitIndex_m_M.linear().transpose();
// //     Eigen::Matrix3d	R_im_imInitIndex_W	=	T_m_W.linear()		*	R_im_imInitIndex_m	*	T_m_W.linear().transpose();
// //     
// //     Eigen::Matrix3d	R_is_isInitIndex_0s	=	R_im_imInitIndex_W;
// //     Eigen::Matrix3d	R_is_isInitIndex_S	=	T_0s_s_S.linear()	*	R_is_isInitIndex_0s	*	T_0s_s_S.linear().transpose();
// //     
// //     
// // // TEST TEST TEST TEST TEST TEST TEST 
// // //     const double euler_angle_ZYX_Z_a		=	atan2(   R_is_isInitIndex_S(1,0) , R_is_isInitIndex_S(0,0) );
// // //     const double euler_angle_ZYX_Y_a		=	atan2( - R_is_isInitIndex_S(2,0) , 
// // // 								sqrt( R_is_isInitIndex_S(0,0)*R_is_isInitIndex_S(0,0) + R_is_isInitIndex_S(1,0)*R_is_isInitIndex_S(1,0) ) );
// // //     const double euler_angle_ZYX_X_a		=	atan2(   R_is_isInitIndex_S(2,1) , R_is_isInitIndex_S(2,2) );
// // // //     std::cout << "angles:      " << euler_angle_ZYX_Z_a*(180/PI) << "  " << euler_angle_ZYX_Y_a*(180/PI) << "  " << euler_angle_ZYX_X_a*(180/PI) << std::endl;
// // // //     std::cout << "angles:      " << euler_angle_ZYX_Z*(180/PI) << "  " << euler_angle_ZYX_Y*(180/PI) << "  " << euler_angle_ZYX_X*(180/PI) << std::endl;
// // // //     std::cout << "axis-angle:  " << AngleAxisd( R_is_isInitIndex_S ).angle()*(180/PI) << "    -   " << AngleAxisd( R_is_isInitIndex_S ).axis().transpose() << std::endl;
// // // 
// // //     
// // //     const double angIncTot = sqrt( (euler_angle_ZYX_Z_a_old-euler_angle_ZYX_Z_a)*(euler_angle_ZYX_Z_a_old-euler_angle_ZYX_Z_a) + 
// // // 				   (euler_angle_ZYX_Y_a_old-euler_angle_ZYX_Y_a)*(euler_angle_ZYX_Y_a_old-euler_angle_ZYX_Y_a) + 
// // // 				   (euler_angle_ZYX_X_a_old-euler_angle_ZYX_X_a)*(euler_angle_ZYX_X_a_old-euler_angle_ZYX_X_a) );
// // //     std::cout << "angIncTot:   " << angIncTot*(180/PI) << std::endl;
// // //         
// // //     euler_angle_ZYX_Z_a_old = euler_angle_ZYX_Z_a;
// // //     euler_angle_ZYX_Y_a_old = euler_angle_ZYX_Y_a;
// // //     euler_angle_ZYX_X_a_old = euler_angle_ZYX_X_a;
// // // TEST TEST TEST TEST TEST TEST TEST 
// //     
// //     
// //     const double indexing_angle			=	atan2( R_is_isInitIndex_S(1,0), R_is_isInitIndex_S(0,0) );
// //     Matrix3d rotRate_mat			=	AngleAxisd( - indexing_angle , Eigen::Vector3d::UnitZ() ).toRotationMatrix();
// //     T_0s_s_S.linear()				=	rotRate_mat		*	T_0sInitIndex_s_S.linear();
// // 
// // 
// //     //		Orientation v2
// //      Eigen::Matrix3d	R_offset		=	T_imInitIndex_m_M.linear().transpose()			*	T_0sInitIndex_s_S.linear();
// // // 			T_0s_s_S.linear()	=	T_im_m_M.linear()	*	R_offset;
// //     
// //     
// //      
// //      
// //      
// //      
// //      
// //      
// //      
// // //     std::cout << " VISUALIZATION ---------------------------------------------------------------------------------------" << std::endl;
// // // //     std::cout << "R_im_imInitIndex_m" << std::endl << R_im_imInitIndex_m << std::endl;   
// // //     std::cout << "R_im_imInitIndex_W" << std::endl << R_im_imInitIndex_W << std::endl;     
// // //   std::cout << "R_is_isInitIndex_0s" << std::endl << R_is_isInitIndex_0s << std::endl;
// // //   std::cout << "  " << R_is_isInitIndex_0s.col(0).norm() << "  " << R_is_isInitIndex_0s.col(1).norm() << "  " << R_is_isInitIndex_0s.col(2).norm() << std::endl;
// // //   std::cout << "  " << R_is_isInitIndex_0s.col(0).dot(R_is_isInitIndex_0s.col(1)) 
// // // 	    << "  " << R_is_isInitIndex_0s.col(0).dot(R_is_isInitIndex_0s.col(2)) 
// // // 	    << "  " << R_is_isInitIndex_0s.col(1).dot(R_is_isInitIndex_0s.col(2)) << std::endl;
// //   
// // //     std::cout << "T_0s_s_S.Rot()    " << std::endl << T_0s_s_S.linear() << std::endl;  
// // //     std::cout << "R_is_isInitIndex_S" << std::endl << R_is_isInitIndex_S << std::endl;
// // //   std::cout << "  " << R_is_isInitIndex_S.col(0).norm() << "  " << R_is_isInitIndex_S.col(1).norm() << "  " << R_is_isInitIndex_S.col(2).norm() << std::endl;
// // //   std::cout << "  " << R_is_isInitIndex_S.col(0).dot(R_is_isInitIndex_S.col(1)) 
// // // 	    << "  " << R_is_isInitIndex_S.col(0).dot(R_is_isInitIndex_S.col(2)) 
// // // 	    << "  " << R_is_isInitIndex_S.col(1).dot(R_is_isInitIndex_S.col(2)) << std::endl;
// // // 	std::cout << "angle:        " << indexing_angle * (180/PI) << std::endl;
// // // 	std::cout << "inc angle:    " << fabs(indexing_angle - indexing_angle_old)* (180.0/PI) << std::endl;
// // //     std::cout << "rotRate_mat       " << std::endl << rotRate_mat << std::endl;
// // 	
// // 	
// //     indexing_angle_old = indexing_angle;	// TEST TEST TEST TEST
// //   }
// //   else	//  Indexing OFF
// //   {
// //     first_time_indexing = false;
// //     
// //     //  Indexing
// //     //		Orientation
// //     
// //     
// //     //	Rate control
// //     
// //     //		Orientation:
// //     if ( (fabs(euler_angle_ZYX_Z) > maxAng_width) && (fabs(euler_angle_ZYX_Y) < maxAng_heigth) )	rate_control_rotation_on = true;
// //     
// //     if ( rate_control_rotation_on ){
// //       const double sign_angle			=	euler_angle_ZYX_Z/fabs(euler_angle_ZYX_Z);
// //       Matrix3d rotRate_mat			=	AngleAxisd( sign_angle * K_rate_control_Rot * time_increment_ , Eigen::Vector3d::UnitZ() ).toRotationMatrix();
// //       T_0s_s_S.linear()				=	rotRate_mat		*	T_0s_s_S.linear();
// //     }
// //     
// //     //		Position:
// //     const Vector3d T_is_s_S_Pos = T_is_0s_S.translation();
// //     const double D				=	K_position_control_Pos	*	sqrt( T_is_s_S_Pos(0) * T_is_s_S_Pos(0) + T_is_s_S_Pos(1) * T_is_s_S_Pos(1) );
// //     if ( D > R )	rate_control_position_on = true;
// //     
// //     if ( rate_control_position_on ){
// //       Vector3d ratePosInc = ( 1 - R/D ) * T_0s_s_S.linear() * Vector3d( T_is_s_S_Pos(0), T_is_s_S_Pos(1), 0.0 );
// //       //     Dps_0 = K_2 * ds		<-->		ps_0_{i+1} = ps_0_{i} + K_2 * ds * u( |ds| - R ) * T
// //       T_0s_s_S.translation() 			+=	K_rate_control_Pos	*	time_increment_	*	ratePosInc;
// //     }
// //   }
// //   
// //   T_is_s_S = T_0s_s_S * T_is_0s_S;
    
    
    
    
    
  
  
  // Camera Pose ------------------------------------------------------
  
  //   Stylus depth command
//   const double haptic_stylus_angle = euler_angle_ZYX_X;
//   const double haptic_stylus_angle_depth_on = 22.5 * (PI/180);
//   const double plat2cam_dist_min = 1.5;
//   const double plat2cam_dist_max = 10.0;
//   if (indexing_on){
// //   if ( rate_control_position_on || rate_control_rotation_on ){
//     if ( fabs(haptic_stylus_angle) > haptic_stylus_angle_depth_on ){
//       origin_cam_dist += ( fabs(haptic_stylus_angle)/haptic_stylus_angle ) * time_increment_;
//       if ( origin_cam_dist < plat2cam_dist_min ) 		origin_cam_dist = plat2cam_dist_min;    
//       if ( origin_cam_dist > plat2cam_dist_max ) 		origin_cam_dist = plat2cam_dist_max;
//     }
//   }
  
  Eigen::Vector3d origin_cam_dir(1.0, 0.0, 0.35);
  Eigen::Vector3d origin_cam = origin_cam_dist * origin_cam_dir;
  
  //   Get rotation angle
  const double cam_angle = AngleAxisd( Quaterniond( T_0s_s_S.linear() ) ).angle() * AngleAxisd( Quaterniond( T_0s_s_S.linear() ) ).axis()(2);

  origin_cam = AngleAxisd( cam_angle , Eigen::Vector3d::UnitZ() ).toRotationMatrix() * origin_cam;

  Vector3d camPointOffset(0.0, 0.0, 1.0);
  
  T_camPose_S.translation() = T_0s_s_S.translation() + camPointOffset + origin_cam;  
  
  
  
  // Gather data ------------------------------------------------------
  //   Slave Workspace origin
  ps_0 = T_0s_s_S.translation();
  quats_0 = T_0s_s_S.linear();
  //   Slave Workspace pose
  ps_i = T_is_s_S.translation();
  quats_i = T_is_s_S.linear();
  
  // Limits
  if (ps_0(0) < Xmin)		ps_0(0) = Xmin;
  if (ps_0(0) > Xmax)		ps_0(0) = Xmax;
  if (ps_0(1) < Ymin)		ps_0(1) = Ymin; 
  if (ps_0(1) > Ymax)		ps_0(1) = Ymax; 
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;
  

  
//   std::cout << " VISUALIZATION ---------------------------------------------------------------------------------------" << std::endl;
//   std::cout << "  Rm_0:          " << std::endl << Rm_0 << std::endl;  
//   std::cout << "  Rm:            " << std::endl << Rm << std::endl;  
//   std::cout << "  Rm_i:          " << std::endl << Rm_i << std::endl;   
//   std::cout << "  Rm*Rm_i*Rm^T:  " << std::endl << Rm * Rm_i * Rm.transpose() << std::endl;
//   std::cout << "  Rs_0:          " << std::endl << Rs_0 << std::endl; 
//   std::cout << "  ang_xy_raw:    " << asin(x_rm_xy(1))*(180/PI) << std::endl;
//   std::cout << "  pm_i:          " << pm_i.transpose()<< std::endl;
//   std::cout << "  T_im_m_M:      " << std::endl<< T_im_m_M.affine()<< std::endl;  
//   std::cout << "  T_im_m_M_2:    " << std::endl<< T_im_m_M_2.affine()<< std::endl;
//   std::cout << "  resta:         " << std::endl<< T_im_m_M.affine() - T_im_m_M_2.affine()<< std::endl;
//   std::cout << "  T_0s_s_S.affine:    " << std::endl<< T_0s_s_S.affine()<< std::endl;
//   std::cout << "  angle:         " << AngleAxisd( Quaterniond(T_0s_s_S.linear()) ).angle() * (180/PI) << std::endl;
//   std::cout << "  T_is_0s_S.affine: " << std::endl<< T_is_0s_S.affine()<< std::endl;
//   std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;
//   std::cout << "  T_0sInitIndex_s_S.affine:  " << std::endl<< T_0sInitIndex_s_S.affine()<< std::endl;
}



void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v3(void){
  
  // ****************************************************************************************************************
  // Inner workspace position control with rate control on the workspace outter layer
  // When omni_grey_button_pressed is TRUE then an indexing technique is applied instead of the position control
  // ****************************************************************************************************************
  
  
  // MASTER reference frame --------------------------------------------------------------
  Eigen::Transform<double,3,Affine>	T_im_m_M  =	Translation3d(pm_i)	*	AngleAxisd(quatm_i);
  Eigen::Transform<double,3,Affine>	T_0m_m_M  =	Translation3d(pm_0)	*	AngleAxisd(quatm_0);
  Eigen::Transform<double,3,Affine>	T_m_W 	  =	Translation3d(pm)	*	AngleAxisd(quatm);
  Eigen::Transform<double,3,Affine>	T_im_0m_W =	T_m_W 			*	T_0m_m_M.inverse()	*	T_im_m_M	*	T_m_W.inverse();  

  T_HIPrH = T_im_0m_W;
  


  


 
  // Worskspace Mapping algorithm ------------------------------------------------------


  //  Algorithm data **********************
  
  //	Scaling control
  Matrix3d K_s_m(Matrix3d::Identity());
  K_s_m(0,0) = scale[0];
  K_s_m(1,1) = scale[1];
  K_s_m(2,2) = scale[2];
  const double K_position_control_Pos		=	10.0;

  //	Indexing
  const double K_indexing_Pos			=	1.0;
  const double K_indexing_Rot			=	1.0;  
  
  //	Rate control
  const double R 				=	0.5;  
  const double K_rate_control_Pos		=	20.0;
  const double maxAng_width			=	45.0 * (PI/180);
  const double maxAng_heigth			=	45.0 * (PI/180);
  const double K_rate_control_Rot		=	18.0 * (PI/180);
  
  
  //   Algorithms setting *****************

  //     Euler Angles Z Y X convention ( matrix multiplication = R_Z R_Y R_X )
//   const double euler_angle_ZYX_Z		=	atan2(   T_is_0s_S(1,0) , T_is_0s_S(0,0) );
//   const double euler_angle_ZYX_Y		=	atan2( - T_is_0s_S(2,0) , sqrt( T_is_0s_S(0,0)*T_is_0s_S(0,0) + T_is_0s_S(1,0)*T_is_0s_S(1,0) ) );
//   const double euler_angle_ZYX_X		=	atan2(   T_is_0s_S(2,1) , T_is_0s_S(2,2) );

  //	 Indexing
  bool indexing_on				=	omni_white_button_pressed && !omni_grey_button_pressed;
    
//   //	 Rate control
//   //		Rotation
//   bool rate_control_rotation_on			=	false;
//   if ( (fabs(euler_angle_ZYX_Z) > maxAng_width) && (fabs(euler_angle_ZYX_Y) < maxAng_heigth) )		rate_control_rotation_on = true;
//   //		Translation
//   bool rate_control_position_on 		=	false;
//   const Vector3d T_is_s_S_Pos			=	T_is_0s_S.translation();
//   const double D				=	K_position_control_Pos	*	sqrt( T_is_s_S_Pos(0) * T_is_s_S_Pos(0) + T_is_s_S_Pos(1) * T_is_s_S_Pos(1) );
//   if ( D > R )												rate_control_position_on = true;
//   

  
  // Camera setting
  T_CrVH.linear()		=	Eigen::Matrix3d::Identity();
  T_CrVH.translation()		=	origin_cam_dist * Eigen::Vector3d(1.0, 0.0, 0.35);
  
  
  
  //   Algorithms execution ***************
  
  //	Scaling control ----------------
  T_VHIPrVH.linear()		=	T_HIPrH.linear();
  T_VHIPrVH.translation()	=	K_position_control_Pos	*	K_s_m	*	T_HIPrH.translation();



//     T_HIPrH, T_VHIPrVH, T_VHrW, T_TCPrW, T_TCPrVHIP, T_CrW;
  

  

// //   //    Indexing ------------------------
// //   if (!init_indexing_offset && mi_init_changed && m_init && s_init && s0_init){
// // 
// //     T_TCPrW_detach	=	Eigen::Matrix4d::Identity();
// //     
// //     init_indexing_offset = true;
// //   }
// //   
// //   if (indexing_on)	// detached
// //   {
// //     T_TCPrVHIP		=	Eigen::Matrix4d::Identity();
// //     T_TCPrVHIP.linear()	=	T_VHIPrVH.linear().transpose()	*	T_CrVH.linear()		*	
// //     
// //     
// //     T_VHrW		=	T_TCPrW_detach	*	T_TCPrVHIP.inverse()	*	T_VHIPrVH.inverse();
// //     
// //     T_VHIPrVH_attach	=	T_VHIPrVH;
// //   }  
// //   else			// attached
// //   {
// //     //   //	Rate control --------------------
// //     //   //		Rotation    
// //     //   if ( rate_control_rotation_on )
// //     //   {
// //     // 
// //     //   }
// //     //   //		Translation
// //     //   if ( rate_control_position_on )
// //     //   {
// //     // 
// //     //   }
// //     
// //     
// //     T_TCPrW		=	T_VHrW		*	T_VHIPrVH	*	T_TCPrVHIP;
// //     
// //     T_TCPrW_detach	=	T_TCPrW;
// //   }
  
  


  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

  
  
  
  // Gather data ------------------------------------------------------
  //   Slave Workspace origin
  ps_0		=	T_VHrW.translation();
  quats_0	=	T_VHrW.linear();
  //   Slave Workspace pose
  ps_i		=	T_TCPrW.translation();
  quats_i	=	T_TCPrW.linear();
  
  // Limits
  if (ps_0(0) < Xmin)		ps_0(0) = Xmin;
  if (ps_0(0) > Xmax)		ps_0(0) = Xmax;
  if (ps_0(1) < Ymin)		ps_0(1) = Ymin; 
  if (ps_0(1) > Ymax)		ps_0(1) = Ymax; 
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;
  

  
//   std::cout << " VISUALIZATION ---------------------------------------------------------------------------------------" << std::endl;
//   std::cout << "  Rm_0:          " << std::endl << Rm_0 << std::endl;  
//   std::cout << "  Rm:            " << std::endl << Rm << std::endl;  
//   std::cout << "  Rm_i:          " << std::endl << Rm_i << std::endl;   
//   std::cout << "  Rm*Rm_i*Rm^T:  " << std::endl << Rm * Rm_i * Rm.transpose() << std::endl;
//   std::cout << "  Rs_0:          " << std::endl << Rs_0 << std::endl; 
//   std::cout << "  ang_xy_raw:    " << asin(x_rm_xy(1))*(180/PI) << std::endl;
//   std::cout << "  pm_i:          " << pm_i.transpose()<< std::endl;
//   std::cout << "  T_im_m_M:      " << std::endl<< T_im_m_M.affine()<< std::endl;  
//   std::cout << "  T_im_m_M_2:    " << std::endl<< T_im_m_M_2.affine()<< std::endl;
//   std::cout << "  resta:         " << std::endl<< T_im_m_M.affine() - T_im_m_M_2.affine()<< std::endl;
//   std::cout << "  T_0s_s_S.affine:    " << std::endl<< T_0s_s_S.affine()<< std::endl;
//   std::cout << "  angle:         " << AngleAxisd( Quaterniond(T_0s_s_S.linear()) ).angle() * (180/PI) << std::endl;
//   std::cout << "  T_is_0s_S.affine: " << std::endl<< T_is_0s_S.affine()<< std::endl;
//   std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;
//   std::cout << "  T_0sInitIndex_s_S.affine:  " << std::endl<< T_0sInitIndex_s_S.affine()<< std::endl;
}




void ws_tf_alg_scaling_control_with_posrot_rate_control_constant_velocity_with_indexing_v4(void){
  
  // ****************************************************************************************************************
  // Inner workspace position control with rate control on the workspace outter layer
  // When omni_grey_button_pressed is TRUE then an indexing technique is applied instead of the position control
  // ****************************************************************************************************************

  
  
  // Worskspace Mapping algorithm ------------------------------------------------------


  //  Algorithm data **********************
  
  //	Scaling control
  Matrix3d K_s_m(Matrix3d::Identity());
  K_s_m(0,0) = scale[0];
  K_s_m(1,1) = scale[1];
  K_s_m(2,2) = scale[2];
  const double K_position_control_Pos		=	10.0;

  //	Indexing
  const double K_indexing_Pos			=	1.0;
  const double K_indexing_Rot			=	1.0;  
  
  //	Rate control
  const double R 				=	0.5;  
  const double K_rate_control_Pos		=	20.0;
  const double maxAng_width			=	45.0 * (PI/180);
  const double maxAng_heigth			=	45.0 * (PI/180);
  const double K_rate_control_Rot		=	18.0 * (PI/180);
  
  //	Camera
  Eigen::Vector3d origin_cam		=	7.0 * Eigen::Vector3d(1.0, 0.0, 0.35);
  T_VH_C.translation()			=	Eigen::Vector3d(0.0, 0.0, 1.0)	+	origin_cam;

  Eigen::Vector3d eigen_cam_axis_z	=	origin_cam						 .normalized();
  Eigen::Vector3d eigen_cam_axis_x	=	( Eigen::Vector3d::UnitZ().cross( eigen_cam_axis_z ) 	).normalized();
  Eigen::Vector3d eigen_cam_axis_y	=	( eigen_cam_axis_z.cross( eigen_cam_axis_x ) 		).normalized();

  T_VH_C.linear() << eigen_cam_axis_x(0), eigen_cam_axis_y(0), eigen_cam_axis_z(0), 
		eigen_cam_axis_x(1), eigen_cam_axis_y(1), eigen_cam_axis_z(1),
		eigen_cam_axis_x(2), eigen_cam_axis_y(2), eigen_cam_axis_z(2);
  


  
  
  
  // MASTER reference frame --------------------------------------------------------------
  Eigen::Transform<double,3,Affine>	T_im_m_M  =	Translation3d(pm_i)	*	AngleAxisd(quatm_i);
  Eigen::Transform<double,3,Affine>	T_0m_m_M  =	Translation3d(pm_0)	*	AngleAxisd(quatm_0);
  Eigen::Transform<double,3,Affine>	T_m_W 	  =	Translation3d(pm)	*	AngleAxisd(quatm);
  Eigen::Transform<double,3,Affine>	T_im_0m_W =	T_m_W 			*	T_0m_m_M.inverse()	*	T_im_m_M	*	T_m_W.inverse();  

  
  // Scaling from Master Environment to Slave Environment --------------------------------
  Eigen::Transform<double,3,Affine> 	T_is_0s_W =	Translation3d( K_position_control_Pos * K_s_m * T_im_0m_W.translation() ) 	* 
								AngleAxisd( Quaterniond( T_im_0m_W.linear() ) );


  // SLAVE reference frame --------------------------------------------------------------
  Eigen::Transform<double,3,Affine>	T_s_W 	  =	Translation3d(ps)	*	AngleAxisd(quats);
  Eigen::Transform<double,3,Affine>	T_0s_s_S  =	Translation3d(ps_0)	*	AngleAxisd(quats_0);
  Eigen::Transform<double,3,Affine>	T_is_0s_S =	T_s_W.inverse()		*	T_is_0s_W	*	T_s_W;
  Eigen::Transform<double,3,Affine>	T_is_s_S;

  T_VH_TCP = T_is_0s_S;

  
  
  //   Algorithms setting *****************

  //     Euler Angles Z Y X convention ( matrix multiplication = R_Z R_Y R_X )
  const double euler_angle_ZYX_Z		=	atan2(   T_VH_TCP(1,0) , T_VH_TCP(0,0) );
  const double euler_angle_ZYX_Y		=	atan2( - T_VH_TCP(2,0) , sqrt( T_VH_TCP(0,0)*T_VH_TCP(0,0) + T_VH_TCP(1,0)*T_VH_TCP(1,0) ) );
  const double euler_angle_ZYX_X		=	atan2(   T_VH_TCP(2,1) , T_VH_TCP(2,2) );

  //	 Indexing
  bool indexing_on				=	omni_white_button_pressed && !omni_grey_button_pressed;
    
  //	 Rate control
  //		Rotation
  bool rate_control_rotation_on			=	false;
  if ( (fabs(euler_angle_ZYX_Z) > maxAng_width) && (fabs(euler_angle_ZYX_Y) < maxAng_heigth) )		rate_control_rotation_on = true;
  //		Translation
  bool rate_control_position_on 		=	false;
  const Vector3d T_VH_TCP_Pos			=	T_VH_TCP.translation();
  const double D				=	K_position_control_Pos	*	sqrt( T_VH_TCP_Pos(0) * T_VH_TCP_Pos(0) + T_VH_TCP_Pos(1) * T_VH_TCP_Pos(1) );
  if ( D > R )												rate_control_position_on = true;


  
  //   Algorithms execution ***************
  
  //    Indexing ------------------------
  if (!indexing_on)
  {
// //       //	Rate control --------------------
// //       //		Rotation    
// //       if ( rate_control_rotation_on )
// //       {
// //     
// //       }
// //       //		Translation
// //       if ( rate_control_position_on )
// //       {
// //     
// //       }

    T_W_TCP	=	T_W_VH * T_VH_TCP;
  }  
  else	// indexing on
  {
    T_W_VH	=	T_W_TCP * T_VH_TCP.inverse();
  }
  
  

  // Camera Pose ------------------------------------------------------
  T_W_C		=	T_W_TCP * T_VH_TCP;
  
  
  
  
 
  // Gather data ------------------------------------------------------ 
  T_camPose_S	=	T_W_C;
  T_is_s_S	=	T_W_TCP;
    
  //   Slave Workspace origin
  ps_0 = T_0s_s_S.translation();
  quats_0 = T_0s_s_S.linear();
  //   Slave Workspace pose
  ps_i = T_is_s_S.translation();
  quats_i = T_is_s_S.linear();
  
  
  // Limits
  if (ps_0(0) < Xmin)		ps_0(0) = Xmin;
  if (ps_0(0) > Xmax)		ps_0(0) = Xmax;
  if (ps_0(1) < Ymin)		ps_0(1) = Ymin; 
  if (ps_0(1) > Ymax)		ps_0(1) = Ymax; 
  
  if (ps_i(0) < Xmin)		ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax)		ps_i(0) = Xmax;
  if (ps_i(1) < Ymin)		ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax)		ps_i(1) = Ymax;
  

  
//   std::cout << " VISUALIZATION ---------------------------------------------------------------------------------------" << std::endl;
//   std::cout << "  Rm_0:          " << std::endl << Rm_0 << std::endl;  
//   std::cout << "  Rm:            " << std::endl << Rm << std::endl;  
//   std::cout << "  Rm_i:          " << std::endl << Rm_i << std::endl;   
//   std::cout << "  Rm*Rm_i*Rm^T:  " << std::endl << Rm * Rm_i * Rm.transpose() << std::endl;
//   std::cout << "  Rs_0:          " << std::endl << Rs_0 << std::endl; 
//   std::cout << "  ang_xy_raw:    " << asin(x_rm_xy(1))*(180/PI) << std::endl;
//   std::cout << "  pm_i:          " << pm_i.transpose()<< std::endl;
//   std::cout << "  T_im_m_M:      " << std::endl<< T_im_m_M.affine()<< std::endl;  
//   std::cout << "  T_im_m_M_2:    " << std::endl<< T_im_m_M_2.affine()<< std::endl;
//   std::cout << "  resta:         " << std::endl<< T_im_m_M.affine() - T_im_m_M_2.affine()<< std::endl;
//   std::cout << "  T_0s_s_S.affine:    " << std::endl<< T_0s_s_S.affine()<< std::endl;
//   std::cout << "  angle:         " << AngleAxisd( Quaterniond(T_0s_s_S.linear()) ).angle() * (180/PI) << std::endl;
//   std::cout << "  T_is_0s_S.affine: " << std::endl<< T_is_0s_S.affine()<< std::endl;
//   std::cout << "  T_is_s_S.affine:  " << std::endl<< T_is_s_S.affine()<< std::endl;
//   std::cout << "  T_0sInitIndex_s_S.affine:  " << std::endl<< T_0sInitIndex_s_S.affine()<< std::endl;
}


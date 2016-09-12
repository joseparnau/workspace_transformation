#include <ros/ros.h>

#include "phantom_omni/PhantomButtonEvent.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#include "workspace_transformation/setAlgorithm.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <cmath>

#define N_HAPTIC_JOINTS 6


using namespace Eigen;


// Global variables ------------------------------

double loop_rate_in_hz = 100;

//  Haptic
bool omni_white_button_pressed, omni_grey_button_pressed;
bool first_haptic_angle_read;
double haptic_angles[N_HAPTIC_JOINTS];
ros::Publisher pub_OmniForceFeedback;
geometry_msgs::Vector3 force;

//  Algorithm
unsigned int algorithm_type;
double scale[3];	// NewWorkspace / BaseWorkspace
bool m_init, mi_init, s_init, s0_init, algorithm_set, mi_init_changed;
Matrix<double,3,1> pm_i, pm_im1, ps_i, ps_im1, pm_0, ps_0, ps, pm, dm, ds, vm_i, p_aux, dm_aux, ds_aux;
Matrix3d Rm_i(3,3), Rs_i(3,3), Rm_0(3,3), Rs_0(3,3), Rm(3,3), Rs(3,3), Ks(3,3);
Quaternion<double> quatm_0, quatm_i, quatm, quats, quats_0, quats_i;
double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;

//  Camera info
Eigen::Transform<double,3,Affine> T_camPose_S;

//  Time management
double period;
double time_increment_;


// Subscriber functions ----------------------------
void PoseMasterWSCallback(const geometry_msgs::PoseStamped& msg){
  
  pm_i << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  quatm_i.x() = msg.pose.orientation.x;
  quatm_i.y() = msg.pose.orientation.y;
  quatm_i.z() = msg.pose.orientation.z;
  quatm_i.w() = msg.pose.orientation.w;
  Rm_i = quatm_i.toRotationMatrix();
  
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
}

void omniButtonCallback(const phantom_omni::PhantomButtonEvent& msg)
{
  omni_grey_button_pressed = (bool)msg.grey_button;
  omni_white_button_pressed = (bool)msg.white_button;
}

void HapticAnglesCallback(const sensor_msgs::JointState& msg)
{
  for (unsigned int i=0; i< N_HAPTIC_JOINTS; i++)	haptic_angles[i] = msg.position[i];  
  first_haptic_angle_read = true;  
}

bool algorithmSrvCallback(workspace_transformation::setAlgorithm::Request &req, 
			  workspace_transformation::setAlgorithm::Response &res){
  algorithm_type = req.algorithm_number;
  
  if (!algorithm_set)	algorithm_set = true;
  
  ROS_INFO(" Workspace transformation algorithm set to id %d", algorithm_type);
  
  return true;
}


// Workspace mapping functions ---------------------
void ws_tf_alg_scaling_smoothposrotRateControl_camRateControl_FormatJAC(void);
void ws_tf_alg_scaling_Bubble_smoothposrotRateControl_camRateControl_FormatJAC(void);
//  Force Feedback
void force_feedback_pos_rate_control(void);


int main(int argc, char** argv){
  ros::init(argc, argv, "workspace_transformation");

  ros::NodeHandle node;
  ros::Rate loop_rate(loop_rate_in_hz);


  // Topics
  //  Publishers
  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("poseSlaveWorkspace", 1);
  ros::Publisher pub_poseSlaveWSOrigin = node.advertise<geometry_msgs::PoseStamped>("poseSlaveWorkspaceOrigin", 1);
  ros::Publisher pub_set_camera_pose = node.advertise<geometry_msgs::PoseStamped>("Set_ActiveCamera_Pose", 1);
  pub_OmniForceFeedback = node.advertise<geometry_msgs::Vector3>("set_forces", 1);
  //  Subscribers
  ros::Subscriber sub_PoseMasterWS = node.subscribe("poseMasterWorkspace", 1, &PoseMasterWSCallback);
  ros::Subscriber sub_BaseMasterWS = node.subscribe("baseMasterWorkspace", 1, &BaseMasterWSCallback);
  ros::Subscriber sub_BaseSlaveWS = node.subscribe("baseSlaveWorkspace", 1, &BaseSlaveWSCallback);
  ros::Subscriber sub_OriginSlaveWS = node.subscribe("originSlaveWorkspace", 1, &OriginSlaveWSCallback); 
  ros::Subscriber sub_scale = node.subscribe("scale", 1, &scaleCallback);
  ros::Subscriber subOmniButtons = node.subscribe("button", 1, &omniButtonCallback);
  ros::Subscriber sub_HapticAngles = node.subscribe("angles", 1, &HapticAnglesCallback);
  //  Services
  ros::ServiceServer service_server_algorithm = node.advertiseService("set_algorithm", algorithmSrvCallback);

  
  // INITIALIZATION ------------------------------------------------------------------------
  
  //  Haptic
  omni_white_button_pressed = omni_grey_button_pressed = first_haptic_angle_read = false;  
  
  //  Algorithm
  algorithm_type = 0;

  for (unsigned int i=0; i<3; i++)	scale[i] = 1.0;  
  
  m_init = s_init = mi_init = s0_init = algorithm_set = false;

  dm << 0,0,0;
  ds << 0,0,0;
  pm_im1 << 0,0,0;
  ps_im1 << 0,0,0;
  vm_i << 0,0,0;
  
  ps_0 << 0.0, 0.0, 0.0;
  quats_0.x() = quats_0.y() = quats_0.z() = 0.0;
  quats_0.w() = 1.0;
  Rs_0 = quats_0.toRotationMatrix();  

  //   Auxiliary pose
  geometry_msgs::PoseStamped outputPose;  
  outputPose.pose.position.x = outputPose.pose.position.y = outputPose.pose.position.z = 0.0;
  outputPose.pose.orientation.x = outputPose.pose.orientation.y = outputPose.pose.orientation.z = 0.0;
  outputPose.pose.orientation.w = 1.0;  
  
  //   Workspace boundaries
  Xmin = -5.0;
  Ymin = -5.0;
  Zmin = 0.0;
  Xmax = 5.0;
  Ymax = 5.0;
  Zmax = 2.0;
  
  // Default camera pose
  geometry_msgs::PoseStamped cameraPose;  
  cameraPose.pose.position.x = cameraPose.pose.position.y = cameraPose.pose.position.z = 0.0;
  cameraPose.pose.orientation.x = cameraPose.pose.orientation.y = cameraPose.pose.orientation.z = 0.0;
  cameraPose.pose.orientation.w = 1.0;
  
  Eigen::Vector3d origin_cam = 10.0 * Eigen::Vector3d(1.0, 0.0, 0.5);
  T_camPose_S.translation()  = Eigen::Vector3d(0.0, 0.0, 1.0) + origin_cam;

  Eigen::Vector3d eigen_cam_axis_z = origin_cam.normalized();
  Eigen::Vector3d eigen_cam_axis_x = ( Eigen::Vector3d::UnitZ().cross( eigen_cam_axis_z ) ).normalized();
  Eigen::Vector3d eigen_cam_axis_y = ( eigen_cam_axis_z.cross( eigen_cam_axis_x ) ).normalized();

  T_camPose_S.linear() << eigen_cam_axis_x(0), eigen_cam_axis_y(0), eigen_cam_axis_z(0), 
			  eigen_cam_axis_x(1), eigen_cam_axis_y(1), eigen_cam_axis_z(1),
			  eigen_cam_axis_x(2), eigen_cam_axis_y(2), eigen_cam_axis_z(2);
  
  // Time management
  period = 1.0/(double)loop_rate_in_hz;
  timeval past_time_, current_time_;  
  gettimeofday(&current_time_, NULL);  
  time_increment_ = 0;
  
  // File management
  std::ofstream WTdataRecord;  
  WTdataRecord.open("/home/users/josep.a.claret/data/WTdataRecord.txt", std::ofstream::trunc);
    
  
  // UPDATE -------------------------------------------------------------------------------
  while (ros::ok())
  {
    if (m_init && s_init && mi_init)
    {
      // Time management
//       past_time_ = current_time_;
//       gettimeofday(&current_time_, NULL);
//       time_increment_ = ( (current_time_.tv_sec*1e6 + current_time_.tv_usec) - (past_time_.tv_sec*1e6 + past_time_.tv_usec) ) / 1e6;
      time_increment_ = period;
      
      // Velocity computation
      vm_i = (pm_i - pm_im1)/time_increment_; 
      
      ws_tf_alg_scaling_Bubble_smoothposrotRateControl_camRateControl_FormatJAC();
            
            
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
    }
    
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
    

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  WTdataRecord.close();
  
  return 0;
}


void ws_tf_alg_scaling_smoothposrotRateControl_camRateControl_FormatJAC(void){
    
  // ****************************************************************************************************************
  // Inner workspace Smooth position & orientation control with camera rate control on the workspace outter layer
  // ****************************************************************************************************************
  
  // MASTER reference frame --------------------------------------------------------------
  Eigen::Transform<double,3,Affine> T_im_m_M  = Translation3d(pm_i)	*	AngleAxisd(quatm_i);
  Eigen::Transform<double,3,Affine> T_0m_m_M  = Translation3d(pm_0)	*	AngleAxisd(quatm_0);
  Eigen::Transform<double,3,Affine> T_m_W     = Translation3d(pm)	*	AngleAxisd(quatm);
  Eigen::Transform<double,3,Affine> T_im_0m_W = T_m_W * T_0m_m_M.inverse() * T_im_m_M * T_m_W.inverse();  

  // Scaling from Master Environment to Slave Environment --------------------------------
  Matrix3d K_s_m(Matrix3d::Identity());
  K_s_m(0,0) = scale[0];
  K_s_m(1,1) = scale[1];
  K_s_m(2,2) = scale[2];

  Eigen::Transform<double,3,Affine> T_is_0s_W = Translation3d( K_s_m * T_im_0m_W.translation() ) * AngleAxisd( Quaterniond( T_im_0m_W.linear() ) );

  // SLAVE reference frame --------------------------------------------------------------
  Eigen::Transform<double,3,Affine> T_s_W     = Translation3d(ps)   * AngleAxisd(quats);
  Eigen::Transform<double,3,Affine> T_0s_s_S  = Translation3d(ps_0) * AngleAxisd(quats_0);
  Eigen::Transform<double,3,Affine> T_is_0s_S = T_s_W.inverse()     * T_is_0s_W * T_s_W;
  Eigen::Transform<double,3,Affine> T_is_s_S;

  // SLAVE workspace frames --------------------------------------------------------------
  Eigen::Transform<double,3,Affine> T_W_TCP, T_W_VH, T_VH_TCP, T_W_C, T_VH_C;
  T_W_VH   = T_0s_s_S;
  T_VH_TCP = T_is_0s_S;
 
  // Camera offset --------------------------------------------------------------
  Eigen::Vector3d origin_cam = 5.0 * Eigen::Vector3d(1.0, 0.0, 0.6);

  T_VH_C.translation() = origin_cam;
  Eigen::Vector3d eigen_cam_axis_z = origin_cam.normalized();
  Eigen::Vector3d eigen_cam_axis_x = ( Eigen::Vector3d::UnitZ().cross( eigen_cam_axis_z ) ).normalized();
  Eigen::Vector3d eigen_cam_axis_y = ( eigen_cam_axis_z.cross( eigen_cam_axis_x ) ).normalized();

  T_VH_C.linear() << eigen_cam_axis_x(0), eigen_cam_axis_y(0), eigen_cam_axis_z(0), 
                     eigen_cam_axis_x(1), eigen_cam_axis_y(1), eigen_cam_axis_z(1),
                     eigen_cam_axis_x(2), eigen_cam_axis_y(2), eigen_cam_axis_z(2);
  
 
  // Worskspace Mapping algorithm ------------------------------------------------------

  //  Algorithm data **********************
  
  //   Position control
  const double K_position_control_Pos = 10.0;

  //   Rate control
  const double R = 0.5;  
  const double K_rate_control_Pos = 3.0;
  const double maxAng_width       = 45.0 * (M_PI/180);
  const double maxAng_heigth      = 45.0 * (M_PI/180);
  const double K_rate_control_Rot = 75.0 * (M_PI/180);
  
  
  //   Algorithms setting *****************

  //     Euler Angles Z Y X convention ( matrix multiplication = R_Z R_Y R_X )
  const double euler_angle_ZYX_Z = atan2(   T_VH_TCP(1,0) , T_VH_TCP(0,0) );
  const double euler_angle_ZYX_Y = atan2( - T_VH_TCP(2,0) , sqrt( T_VH_TCP(0,0)*T_VH_TCP(0,0) + T_VH_TCP(1,0)*T_VH_TCP(1,0) ) );
  const double euler_angle_ZYX_X = atan2(   T_VH_TCP(2,1) , T_VH_TCP(2,2) );
    
  //     Rate control
  //       Rotation
  bool rate_control_rotation_on = false;
  if ( (fabs(euler_angle_ZYX_Z) > maxAng_width) && (fabs(euler_angle_ZYX_Y) < maxAng_heigth) ) rate_control_rotation_on = true;
  //       Translation
  bool rate_control_position_on = false;
  const Vector3d T_VH_TCP_Pos = T_VH_TCP.translation();
  const double D              = K_position_control_Pos * sqrt( T_VH_TCP_Pos(0) * T_VH_TCP_Pos(0) + T_VH_TCP_Pos(1) * T_VH_TCP_Pos(1) );
  if ( D > R ) rate_control_position_on = true;


  //   Algorithms execution ***************
  
  //     Scaling ----------------
  T_VH_TCP.translation() = K_position_control_Pos * T_VH_TCP.translation();
  T_W_TCP                = T_W_VH * T_VH_TCP;

  //     Rate control --------------------
  //       Rotation
  if ( rate_control_rotation_on )
  {
    double rateRotInc    = euler_angle_ZYX_Z/fabs(euler_angle_ZYX_Z) * (1 - maxAng_width/fabs(euler_angle_ZYX_Z));
    Matrix3d rotRate_mat = AngleAxisd( rateRotInc * K_rate_control_Rot * time_increment_ , Eigen::Vector3d::UnitZ() ).toRotationMatrix();    
    T_W_VH.linear()      = rotRate_mat * T_W_VH.linear();
  }
  //       Translation
  if ( rate_control_position_on )
  {
    Vector3d ratePosInc   = ( 1 - R/D ) * T_W_VH.linear() * Vector3d( T_VH_TCP.translation()(0), T_VH_TCP.translation()(1), 0.0 ).normalized();
    T_W_VH.translation() += K_rate_control_Pos * time_increment_ * ratePosInc;
  } 

  //  Camera Pose --------------------
  T_W_C = T_W_VH * T_VH_C;

  
  // Gather data ------------------------------------------------------
  //   Slave Workspace origin
  ps_0    = T_W_VH.translation();
  quats_0 = T_W_VH.linear();
  //   Slave Workspace pose
  ps_i    = T_W_TCP.translation();
  quats_i = T_W_TCP.linear();
  //   Camera pose
  T_camPose_S = T_W_C;
  
  // Limits
  if (ps_0(0) < Xmin) ps_0(0) = Xmin;
  if (ps_0(0) > Xmax) ps_0(0) = Xmax;
  if (ps_0(1) < Ymin) ps_0(1) = Ymin; 
  if (ps_0(1) > Ymax) ps_0(1) = Ymax; 
  
  if (ps_i(0) < Xmin) ps_i(0) = Xmin;  
  if (ps_i(0) > Xmax) ps_i(0) = Xmax;
  if (ps_i(1) < Ymin) ps_i(1) = Ymin;  
  if (ps_i(1) > Ymax) ps_i(1) = Ymax;  
}

void ws_tf_alg_scaling_Bubble_smoothposrotRateControl_camRateControl_FormatJAC(void){
  
  ws_tf_alg_scaling_smoothposrotRateControl_camRateControl_FormatJAC();
  
  force_feedback_pos_rate_control();
}


void force_feedback_pos_rate_control(void){
  
  // ****************************************************************************************************************
  // Force feedback when in rate control area
  // ****************************************************************************************************************
  
  // Variables
  Ks = MatrixXd::Zero(3,3);
  Ks(0,0) = scale[0];
  Ks(1,1) = scale[1];
  Ks(2,2) = scale[2];
 
  Eigen::Vector3d omni_force = Eigen::Vector3d::Zero();
  double R = 0.05;
  double Fmax = 3.3; // 3.3 N es la máxima força nominal
  double Kf = 60;
  double Kv = 0.5;

  // Displacement
  dm = pm_i - pm_0;
  ds = Ks * Rs.transpose() * Rm * dm;  
  
  // Omni feedback force computation
  double D = sqrt( ds(0)*ds(0) + ds(1)*ds(1) );  
  
  if (D > R)	omni_force = - Kf * (D - R) * (ds/D);
  omni_force += - Kv*vm_i; // Damping
  omni_force(2) = 0.0;     // No force in vertical axis
  omni_force = Rm.transpose() * Rs * omni_force; // To master reference frame
  
  if (omni_force(0) < -Fmax) omni_force(0) = -Fmax;
  if (omni_force(0) > Fmax)  omni_force(0) = Fmax;
  if (omni_force(1) < -Fmax) omni_force(1) = -Fmax;
  if (omni_force(1) > Fmax)  omni_force(1) = Fmax;  

  // Publish
  force.x = omni_force(0);
  force.y = omni_force(1);
  force.z = omni_force(2);
  pub_OmniForceFeedback.publish(force);
}
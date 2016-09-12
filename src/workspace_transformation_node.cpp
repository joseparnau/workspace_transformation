#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_broadcaster.h>


// Global variables
double scale[3];	// NewWorkspace / BaseWorkspace

bool init;

tf::Transform Tm, Ts, Tmo, Tso, Toffset;


// Subscriber functions
void PoseMasterWSCallback(const geometry_msgs::PoseStamped& msg){
  
  Tm.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
  Tm.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
  
  if (!init)	init = true;
}

void OriginMasterWSCallback(const geometry_msgs::PoseStamped& msg){
  
  Tmo.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
  Tmo.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
  
  // Increments absoluts 
  Toffset = Tmo.inverse() * Tso;
  Toffset.setOrigin(tf::Vector3(0., 0., 0.));
}

void OriginSlaveWSCallback(const geometry_msgs::PoseStamped& msg){
  
  Tso.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
  Tso.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
  
  // Increments absoluts 
  Toffset = Tmo.inverse() * Tso;
  Toffset.setOrigin(tf::Vector3(0., 0., 0.));
}

void scaleCallback(const geometry_msgs::Vector3& msg){

  scale[0] = msg.x;
  scale[1] = msg.y;
  scale[2] = msg.z;
  ROS_INFO(" New Scale:  %.2f %.2f %.2f", scale[0], scale[1], scale[2]);  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "workspace_transformation");

  ros::NodeHandle node;
  ros::Rate loop_rate(10);


  // Topics
  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("poseNewWorkspace", 1000);

  ros::Subscriber sub_PoseBaseWS = node.subscribe("poseMasterWorkspace", 10, &PoseMasterWSCallback);
  ros::Subscriber sub_OriginBaseWS = node.subscribe("originMasterWorkspace", 10, &OriginMasterWSCallback);
  ros::Subscriber sub_OriginNewWS = node.subscribe("originSlaveWorkspace", 10, &OriginSlaveWSCallback);  
  ros::Subscriber sub_scale = node.subscribe("scale", 10, &scaleCallback);


  // Variables
  init = false;
  
  for (unsigned int i=0; i<3; i++)	scale[i] = 1;

  geometry_msgs::PoseStamped outputPose;  
  outputPose.pose.position.x = 0;	outputPose.pose.position.y = 0;		outputPose.pose.position.z = 0;
  outputPose.pose.orientation.x = 0;	outputPose.pose.orientation.y = 0;	outputPose.pose.orientation.z = 0;	outputPose.pose.orientation.w = 1;
  
  //  Tf Broadcasters: init
  Tm.setOrigin(tf::Vector3(0., 0., 0.));	Tm.setRotation(tf::Quaternion(0., 0., 0., 1.));
  Ts.setOrigin(tf::Vector3(0., 0., 0.));	Ts.setRotation(tf::Quaternion(0., 0., 0., 1.));
  Tmo.setOrigin(tf::Vector3(0., 0., 0.));	Tmo.setRotation(tf::Quaternion(0., 0., 0., 1.));
  Tso.setOrigin(tf::Vector3(0., 0., 0.));	Tso.setRotation(tf::Quaternion(0., 0., 0., 1.));
  Toffset.setOrigin(tf::Vector3(0., 0., 0.));	Toffset.setRotation(tf::Quaternion(0., 0., 0., 1.));

  //  Auxiliary variables
  tf::Transform T1, T2, Tp;
  Tp.setRotation(tf::Quaternion(0., 0., 0., 1.));
  double p1[3], p2[3];
  
  while (ros::ok())
  {
    
    if (init)	// only publish if received first 
    {
      // Increments absoluts 
      
      //  Rotation
      Ts = Toffset*Tm;

      //  Position
      Tp.setOrigin(tf::Vector3(Tm.getOrigin()[0], Tm.getOrigin()[1], Tm.getOrigin()[2]));
      T1 = Toffset*Tp;
      p1[0] = T1.getOrigin()[0];
      p1[1] = T1.getOrigin()[1];
      p1[2] = T1.getOrigin()[2];
      Tp.setOrigin(tf::Vector3(Tmo.getOrigin()[0], Tmo.getOrigin()[1], Tmo.getOrigin()[2]));
      T2 = Toffset*Tp;
      p2[0] = T2.getOrigin()[0];
      p2[1] = T2.getOrigin()[1];
      p2[2] = T2.getOrigin()[2];      
      Ts.setOrigin(tf::Vector3(Tso.getOrigin()[0] + scale[0]*p1[0] - scale[0]*p2[0],
			       Tso.getOrigin()[1] + scale[1]*p1[1] - scale[1]*p2[1],
			       Tso.getOrigin()[2] + scale[2]*p1[2] - scale[2]*p2[2]));

      // Send data
      outputPose.header.stamp = ros::Time::now();
      outputPose.pose.position.x = Ts.getOrigin()[0];
      outputPose.pose.position.y = Ts.getOrigin()[1];
      outputPose.pose.position.z = Ts.getOrigin()[2];
      outputPose.pose.orientation.x = Ts.getRotation()[0];
      outputPose.pose.orientation.y = Ts.getRotation()[1];
      outputPose.pose.orientation.z = Ts.getRotation()[2];
      outputPose.pose.orientation.w = Ts.getRotation()[3];  
      pub.publish(outputPose);
    }
    
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  return 0;
}
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <functional>
#include <geometry_msgs/Pose2D.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visp/vpCameraParameters.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <ecn_common/color_detector.h>
#include <calc.h>

using namespace std;

vpColVector q(2);
vpFeaturePoint s;

// Modify parameters here
double lambda = 50;
double l1 = 0.8;
double l2 = 0.6;

void TargetPointMessageCallback(const geometry_msgs::Pose2D message_s)
{
	//this function only sets the received message message_s to the vpFeaturePoint format
	s.set_x(message_s.x);
	s.set_y(message_s.y);
}

void JointStatesMessageCallback(const sensor_msgs::JointState message_s)
{
	//this function only sets the received message message_s to the vpFeaturePoint format
	q[0] = message_s.position[0];
	q[1] = message_s.position[1];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reference_command_node");

	s.set_Z(1);

	ros::NodeHandle n;

	ros::Publisher velocity_pub = n.advertise<sensor_msgs::JointState>("desired_joint_v", 1000);

	ros::Subscriber cible_position_sub = n.subscribe("target_position", 1000, TargetPointMessageCallback);
	ros::Subscriber joint_states_sub = n.subscribe("joint_states", 1000, JointStatesMessageCallback);

	tf::TransformListener listener;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		auto L = s.interaction(); //L is the interaction matrix

		sensor_msgs::JointState targetJointState; //this is what we will publish

		vpMatrix Rbc = GetRbc(listener);

		//////////////////////////////////////////////////////////////////////
		// here we have our computations to get the right values for qPoint //
		//////////////////////////////////////////////////////////////////////
		vpMatrix sVector(s.get_s());

		auto sPoint = -lambda * sVector;

		auto J = GetJac(q[0], q[1], l1, l2);

		auto finalMatrix = L * Rbc * J;

		auto qPoint = finalMatrix.pseudoInverse() * sPoint;

		targetJointState.name.push_back("Vx");
		targetJointState.name.push_back("Vy");
		targetJointState.velocity.push_back(qPoint[0][0]);
		targetJointState.velocity.push_back(qPoint[1][0]);
		velocity_pub.publish(targetJointState);

		cout << "target speed: " << qPoint << endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

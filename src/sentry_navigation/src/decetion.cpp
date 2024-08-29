#include <ros/ros.h>
#include "decetion.hpp"
int main(int argc, char *argv[]) {
	ros::init(argc, argv, "rmul_decision");


	ros::NodeHandle nh;
	DecisionNode node;
	auto timer = nh.createTimer(
	    ros::Duration(0.1), [&](auto) { node.start(); }, false);

	ros::spin();
}
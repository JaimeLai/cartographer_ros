#include "cartographer_ros/CltoolsBridge.h"
#include "msg/TLmwProtocol.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"

CltoolsBridge::~CltoolsBridge()
{

}

CltoolsBridge::CltoolsBridge()
{

}

s32 CltoolsBridge::init()
{
    auto ret = m_sub_lidar.init(TLmwTopicProtocol::lidar(), std::bind(&CltoolsBridge::lidarCb, this, std::placeholders::_1));
    
    if(ret != 0)
	{
		LLOG(LFONT_RED("雷达初始化有误[%]"), ret);
		return ret;
	}

    ret = m_sub_chassis.init(TLmwTopicProtocol::chassis(), std::bind(&CltoolsBridge::chassisCb, this, std::placeholders::_1));
    
    if(ret != 0)
	{
		LLOG(LFONT_RED("底盘始化有误[%]"), ret);
		return ret;
	}

	m_lidar_pub = m_node_handle.advertise<sensor_msgs::LaserScan>("/scan", 50);
	m_imu_pub = m_node_handle.advertise<nav_msgs::Odometry>("/odom", 50);

	LLOG("初始化完成");

    return 0;
}

void CltoolsBridge::lidarCb(TLmwLidar &&msg)
{
	// tf发布
	// geometry_msgs::TransformStamped lidar_trans;
	// tf2_ros::TransformBroadcaster broadcaster_lidar;

	// lidar_trans.header.stamp = ros::Time::now();
	// lidar_trans.header.frame_id = "lidar";
	// lidar_trans.child_frame_id = "base_link";

	// lidar_trans.transform.translation.x = 0;
	// lidar_trans.transform.translation.y = 0;
	// lidar_trans.transform.translation.z = 0.15;
	// lidar_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.01);

	// broadcaster_lidar.sendTransform(lidar_trans);	

	if(msg.units.size() != 360)
	{
		return;
	}

    sensor_msgs::LaserScan out;

	// 设置消息头
	out.header.stamp = ros::Time::now();
	out.header.frame_id = "scan";
	

	// 设置激光扫描的参数
	out.angle_min = 0.01;
	out.angle_max = 6.28;
	out.angle_increment = 0.01;
	out.time_increment = 0.0005;
	out.scan_time = 0.1;

	// 设置激光束的数量和回波数量
	out.ranges.resize(360);
	out.intensities.resize(360);

	for(size_t i = 0; i < out.ranges.size(); ++i)
	{
		out.ranges[i] = msg.units[i].range;
		out.intensities[i] = msg.units[i].signal;

		// out.ranges[i].echoes.emplace_back(msg.units[i].range);
		// out.intensities[i].echoes.emplace_back(msg.units[i].signal);
	}

	m_lidar_pub.publish(out);
}

void CltoolsBridge::chassisCb(TLmwChassis &&msg)
{
	// tf发布
	geometry_msgs::TransformStamped odom_trans;
	auto odom_orientation = tf::createQuaternionMsgFromYaw(0.01);
	tf2_ros::TransformBroadcaster broadcaster;
	auto time = ros::Time::now();

	odom_trans.header.stamp = time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = 0;
	odom_trans.transform.translation.y = 0;
	odom_trans.transform.translation.z = 0.05;
	odom_trans.transform.rotation = odom_orientation;

	broadcaster.sendTransform(odom_trans);

	// 发布消息
	nav_msgs::Odometry out;

	// 设置消息头
	out.header.stamp = time;
	out.header.frame_id = "odom";
	out.child_frame_id = "base_link";

	out.pose.pose.position.x = msg.odm.raw_pose.x;
	out.pose.pose.position.y = msg.odm.raw_pose.y;
	// out.pose.pose.position.z = msg.odm.fuse_pose.z;
	out.pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg.imu[TLmwChassis::YAW]);

	out.twist.twist.angular.z = msg.odm.ang_velocity;
	out.twist.twist.linear.x = msg.odm.lin_velocity;

	m_imu_pub.publish(out);
}


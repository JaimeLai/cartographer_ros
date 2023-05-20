#pragma once

#include "ltools/LLog.h"
#include "ltools/LMutex.h"

#include "lmw/LTopicSub.h"
#include "msg/TLmwChassis.h"
#include "msg/TLmwLidar.h"
#include "ros/publisher.h"
#include "ros/node_handle.h"

class CltoolsBridge
{
public:
    CltoolsBridge();
    ~CltoolsBridge();

    /***************
	 * @brief 初始化
	 * @param in 无
	 * @return 0:成功 其他:失败
	 ***************/
    s32 init();

	/***************
	 * @brief 数据接口
	 * @param in 无
	 * @return 数据
	 ***************/

private:
	/***************
	 * @brief 回调函数(雷达)
	 * @param in msg 数据
	 * @return 无
	 ***************/
	void lidarCb(TLmwLidar &&msg);

	/***************
	 * @brief 回调函数(底盘)
	 * @param in msg 数据
	 * @return 无
	 ***************/
	void chassisCb(TLmwChassis &&msg);

	/***************
	 * @brief 回调函数(深度相机数据)
	 * @param in msg 数据
	 * @return 无
	 ***************/
	// void depthCb(TLmw &&msg);		

  	// LTopicSub<TLmwCamera> m_sub_camera; //相机回调
	// LTopicSub<TLmwCamera> m_sub_depth;	// 深度相机回调
	LTopicSub<TLmwLidar>   m_sub_lidar;	    //雷达回调
	LTopicSub<TLmwChassis> m_sub_chassis;	//底盘回调

	// LMutex m_camera_mtx;	   // 锁
	// LMutex m_depth_mtx;		   // 锁
	// TCameraData m_camera_data; // 相机数据
	// TCameraData m_depth_data;  // 深度相机数据

    ros::Publisher  m_lidar_pub; //数据发布
    ros::Publisher  m_imu_pub;   //数据发布
	ros::NodeHandle m_node_handle; //句柄
};

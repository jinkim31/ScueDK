#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <QDebug>
#include <QIODevice>
#include "../include/serial_line/indexes.h"
#include "../include/serial_line/qnode.hpp"
#include "../include/serial_line/packetgenerator.h"

namespace serial_line
{

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{
}

QNode::~QNode() {
  if(ros::isStarted())
  {

    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"serial_line");
  if ( ! ros::master::check() )
  {
    return false;
  }
  ros::start();
  ros::NodeHandle n;

  robotReadPub = n.advertise<std_msgs::ByteMultiArray>("scue_read",100);
  operatorRobotSetSub = n.subscribe("final_robot_set_data",100, &QNode::operatorMsgCallback,this);
  jointSetSub = n.subscribe("joint_states",100, &QNode::operatorMsgCallback,this);
  scueSetSub = n.subscribe("scue_set",1000, &QNode::scueSetCallback,this);
  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  Q_EMIT rosShutdown();
}

void QNode::publishScueRead()
{
  std_msgs::ByteMultiArray scueReadMsg;
  int size = sizeof(scueRead);
  for(int i=0; i<size; i++)
  {
    scueReadMsg.data.push_back(*((unsigned char*)&scueRead + i));
  }

  robotReadPub.publish(scueReadMsg);
}

void QNode::scueSetCallback(const std_msgs::ByteMultiArray &msg)
{
  if(msg.data.size() < 3) return;
  PacketGenerator::DataChange d;
  d.address = (uint8_t)(msg.data.at(0)) | ((uint8_t)(msg.data.at(1))<<8);
  for(int i=2; i < msg.data.size(); i++)
  {
    d.data.append(msg.data.at(i));
  }

  processScueSet(d);
}

void QNode::operatorMsgCallback(const sensor_msgs::JointState &msg)
{
  //qDebug()<<"joint msg rcved";
  translateJointStateMsg(msg);
}


}

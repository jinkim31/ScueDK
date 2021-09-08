#ifndef _SERIAL_LINE_NODE_
#define _SERIAL_LINE_NODE_

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/ByteMultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include "serial/serial.h"
#include "../../structs/structs.h"
#include "packetgenerator.h"
#include "packettranslator.h"

using namespace std;
using namespace serial_line;
using namespace serial;

class Node
{
public:
  Node(int &argc, char **argv);
  ~Node();
  void run();

private:
  enum CommStatus
  {
    IDLE,
    CONNECTING,
    CONNECTED,
  };

  enum ConversationStatus
  {
    CONVERSATION_IDLE,
    CONVERSATION_REQUEST,
    CONVERSATION_RESPONSE
  };


  inline static Serial serial;
  ConversationStatus conversationStatus;
  CommStatus commStatus;
  PacketGenerator packetGenerator;
  PacketTranslator packetTranslator;
  int timeoutCnt;
  std_msgs::ByteMultiArray scueSetMsg;
  scue::Master scueRead;//?????????
  ros::Publisher robotReadPub;
  ros::Subscriber jointSetSub;
  ros::Subscriber scueSetSub;
  bool moderateComm();
  void init();
  void publishScueRead();
  void scueSetCallback(const std_msgs::ByteMultiArray &msg);
  void translateJointStateMsg(sensor_msgs::JointState msg);
  void processScueSet(PacketGenerator::DataChange d);
  void timerCallback();
  void serialReadCallback();
  static void serialReadThread();
  static bool serialRead;
  static mutex m;
  static queue<PacketTranslator::PacketInfo> readPacketQueue;
};

#endif

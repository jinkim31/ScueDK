#include "../inc/node.h"

bool Node::serialRead;
mutex Node::m;
queue<PacketTranslator::PacketInfo> Node::readPacketQueue;

Node::Node(int &argc, char **argv)
{
  ros::init(argc, argv, "serial_line");
  ros::start();
  ros::NodeHandle n;

  robotReadPub = n.advertise<std_msgs::ByteMultiArray>("scue_read",100);
  scueSetSub = n.subscribe("scue_set",1000, &Node::scueSetCallback, this);

  if(argc != 2)
  {
    ROS_FATAL("Port name not provided. Provide port name as rosrun argument.\n E.g. rosrun scuedk serial_line ttyUSB0");
    return;
  }

  //argv[0]: node executable dir, argv[1]: port name
  string portName = argv[1];

  serial.setPort("/dev/"+portName);
  serial.setBaudrate(1000000);

  conversationStatus = CONVERSATION_IDLE;

  thread readThread(this->serialReadThread);
  serialRead = true;
  run();
  serialRead = false;
  readThread.join();
}

Node::~Node()
{

}

void Node::run()
{
  ros::Rate loopRate(1000);

  while ( ros::ok() )
  {
    timerCallback();
    ros::spinOnce();
    loopRate.sleep();
  }
}

void Node::publishScueRead()
{
  std_msgs::ByteMultiArray scueReadMsg;
  int size = sizeof(scueRead);
  for(int i=0; i<size; i++)
  {
    scueReadMsg.data.push_back(*((unsigned char*)&scueRead + i));
  }

  robotReadPub.publish(scueReadMsg);
}

void Node::scueSetCallback(const std_msgs::ByteMultiArray &msg)
{
  if(msg.data.size() < 3) return;
  PacketGenerator::DataChange d;
  d.address = (uint8_t)(msg.data.at(0)) | ((uint8_t)(msg.data.at(1))<<8);
  for(int i=2; i < msg.data.size(); i++)
  {
    d.data.push_back(msg.data.at(i));
  }

  packetGenerator.addDataChange(d);
}

void Node::timerCallback()
{
  if(!moderateComm()) return;

  switch(conversationStatus)
  {
  case CONVERSATION_IDLE:
  {
    conversationStatus = CONVERSATION_REQUEST;
    break;
  }
  case CONVERSATION_REQUEST:
  {
    serial.write(packetGenerator.generatePacket());
    conversationStatus = CONVERSATION_RESPONSE;
    timeoutCnt = 0;
    break;
  }
  case CONVERSATION_RESPONSE:
  {
    if(!readPacketQueue.empty())
    {
      while(!readPacketQueue.empty())
      {
        std_msgs::ByteMultiArray scueReadMsg;
        for(int i=0; i<readPacketQueue.front().parameter.size(); i++)
        {
          scueReadMsg.data.push_back(readPacketQueue.front().parameter[i]);
        }
        robotReadPub.publish(scueReadMsg);
        conversationStatus = CONVERSATION_REQUEST;
        readPacketQueue.pop();
      }
      convPerSec++;
      conversationStatus = CONVERSATION_IDLE;
    }
    else
    {
      if(timeoutCnt++ > 100)
      {
        timeoutPerSec++;
        conversationStatus = CONVERSATION_REQUEST;
     } 
    }
    break;
  }
  }

  if((ros::Time::now() - timeLastDebug).toSec() >= 1.0)
  {
    cout<<"["<<convPerSec<<"Hz Timeouts:"<<timeoutPerSec<<"]"<<endl;
    convPerSec = 0;
    timeoutPerSec = 0;
    timeLastDebug=ros::Time::now();
  }
}

void Node::serialReadThread()
{
  PacketTranslator packetTranslator;
  PacketTranslator::PacketInfo packetInfo;
  unsigned char byteArray[100];
  while(serialRead)
  {
    int size = serial.available();
    if(size != 0)
    {
      serial.read(byteArray, size);
      for(int i=0; i<size; i++)
      {
        if(packetTranslator.pushByte(packetInfo, byteArray[i]))
        {
          m.lock();
          readPacketQueue.push(packetInfo);
          m.unlock();
        }
      }
    }
  }
}

bool Node::moderateComm()
{
  if(!serial.isOpen())
  {
    try
    {
      serial.open();
    }
    catch(exception e)
    {
      cout<<"No such port name."<<endl;
      return false;
    }
    if(serial.isOpen())
    {
      cout<<"Port opened"<<endl;
      timeLastDebug=ros::Time::now();
      convPerSec = 0;
      timeoutPerSec = 0;
    }
    else
    {
      cout<<"Port open failed"<<endl;
      return false;
    }
  }

  return true;
}

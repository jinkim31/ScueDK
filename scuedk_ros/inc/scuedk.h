
/*
 * ScueDK 2.1.1 by Jin Kim 2021 @ Robit
 *
 *
 *
 */


#ifndef SCUEDK_H
#define SCUEDK_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <std_msgs/ByteMultiArray.h>
#include "../../structs/structs.h"

using namespace std;
namespace scue{

template <typename MasterStruct>
class Scue
{
public:
  typedef struct
  {
    int address;
    vector<uint8_t> data;
  }DataChange;

  typedef unsigned char uint8_t;

  Scue(ros::NodeHandle& nodeHandle)
  {
    this->nodeHandle = &nodeHandle;
    setPub = this->nodeHandle->template advertise<std_msgs::ByteMultiArray>("scue_set",100);
    scueReadSub = nodeHandle.subscribe("scue_read",100, &Scue::scueReadCallback, this);
  }

  MasterStruct ref;

  template<class T, class Wildcard>
  void set(T &target, Wildcard value)
  {
    Scue::DataChange d;
    d.address = (uint8_t*)&target-(uint8_t*)&ref;
    if(d.address < 0 || sizeof(ref) < d.address)
    {
      ROS_WARN("[ScueDK] WARNING: method \'set(T &target, Wildcard value)\' is not being used properly. parameter \'target\' must be a member variable of struct \'ref\'. Request ignored.");
      return;
    }

    int dataSize = sizeof(T);
    T castedValue = (T)value;

    uint8_t valueByteArray[64];
    memcpy(valueByteArray, &castedValue, dataSize);
    for(int i=0; i<dataSize ; i++)
    {
      d.data.push_back(valueByteArray[i]);
    }

    dataChangeList.push_back(d);
  }

  template<class T>
  T get(T &target)
  {
    int address = (uint8_t*)&target-(uint8_t*)&ref;
    if(address < 0 || sizeof(ref) < address)
    {
      ROS_WARN("[ScueDK] WARNING: method \'get(T &target)\' is not being used properly. parameter \'target\' must be a member variable of struct \'ref\'. Request ignored.");
      return (T)0;
    }
    return *((T*)((uint8_t*)&data + address));
  }

  void applySet(bool printSummary = false)
  {
    vector<DataChange> uniqueDataChangeList;

    int duplicate = 0;
    for(DataChange d : dataChangeList)
    {
      bool isUnique = true;
      int i;
      for(i=0; i<uniqueDataChangeList.size(); i++)
      {
        if(uniqueDataChangeList[i].address == d.address)
        {
          uniqueDataChangeList[i] = d;
          isUnique = false;
          duplicate++;
        }
      }

      if(isUnique) uniqueDataChangeList.push_back(d);
    }

    if(printSummary)
    {
      cout<<(double)duplicate/dataChangeList.size()*100<<"% reduced."<<std::endl;
      cout<<"| Address | Length | Data"<<std::endl;
      for(DataChange d : uniqueDataChangeList)
      {
        cout<<"| "<<std::setw(7)<<d.address<<" | "<<std::setw(6)<< d.data.size()<<" | "<< std::hex<< std::uppercase;
        for(int byte : d.data) cout<<std::setfill('0') << std::setw(2)<<byte<<" ";
        cout<<endl;
        std::cout.copyfmt(std::ios(NULL));
      }

       std::cout.copyfmt(std::ios(NULL));
    }

    for(DataChange d : uniqueDataChangeList)
    {
      //cout<<"address "<<d.address<<" to ";
      //for(int byte : d.data) cout<<byte<<" ";
      //cout<<endl;

      std_msgs::ByteMultiArray msg;
      msg.data.push_back(d.address & 0xFF);
      msg.data.push_back((d.address >> 8) & 0xFF);
      for(uint8_t byte : d.data) msg.data.push_back(byte);
      setPub.publish(msg);
    }

    dataChangeList.clear();
  }

  virtual ~Scue()
  {
  }

private:
  ros::NodeHandle* nodeHandle;
  vector<DataChange> dataChangeList;
  ros::Publisher setPub;
  ros::Subscriber scueReadSub;
  MasterStruct data;
  void scueReadCallback(const std_msgs::ByteMultiArray &msg)
  {
    if(msg.data.size() == sizeof (data))
    {
      memcpy(&data, msg.data.data(), sizeof (data));
    }
  }
};

}//namespace scue
#endif // SCUEDK_H

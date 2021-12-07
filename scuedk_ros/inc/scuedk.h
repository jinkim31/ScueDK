
/*
 * ScueDK by Jin Kim 2021 @ Robit
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
#include <functional>
#include <std_msgs/ByteMultiArray.h>
#include "../../structs/structs.h"

using namespace std;
namespace scue
{

class Scue
{
public:
    class CycleListener
    {
    private:
        function<void(void)> handler;
    public:
        explicit CycleListener(function<void(void)> handler) : handler(handler)
        {
        }
        void notify()
        {
            handler();
        }
    };
    typedef struct
    {
        int address;
        vector<uint8_t> data;
    }DataChange;
private:
    bool readReady;
    ros::NodeHandle* nodeHandle;
    vector<DataChange> dataChangeList;
    ros::Publisher setPub;
    ros::Subscriber scueReadSub;
    Master data;
    double timePrevCycle;
    double timeDelta;
    vector<shared_ptr<CycleListener>> cycleListeners;

    void scueReadCallback(const std_msgs::ByteMultiArray &msg)
    {
        if(msg.data.size() == sizeof (data))
        {
            memcpy(&data, msg.data.data(), sizeof (data));
            readReady = true;

            // get timeDelta
            double timeNow = ros::Time::now().toSec();
            if(timePrevCycle != 0.0) timeDelta = timeNow - timePrevCycle;
            timePrevCycle = timeNow;

            for(int i=0;i <cycleListeners.size(); i++) cycleListeners[i]->notify();
        }
        else
        {
            ROS_WARN("[ScueDK] Uplink data size(%d bytes) does not match with master struct size(%d bytes). Make sure you're using latest version of ScueDK.\n",msg.data.size(),sizeof (data));
        }
    }
public:
  typedef unsigned char uint8_t;

  Scue(ros::NodeHandle& nodeHandle)
  {
      this->nodeHandle = &nodeHandle;
      setPub = this->nodeHandle->template advertise<std_msgs::ByteMultiArray>("scue_set",100);
      scueReadSub = nodeHandle.subscribe("scue_read",100, &Scue::scueReadCallback, this);
      readReady = false;
      timePrevCycle = 0.0;
      timeDelta = 0.0;
  }

  Master ref;

  template<class T, class Wildcard>
  void set(T &target, Wildcard value)
  {
    Scue::DataChange d;
    d.address = (uint8_t*)&target-(uint8_t*)&ref;
    if(d.address < 0 || sizeof(ref) < d.address)
    {
      ROS_WARN("[ScueDK] Method \'set(T &target, Wildcard value)\' is not being used properly. parameter \'target\' must be a member variable of struct \'ref\'. Request ignored.");
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
      ROS_WARN("[ScueDK] Method \'get(T &target)\' is not being used properly. parameter \'target\' must be a member variable of struct \'ref\'. Request ignored.");
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

  void addCycleListener(shared_ptr<CycleListener> listener)
  {
      cycleListeners.push_back(listener);
  }

  bool isReadReady()
  {
      return readReady;
  }

  double getTimeDelta()
  {
      return timeDelta;
  }
  virtual ~Scue()
  {
  }
};

}//namespace scue
#endif // SCUEDK_H

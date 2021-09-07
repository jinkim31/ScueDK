#ifndef PACKETGENERATOR_H
#define PACKETGENERATOR_H

#include <iostream>
#include <vector>
#include "sensor_msgs/JointState.h"
#include "Util.h"

typedef vector<unsigned char> ByteArray;

namespace serial_line
{

class PacketGenerator
{
public:
  typedef struct
  {
    unsigned int address;
    ByteArray data;
  }DataChange;

  PacketGenerator();
  ~PacketGenerator();
  ByteArray generatePacket();
  void addDataChange(DataChange d);

  template<class T, class T2>
  DataChange createDataChange(void* structPtr, T* targetPtr, T2 value)
  {
    PacketGenerator::DataChange d;
    d.address = (char*)targetPtr-(char*)structPtr;
    //qDebug()<<"address : "<<d.address<<endl;

    int dataSize = sizeof(T);
    T castedValue = (T)value;

    uint8_t valueByteArray[16];
    memcpy(valueByteArray, &castedValue, dataSize);
    for(int i=0; i<dataSize ; i++)
    {
      d.data.push_back(valueByteArray[i]);
    }

    return d;
  }

private:
  vector<DataChange> dataChangeList;
};

}
#endif // PACKETGENERATOR_H

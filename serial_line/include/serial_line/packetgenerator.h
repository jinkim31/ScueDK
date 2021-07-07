#ifndef PACKETGENERATOR_H
#define PACKETGENERATOR_H

#include <QVector>
#include "device.h"
#include "sensor_msgs/JointState.h"
#include "Util.h"

namespace serial_line
{

class PacketGenerator
{
public:
  typedef struct
  {
    unsigned int address;
    QVector<uint8_t> data;
  }DataChange;

  PacketGenerator();
  ~PacketGenerator();
  QByteArray generatePacket();
  void addDataChange(DataChange d);

  template<class T, class T2>
  DataChange createDataChange(void* structPtr, T* targetPtr, T2 value)
  {
    PacketGenerator::DataChange d;
    d.address = (char*)targetPtr-(char*)structPtr;
    qDebug()<<"address : "<<d.address<<endl;

    int dataSize = sizeof(T);
    T castedValue = (T)value;

    uint8_t valueByteArray[16];
    memcpy(valueByteArray, &castedValue, dataSize);
    for(int i=0; i<dataSize ; i++)
    {
      d.data.append(valueByteArray[i]);
    }

    return d;
  }

private:
  QVector<DataChange> dataChangeList;
};

}
#endif // PACKETGENERATOR_H

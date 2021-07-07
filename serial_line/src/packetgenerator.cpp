#include"../include/serial_line/packetgenerator.h"

namespace serial_line
{

PacketGenerator::PacketGenerator()
{
  dataChangeList.clear();
}

PacketGenerator::~PacketGenerator()
{
}

QByteArray PacketGenerator::generatePacket()
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

  cout<<duplicate<<" duplicate(s) filtered out."<<endl;


  QByteArray packet;

  //header
  packet.append(0xff);
  packet.append(0xff);
  packet.append(0xfd);
  packet.append((int)0x00);

  //id
  packet.append(0xfe);

  //length
  packet.append(0x01); //length 1 placeholder. valid value is set by setLength();
  packet.append(0x01); //length 2 placeholder. valid value is set by setLength();

  //instruction
  packet.append(0xde);

  //parameter
  for(DataChange d : uniqueDataChangeList)
  {
    qDebug()<<"generating packet : address : "<<d.address;

    packet.append(d.address &0xff);
    packet.append((d.address>>8) & 0xff);

    packet.append(d.data.size());

    for(uint8_t byte : d.data)
    {
      packet.append(byte);
    }
  }

  //validate, return
  Util::setLength(packet);
  Util::attachCRC(packet);
  dataChangeList.clear();
  qDebug()<<"packet len : "<<packet.size()<<endl;

  return packet;
}

void PacketGenerator:: addDataChange(DataChange d)
{
  dataChangeList.append(d);
}

}

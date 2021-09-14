#include"../inc/packetgenerator.h"
#include <iostream>

namespace serial_line
{

PacketGenerator::PacketGenerator()
{
  dataChangeList.clear();
}

PacketGenerator::~PacketGenerator()
{
}

ByteArray PacketGenerator::generatePacket()
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

  //cout<<duplicate<<" duplicate(s) filtered out."<<endl;


  ByteArray packet;

  //header
  packet.push_back(0xff);
  packet.push_back(0xff);
  packet.push_back(0xfd);
  packet.push_back((int)0x00);

  //id
  packet.push_back(0xfe);

  //length
  packet.push_back(0x01); //length 1 placeholder. valid value is set by setLength();
  packet.push_back(0x01); //length 2 placeholder. valid value is set by setLength();

  //instruction
  packet.push_back(0xde);

  //parameter
  for(DataChange d : uniqueDataChangeList)
  {
    //cout<<"generating packet : address : "<<d.address<<endl;

    packet.push_back(d.address &0xff);
    packet.push_back((d.address>>8) & 0xff);

    packet.push_back(d.data.size());

    for(uint8_t byte : d.data)
    {
      packet.push_back(byte);
    }
  }

  //validate, return
  Util::setLength(packet);
  Util::attachCRC(packet);

  //for(int i=0; i<packet.size(); i++) std::cout<<(unsigned int)packet[i]<<" ";
  //std::cout<<endl;
  dataChangeList.clear();
  //cout<<"packet len : "<<packet.size()<<endl;

  return packet;
}

void PacketGenerator:: addDataChange(DataChange d)
{
  dataChangeList.push_back(d);
}

}

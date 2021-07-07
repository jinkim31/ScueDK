#ifndef PACKETTRANSLATOR_H
#define PACKETTRANSLATOR_H

#define PACKET_BUFFER_SIZE 500

#include <QByteArray>
#include "sensor_msgs/JointState.h"
#include "config.h"
#include <QDebug>
#include "Util.h"

namespace serial_line
{

class PacketTranslator
{

public:
  typedef struct {
    uint8_t id;
    uint8_t instruction;
    uint8_t error;
    vector<unsigned char> parameter;
  } PacketInfo;

  PacketTranslator()
  {
    nominalPacketCnt = 0;
    state = HEADER_1;
    trialCnt = 0;
    expectedCRC = 0;
    actualCRC = 0;
    packetBufferSize = 0;
  }

  bool pushByte(PacketInfo &packetInfo, uint8_t byte)
  {
    switch (state)
    {
    case HEADER_1:
    {
      clearBuffer();
      if (byte == 0xFF)
      {
        bufferPush(0xFF);
        trialCnt++;
        state = HEADER_2;
      }
      break;
    }
    case HEADER_2:
    {
      if (byte == 0xFF)
      {
        bufferPush(0xFF);
        state = HEADER_3;
      }
      else
      {
        failByte = byte;
        state = HEADER_1;
      }
      break;
    }
    case HEADER_3:
    {
      if (byte == 0xFD)
      {
        bufferPush(0xFD);
        state = RESERVED;
      }
      else
      {
        state = HEADER_1;
      }
      break;
    }
    case RESERVED:
    {
      if (byte == 0x00)
      {
        bufferPush(0x00);
        state = ID;
      }
      else
      {
        state = HEADER_1;
      }
      break;
    }
    case ID:
    {
      bufferPush(byte);
      state = LENGTH_1;
      break;
    }
    case LENGTH_1:
    {
      bufferPush(byte);
      state = LENGTH_2;
      break;
    }
    case LENGTH_2:
    {
      bufferPush(byte);
      length = 256 * packetBuffer[6] + packetBuffer[5];
      if(length < 0 ||1000 < length)
      {
        state = HEADER_1;
      }
      else
      {
        state = INSTRUCTION;
      }
      break;
    }
    case INSTRUCTION:
    {
      bufferPush(byte);
      state = PARAMETER;
      break;
    }
    case PARAMETER:
    {
      bufferPush(byte);
      if (packetBufferSize > (length + 4)) state = CRC_1;
      break;
    }
    case CRC_1:
    {
      bufferPush(byte);
      state = CRC_2;
      break;
    }
    case CRC_2:
    {
      bufferPush(byte);

      actualCRC = packetBuffer[packetBufferSize-2] | (packetBuffer[packetBufferSize-1]<<8);
      expectedCRC = Util::update_crc(packetBuffer, packetBufferSize-2);
      state = HEADER_1;

      if(expectedCRC == actualCRC)
      {
        nominalPacketCnt++;

        packetInfo.id = packetBuffer[4];
        packetInfo.instruction = packetBuffer[7];

        if(packetInfo.instruction == 0x55)
        {
          packetInfo.error = packetBuffer[8];

          packetInfo.parameter.clear();

          int i;
          for(i=9; i < packetBufferSize-2 ; i++)
          {
            packetInfo.parameter.push_back(packetBuffer[i]);
          }
        }
        else
        {
          packetInfo.error = 0;

          packetInfo.parameter.clear();

          int i;
          for(i=8; i < packetBufferSize-2 ; i++)
          {
            packetInfo.parameter.push_back(packetBuffer[i]);
          }
        }

        return true;
      }
      break;
    }
    }

    return false;
  }


  virtual ~PacketTranslator(){};
private:
  enum State {
    HEADER_1,
    HEADER_2,
    HEADER_3,
    RESERVED,
    ID,
    LENGTH_1,
    LENGTH_2,
    INSTRUCTION,
    PARAMETER,
    CRC_1,
    CRC_2
  };

  State state;
  int nominalPacketCnt;
  int packetBufferSize;
  uint8_t packetBuffer[PACKET_BUFFER_SIZE];
  int length;
  int trialCnt;
  unsigned short expectedCRC, actualCRC;
  uint8_t failByte = 0;

  void clearBuffer()
  {
    packetBufferSize = 0;
  }
  void bufferPush(uint8_t byte)
  {
    if(packetBufferSize > PACKET_BUFFER_SIZE-2)
    {
      clearBuffer();
      state = HEADER_1;
    }

    packetBuffer[packetBufferSize++] = byte;
  }
};


}
#endif // PACKETTRANSLATOR_H

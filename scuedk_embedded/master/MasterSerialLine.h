/*
 * MobileMasterSerialLine.h
 *
 * Written by Jin kim 2021 @ Robit
 *
 */

#ifndef SERIAL_MASTERSERIALLINE_H_
#define SERIAL_MASTERSERIALLINE_H_

#define PACKET_INSTRUCTION_DATA_INJECTION 0xDE

#include "main.h"
#include "stdbool.h"
#include "packetTranslator.h"
#include "vector"
#include "queue"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class MasterSerialLine
{
public:
	class Slave
	{
	private:
		int id;
		uint8_t* ptr;
		int headAdr, size;
		bool simulatedSlave;

	public:
		Slave(int id, int headAdr, int size, bool simulatedSlave=false) : id(id), headAdr(headAdr), size(size), simulatedSlave(simulatedSlave){}
		~Slave(){}

		bool bears(int globalAdr){return (headAdr <= globalAdr && globalAdr < headAdr+size);}
		bool isSimulated(){return simulatedSlave;}
		int getId(){return id;}
		int getHeadAdr(){return headAdr;}
		int getSize(){return size;}
	};

	typedef struct _GlobalDataChange
	{
		int address;
		vector<uint8_t> data;
	}GlobalDataChange;

	typedef struct _LocalDataChange
	{
		int id;
		int address;
		Slave *slave;
		vector<uint8_t> data;
	}LocalDataChange;

private:

	class CommStateManager
	{
	public:
		enum State
		{
			IDLE,
			REQUEST,
			READ,
			UPLINK
		};

		CommStateManager(State initialState, int updateRate, int timeout): state(initialState), maxTimeoutCnt(timeout/updateRate)//updateRate, timeout in milliseconds.
		{
			timeoutCnt = 0;
			abortState = IDLE;
		}

		void setState(State state, State abortState=IDLE)
		{
			this->state = state;
			timeoutCnt = 0;
		}

		State getState(){return state;}

		void update()
		{
			if(timeoutCnt++ >= maxTimeoutCnt)
			{
				state = abortState;
				timeoutCnt=0;
			}
		}
	private:
		State state;
		State abortState;
		int timeoutCnt;
		int maxTimeoutCnt;
	};

	enum InjectState
	{
		ADDRESS_1,
		ADDRESS_2,
		SIZE,
		VALUE
	};

	uint8_t *masterPtr;
	int updateRate;
	UART_HandleTypeDef *huartPc, *huartSlave;
	CommStateManager* stateManager;
	PacketTranslator packetTranslatorPc, packetTranslatorSlave;
	vector<Slave> slaveList;
	vector<GlobalDataChange> globalDataChangeList;
	vector<LocalDataChange> localDataChangeList;
	queue<vector<uint8_t>> injectionPacketQueue;

	void getGlobalDataChange(vector<uint8_t>& parameter)
	{
		globalDataChangeList.clear();

		InjectState state = ADDRESS_1;

		uint8_t address;
		int size, valueByteIndex = 0;
		uint8_t valueByteArray[16];

		for(uint8_t byte : parameter)
		{
			switch(state)
			{
			case ADDRESS_1:
			{
				address = byte;
				state = ADDRESS_2;
				break;
			}
			case ADDRESS_2:
			{
				address = address | (byte<<8);
				state = SIZE;
				break;
			}
			case SIZE:
			{
				size = byte;
				state = VALUE;
				valueByteIndex = 0;
				break;
			}
			case VALUE:
			{
				valueByteArray[valueByteIndex++] = byte;
				if(valueByteIndex == size)
				{
					GlobalDataChange dataChange;
					dataChange.address = address;
					for(int i=0; i<size ; i++) dataChange.data.push_back(valueByteArray[i]);
					globalDataChangeList.push_back(dataChange);
					printf("injecting to %d\n",address);
					//memcpy((uint8_t*)&dataStructure + address, valueByteArray, size);*
					state = ADDRESS_1;
				}
				break;
			}
			}
		}
	}
	void splitDataChange()
	{
		localDataChangeList.clear();

		for(GlobalDataChange g : globalDataChangeList)
		{
			for(Slave slave : slaveList)
			{
				if(slave.bears(g.address))
				{
					printf("Data change assigned to slave id:%d\n",slave.getId());
					LocalDataChange localDataChange;
					localDataChange.address = g.address - slave.getHeadAdr();
					localDataChange.slave = &slave;
					localDataChange.data = g.data;
					localDataChange.id = slave.getId();

					if(slave.isSimulated())
					{

						memcpy(masterPtr+localDataChange.address,&localDataChange.data[0], localDataChange.data.size());
					}
					else
					{
						localDataChangeList.push_back(localDataChange);
					}
				}
				break;
			}
		}
	}
	void generateInjectionPacket()
	{
		for(Slave slave : slaveList)
		{
			vector<LocalDataChange> slaveSpecificDataChange;

			//search for local changes that describe data change of iterated slave
			for(LocalDataChange localDataChange : localDataChangeList)
			{
				if(localDataChange.id == slave.getId())
				{
					slaveSpecificDataChange.push_back(localDataChange);
				}
			}

			//generate packet

			//header
			vector<uint8_t> packet;
			packet.push_back(0xFF);
			packet.push_back(0xFF);
			packet.push_back(0xFD);
			packet.push_back(0x00);

			//id
			packet.push_back(slave.getId());

			//length
			packet.push_back(0); //length placeholder. valid value is set by setLength()
			packet.push_back(0); //length placeholder. valid value is set by setLength()

			//instruction
			packet.push_back(PACKET_INSTRUCTION_DATA_INJECTION);

			//parameter
			for(LocalDataChange localDataChange : slaveSpecificDataChange)
			{
				packet.push_back(localDataChange.slave->getHeadAdr() & 0xFF);
				packet.push_back((localDataChange.slave->getHeadAdr()>>8) & 0xFF);

				packet.push_back(localDataChange.data.size());
				for(uint8_t valueByte : localDataChange.data)
				{
					packet.push_back(valueByte);
				}
			}

			//validate packet
			Util::setLength(packet);
			Util::attachCRC(packet);

			//save to packet queue
			//TODO : prevent packet queue from taking too much memory and eventually shutting down the system. line below has been commented temporarily
			//injectionPacketQueue.push(packet);
		}
	}

public:
	MasterSerialLine()
	{
		stateManager = new CommStateManager(CommStateManager::IDLE, 1, 1000);
	}
	MasterSerialLine(uint8_t *masterPtr, int updateRate, UART_HandleTypeDef*huartPc, UART_HandleTypeDef *huartSlave): MasterSerialLine()
	{
		this->masterPtr = masterPtr;
		this->updateRate = updateRate;
		this->huartPc = huartPc;
		this->huartSlave = huartSlave;
	}

	void init(uint8_t *masterPtr, int updateRate, UART_HandleTypeDef*huartPc, UART_HandleTypeDef *huartSlave)
	{
		this->masterPtr = masterPtr;
		this->updateRate = updateRate;
		this->huartPc = huartPc;
		this->huartSlave = huartSlave;
	}
	virtual ~MasterSerialLine()
	{
		delete(stateManager);
	}
	void pushPcRx(uint8_t byte)
	{
		//ignore rx unless comm state is in idle;
		if(stateManager->getState() != CommStateManager::IDLE) return;

		//check full packet receive
		if(packetTranslatorPc.pushByte(byte))
		{
			printf("packet received from pc\n");
			switch(packetTranslatorPc.getPacketInfo().instruction)
			{
			case PACKET_INSTRUCTION_DATA_INJECTION:
			{
				//derive global data changes from received packet parameters.
				getGlobalDataChange(packetTranslatorPc.getPacketInfo().parameter);

				//split global data changes into local data changes.
				//simulated slave data changes get filtered out and related variables in master struct are updated immediately.
				splitDataChange();

				//generate packets for slave data injection.
				generateInjectionPacket();

				//set commstate to REQUEST
				//stateManager->setState(CommStateManager::REQUEST);


				break;
			}
			}
		}
	}

	void pushSlaveRx(uint8_t byte)
	{
		if(packetTranslatorSlave.pushByte(byte))
		{
			packetTranslatorSlave.getPacketInfo();
		}
	}
	void addSlave(Slave slave){slaveList.push_back(slave);}
	void update()
	{
		stateManager->update();

		switch(stateManager->getState())
		{
		case CommStateManager::REQUEST:
		{
			if(!injectionPacketQueue.empty())
			{
				HAL_UART_Transmit(huartSlave, &injectionPacketQueue.front()[0], injectionPacketQueue.front().size(), 1000);
				injectionPacketQueue.pop();
				stateManager->setState(CommStateManager::READ);
			}
			else
			{
				stateManager->setState(CommStateManager::IDLE);
			}
			break;
		}
		case CommStateManager::READ:
		{

			break;
		}
		default:
		{

		}
		}
	}
};

#endif

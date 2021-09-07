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

	void getGlobalDataChange(vector<uint8_t>& parameter);
	void splitDataChange();
	void generateInjectionPacket();

public:
	MasterSerialLine(uint8_t *masterPtr, int updateRate, UART_HandleTypeDef*huartPc, UART_HandleTypeDef *huartSlave): masterPtr(masterPtr), updateRate(updateRate), huartPc(huartPc), huartSlave(huartSlave)
	{
		stateManager = new CommStateManager(CommStateManager::IDLE, 1, 1000);
	}
	virtual ~MasterSerialLine()
	{
		delete(stateManager);
	}
	void pushPcRx(uint8_t byte);
	void pushSlaveRx(uint8_t byte);
	void addSlave(Slave slave){slaveList.push_back(slave);}
	void update();
};

#endif

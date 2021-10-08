
/*
 * ScueDK device data Structs
 *
 * Written by Jin kim 2021 @ Robit
 *
 * Change code below and struct.c to fit user's needs.
 *
 * -- !!DISCLAIMERS!! --
 * 1. WRITE IN STANDARD C. IT WON'T BE COMPATIBLE WITH SLAVE SERIAL_LINE LIBRARY(WRITTEN IN C) OTHERWISE.
 * 2. DEFINE STRUCT FOR EACH SLAVE TYPE(motor driver, manipulator etc). DEFINE A MASTER STRUCT THAT HAS SLAVE STRUCTS DEDICATED TO EACH HARDWARE/SIMULATED SLAVE.
 * 3. ENCAPSULATE EVERY VARIABLE INTO A STRUCT. 'STRAY' ONES MIGHT BE HARD TO SPECIFY THEIR ADDRESSES.
 */

#ifndef _DATA_STRUCTURE_H_
#define _DATA_STRUCTURE_H_

#ifdef __cplusplus
extern "C" {
namespace scue {
#else
#include "stdbool.h"
#endif

/*
 * SLAVE STRUCTS
 * S : SET
 * R : READ
 */

typedef struct _MotorController
{
	float trackTargetSpeed;         //S 회전속도 (rad/s)
	float flipperATargetAngle;      //S 플리퍼A 각도 (rad)
	float flipperBTargetAngle;      //S 플리퍼B 각도 (rad)
    double encoderReading;          //R 누적 엔코더 각도. 오버플로우 없도록 주의 (rad)
}MotorController;

typedef struct _Manipulator
{
	float targetPosition[6];        //S 가장 아래부터 6축의 각도 목표값 (rad)
	float gripperTargetCurrent;     //S 가장 아래부터 6축의 각속도 목표값 (rad/s)
    float targetAcceleration[6];    //S 가장 아래부터 6축의 각가속도 목표값 (rad/s^2)
}Manipulator;

typedef struct _MasterTweak
{
	bool initManipulatorTrigger;    //매니퓰레이터 초기화 트리거
	bool initFlipperTrigger;        //플리퍼 초기화 트리거
}MasterTweak;

/*
 * MASTER STRUCT
 */

typedef struct _Master
{
	MotorController motorControllerA;
	MotorController motorControllerB;
	Manipulator manipulator;
	MasterTweak masterTweak;
}Master;

/*
 * INIT FUNCTIONS
 * Implement them in "struct.c".
 */

void initMotorController(MotorController *s);
void initManipulator(Manipulator *s);
void initMasterTweak(MasterTweak *s);
void initMaster(Master *s);

#ifdef __cplusplus
}//namespace scue
}//extern "C"
#endif
#endif

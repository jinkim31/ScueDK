
/*
 * ScueDK device data Structs
 *
 * Written by Jin kim 2021 @ Robit
 *
 * Change code below and struct.c to fit user's needs.
 *
 * -- !!DISCLAIMERS!! --
 * 1. WRITE IN STANDARD C. IT WON'T BE COMPATIBLE WITH SLAVE SERIAL_LINE LIBRARY(WRITTEN IN C) OTHERWISE.
 * 2. DEFINE STRUCT FOR EACH SLAVE TYPE(motor driver, manipulator etc.). DEFINE A "MASTER"(WITH ITS NAME "Master") STRUCT THAT HAS SLAVE STRUCTS DEDICATED TO EACH HARDWARE/SIMULATED SLAVE.
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

/***********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/

enum Tracks
{
    TRACK_R,
    TRACK_L
};

enum Flippers
{
    FLIPPER_FL,
    FLIPPER_FR,
    FLIPPER_BL,
    FLIPPER_BR
};

/***********************************************************************************************************************
 * ABSTRACTION STRUCTS
 * S : SET
 * R : READ
 **********************************************************************************************************************/

typedef struct Pid
{
    float   kP;                         //SR PID kP
    float   kI;                         //SR PID kI
    float   kD;                         //SR PID kD
    float   errorsumLimit;              //SR PID errrosumLimit
    float   target;                     //SO 목표값
    float   actual;                     //RO 실제값
}Pid;

typedef struct Flipper
{
    Pid     posPid;                     //SR 위치 PID
    Pid     velPid;                     //SR 속도 PID
    Pid     curPid;                     //SR 전류 PID
    bool    inverted;                   //SR 위치 반전
}Flipper;



typedef struct Track
{
    Pid     velPid;                     //SR 트랙 속도 PID
    float   encoderReading;             //RO 누적 엔코더 각도. 오버플로우 없도록 주의 (rad)
}Track;

/***********************************************************************************************************************
 * SLAVE STRUCTS
 * S : SET
 * R : READ
 **********************************************************************************************************************/

typedef struct FlipperController        //[hardware slave] flipper & track motor controller
{
    Flipper flippers[4];                //SR 플리퍼 4개
    bool initTrigger;                  //초기화 트리거
}FlipperController;

typedef struct TrackController
{
    Track tracks[2];                    //SR 트랙 2개
    bool initTrigger;                  //초기화 트리거
}TrackController;

typedef struct Manipulator              //[simulated slave] manipulator
{
    float   targetPosition[6];          //SO 가장 아래축부터 6축의 각도 목표값 (rad)
    float   targetAcceleration[6];      //SO 가장 아래축부터 6축의 각가속도 목표값 (rad/s^2)
    float   gripperTargetCurrent;       //SO 그리퍼 전류제어 목표값(A)
}Manipulator;

typedef struct MasterTweak              //[simulated slave] option tweaks and init triggers for master board
{
	bool    initManipulatorTrigger;     //SR 매니퓰레이터 초기화 트리거
	bool    initFlipperTrigger;         //SR 플리퍼 초기화 트리거
	float boardVoltage;
	float motorVoltage;
}MasterTweak;

/***********************************************************************************************************************
 * MASTER STRUCT
 **********************************************************************************************************************/

typedef struct Master
{
	FlipperController flipperController;
    TrackController trackController;
	Manipulator manipulator;
	MasterTweak masterTweak;
}Master;

/***********************************************************************************************************************
 * INIT FUNCTIONS
 * Implement them in "struct.c".
 **********************************************************************************************************************/

void initPid(Pid* pid);
void initFlipperController(FlipperController* s);
void initTrackController(TrackController* s);
void initManipulator(Manipulator *s);
void initMasterTweak(MasterTweak *s);
void initMaster(Master *s);

#ifdef __cplusplus
}//namespace scue
}//extern "C"
#endif
#endif

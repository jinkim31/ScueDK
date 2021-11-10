#include "structs.h"

/***********************************************************************************************************************
 * Implement init functions here.
 **********************************************************************************************************************/

void initPID(PID *pid)
{
    pid->kD = 0.0f;
    pid->kI = 0.0f;
    pid->kD = 0.0f;
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->actual = 0.0f;
}

void initMotorController(MotorController *s)
{
    for(int i=0; i<2; i++)
    {
        Flipper* f = &s->flippers[i];
        initPID(&f->posPid);
        initPID(&f->velPid);
        initPID(&f->curPid);
        f->inverted = false;
    }

    Track* t = &s->track;
    initPID(&t->velPid);
    t->encoderReading = 0.0f;
}

void initManipulator(Manipulator* s)
{
	for(int i=0; i<6; i++)
	{
		s->targetPosition[i] = 0.0f;
		s->targetAcceleration[i] = 10.0f;
	}
	s->gripperTargetCurrent = 0.0f;
}

void initMasterTweak(MasterTweak* s)
{
	s->initManipulatorTrigger;
	s->initFlipperTrigger;
}

void initMaster(Master* s)
{
    initMotorController(&s->motorControllerA);
    initMotorController(&s->motorControllerB);
	initManipulator(&s->manipulator);
	initMasterTweak(&s->masterTweak);
}


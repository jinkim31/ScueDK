#include "structs.h"

/***********************************************************************************************************************
 * Implement init functions here.
 **********************************************************************************************************************/

void initPid(Pid *pid)
{
    pid->kD = 0.0f;
    pid->kI = 0.0f;
    pid->kD = 0.0f;
    pid->errorsumLimit = 0.0f;
    pid->target = 0.0f;
    pid->actual = 0.0f;
}

void initFlipperController(FlipperController *s)
{
    for(int i=0; i<4; i++)
    {
        Flipper* f = &s->flippers[i];
        initPid(&f->posPid);
        initPid(&f->velPid);
        initPid(&f->curPid);
        f->inverted = false;
    }
    s->initTrigger = false;
    s->SemiAuto = false;
}

void initTrackController(TrackController *s)
{
    for(int i=0; i<2; i++)
    {
        Track *t = &s->tracks[i];
        initPid(&t->velPid);
        t->encoderReading = 0.0f;
    }
    s->initTrigger = false;
}

void initManipulator(Manipulator* s)
{
	for(int i=0; i<6; i++)
	{
		s->targetPosition[i] = 0.0f;
		s->targetAcceleration[i] = 10.0f;
	}
	s->gripperTargetCurrent = 0.0f;

	s->targetPosition[1] = -30.0f;
	s->targetPosition[2] = 30.0f;
}

void initMasterTweak(MasterTweak* s)
{
	s->initManipulatorTrigger;
	s->initFlipperTrigger;
	s->boardVoltage = 0.0f;
	s->motorVoltage = 0.0f;
}

void initMaster(Master* s)
{
    initFlipperController(&s->flipperController);
    initTrackController(&s->trackController);
	initManipulator(&s->manipulator);
	initMasterTweak(&s->masterTweak);
}


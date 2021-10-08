#include "structs.h"

/*
 * Implement init functions here.
 */

void initMotorController(MotorController *s)
{
    s->trackTargetSpeed=0.0;
    s->flipperATargetAngle=0.0;
    s->flipperBTargetAngle=0.0;
    s->encoderReading=0.0;
}

void initManipulator(Manipulator* s)
{
	for(int i=0; i<6; i++)
	{
		s->targetPosition[i] = 0.0;
		s->targetAcceleration[i] = 10.0;
	}
	s->gripperTargetCurrent = 0;
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

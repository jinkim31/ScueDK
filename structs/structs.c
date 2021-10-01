#include "structs.h"

/*
 * Implement init functions here.
 */

void initSlave1(Slave1* s)
{
	s->a=0;
	s->b=0;
    s->c=0;
}

void initSlave2(Slave2* s)
{
	s->d=0;
	s->e=0;
	s->f=0;
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
	initSlave1(&s->slave1);
	initSlave2(&s->slave2);
	initManipulator(&s->manipulator);
	initMasterTweak(&s->masterTweak);
}

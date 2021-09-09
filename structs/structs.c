#include "structs.h"

/*
 * Implement init functions here.
 */

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
		s->targetAngle[i] = 0.0;
	}
}

void initMasterTweak(MasterTweak* s)
{
	s->initTrigger=false;
}

void initMaster(Master* s)
{
	initSlave1(&s->slave1);
	initSlave2(&s->slave2);
	initManipulator(&s->manipulator);
	initMasterTweak(&s->masterTweak);
}

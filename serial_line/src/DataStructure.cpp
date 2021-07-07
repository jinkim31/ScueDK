#include "../include/serial_line/DataStructure.h"

/*
 * DataStructure 1.2.1
 */

FlipperDataStructure createFlipperDataStructure()
{
	FlipperDataStructure s;
	s.initStartTrigger = false;
	s.initCompleted = false;
	s.targetSpeed = 0;
	s.actualSpeed = 0;
  s.targetPosition = 0;
	s.actualPosition = 0;
	s.speedKP = 0;
	s.speedKI = 0;
	s.speedKD = 0;
	s.currentKP = 0;
	s.currentKI = 0;
	s.currentKD = 0;
	s.positionKP = 0;
	s.positionKI = 0;
	s.positionKD = 0;
	s.currentLimit = 0;
	s.actualCurrent = 0;

	return s;
}

TrackDataStructure createTrackDataStructure()
{
	TrackDataStructure s;
	s.targetSpeed = 0;
	s.actualSpeed = 0;
	s.speedKP = 0;
	s.speedKI = 0;
	s.speedKD = 0;

	return s;
}

ManipulatorDataStructure createManiplatorStructure()
{
	ManipulatorDataStructure s;

	s.initCompleted = false;
	s.initStartTrigger = false;

	for(int i=0; i<6; i++)
	{
		s.targetPosition[i] = 0;
		s.actualPosition[i] = 0;
		s.actualPWM[i] = 0;
		s.actualVelocity[i] = 0;
		s.actualCurrent[i] = 0;
	}

	s.gripperTargetCurrent = 0;
	s.gripperActualCurrent = 0;

	return s;
}

MobileMasterDataStructure createMobileMasterStructure()
{
	MobileMasterDataStructure s;
	s.baseRelay = true;
	s.manipulatorRelay = true;
	s.initCompleted = false;
	s.initStartTrigger = false;
	s.flipperFL = createFlipperDataStructure();
	s.flipperFR = createFlipperDataStructure();
	s.flipperBL = createFlipperDataStructure();
	s.flipperBR = createFlipperDataStructure();
	s.trackL = createTrackDataStructure();
	s.trackR = createTrackDataStructure();
	s.manipulator = createManiplatorStructure();

	return s;
}

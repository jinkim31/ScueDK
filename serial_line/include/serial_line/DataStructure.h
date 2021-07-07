#ifndef _DATA_STRUCTURE_H_
#define _DATA_STRUCTURE_H_

/*
 * DataStructure 1.2.1
 */

typedef struct _FlipperDataStructure //60
{
	bool initCompleted, initStartTrigger;
	float targetSpeed, actualSpeed, targetPosition, actualPosition;
	float speedKP, speedKI, speedKD, currentKP, currentKI, currentKD, positionKP, positionKI, positionKD;
	float currentLimit, actualCurrent;

}FlipperDataStructure;

typedef struct _TrackDataStructure //28
{
	double targetSpeed, actualSpeed;
	float speedKP, speedKI, speedKD;
}TrackDataStructure;

typedef struct _ManipulatorDataStructure//53
{
	bool initCompleted, initStartTrigger;
	float targetPosition[6];
	float actualPosition[6];
	float actualPWM[6];
	float actualVelocity[6];
	float actualCurrent[6];
	float gripperTargetCurrent, gripperActualCurrent;
}ManipulatorDataStructure;


typedef struct _MobileMasterDataStructure//
{
	bool initCompleted, initStartTrigger;
	bool manipulatorRelay, baseRelay;
	FlipperDataStructure flipperFL, flipperFR, flipperBL, flipperBR;
	TrackDataStructure trackL, trackR;
	ManipulatorDataStructure manipulator;
}MobileMasterDataStructure;

FlipperDataStructure createFlipperDataStructure();
TrackDataStructure createTrackDataStructure();
ManipulatorDataStructure createManiplatorStructure();
MobileMasterDataStructure createMobileMasterStructure();

#endif

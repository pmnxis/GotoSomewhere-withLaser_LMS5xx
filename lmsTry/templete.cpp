#include <Aria.h>

AREXPORT double ArRangeDevice::currentReadingPolar(double startAngle,
	double endAngle,
	double *angle) const
{
	ArPose pose;
	if (myRobot != NULL)
		pose = myRobot->getPose();
	else
	{
		ArLog::log(ArLog::Normal, "ArRangeDevice %s: NULL robot, won't get polar reading correctly", getName());
		pose.setPose(0, 0);
	}
	return myCurrentBuffer.getClosestPolar(startAngle, endAngle,
		pose,
		myMaxRange,
		angle);
}


AREXPORT double ArRobot::checkRangeDevicesCurrentPolar(
	double startAngle, double endAngle, double *angle,
	const ArRangeDevice **rangeDevice,
	bool useLocationDependentDevices) const
{
	double closest = 32000;
	double closeAngle, tempDist, tempAngle;
	std::list<ArRangeDevice *>::const_iterator it;
	ArRangeDevice *device;
	bool foundOne = false;
	const ArRangeDevice *closestRangeDevice = NULL;

	for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
	{
		device = (*it);
		device->lockDevice();
		if (!useLocationDependentDevices && device->isLocationDependent())
		{
			device->unlockDevice();
			continue;
		}
		if (!foundOne ||
			(tempDist = device->currentReadingPolar(startAngle, endAngle,
				&tempAngle)) < closest)
		{
			if (!foundOne)
			{
				closest = device->currentReadingPolar(startAngle, endAngle,
					&closeAngle);
				closestRangeDevice = device;
			}
			else
			{
				closest = tempDist;
				closeAngle = tempAngle;
				closestRangeDevice = device;
			}
			foundOne = true;
		}
		device->unlockDevice();
	}
	if (!foundOne)
		return -1;
	if (angle != NULL)
		*angle = closeAngle;
	if (rangeDevice != NULL)
		*rangeDevice = closestRangeDevice;
	return closest;

}

AREXPORT double ArRangeBuffer::getClosestPolar(double startAngle,
	double endAngle,
	ArPose startPos,
	unsigned int maxRange,
	double *angle) const
{
	return getClosestPolarInList(startAngle, endAngle,
		startPos, maxRange, angle, &myBuffer);
}

AREXPORT double ArRangeBuffer::getClosestPolarInList(
	double startAngle, double endAngle, ArPose startPos,
	unsigned int maxRange, double *angle,
	const std::list<ArPoseWithTime *> *buffer)
{
	double closest;
	bool foundOne = false;
	std::list<ArPoseWithTime *>::const_iterator it;
	ArPoseWithTime *reading;
	double th;
	double closeTh;
	double dist;
	double angle1, angle2;

	startAngle = ArMath::fixAngle(startAngle);
	endAngle = ArMath::fixAngle(endAngle);

	for (it = buffer->begin(); it != buffer->end(); ++it)
	{
		reading = (*it);

		angle1 = startPos.findAngleTo(*reading);
		angle2 = startPos.getTh();
		th = ArMath::subAngle(angle1, angle2);
		if (ArMath::angleBetween(th, startAngle, endAngle))
		{
			if (!foundOne || (dist = reading->findDistanceTo(startPos)) < closest)
			{
				closeTh = th;
				if (!foundOne)
					closest = reading->findDistanceTo(startPos);
				else
					closest = dist;
				foundOne = true;
			}
		}
	}
	if (!foundOne)
		return maxRange;
	if (angle != NULL)
		*angle = closeTh;
	if (closest > maxRange)
		return maxRange;
	else
		return closest;
}
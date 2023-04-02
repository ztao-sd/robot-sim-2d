#include "wmr.h"

void UnicycleWMR::Model::SimStep(const double dT)
{
	lock.lock();
	velocity(0) = velocityLeft + (velocityRight - velocityLeft) * (wheelDist / 2 - centerOffsetY) / wheelDist;
	velocity(1) = (velocityRight - velocityLeft) / wheelDist;
	stateTransition(0, 0) = cos(genState(2));
	stateTransition(0, 1) = -sin(genState(2)) * centerOffsetX;
	stateTransition(1, 0) = sin(genState(2));
	stateTransition(1, 1) = cos(genState(2)) * centerOffsetX;
	stateTransition(2, 0) = 0;
	stateTransition(2, 1) = 1;
	genStateDot = stateTransition * velocity;
	for (int i = 0; i < genState.size(); ++i)
	{
		genState(i) += genStateDot(i) * dT;
	}
	lock.unlock();
}

void UnicycleWMR::Model::SetCtrlInput(const double vl, const double vr)
{
	lock.lock();
	velocityLeft = vl > maxSpeed ? maxSpeed : (vl < -maxSpeed ? -maxSpeed : vl);
	velocityRight = vr > maxSpeed ? maxSpeed : (vr < -maxSpeed ? -maxSpeed : vr);
	lock.unlock();
}

void UnicycleWMR::Model::SetCtrlInput2(const double v, const double w)
{
	lock.lock();
	static double c = (wheelDist / 2 - centerOffsetY) / wheelDist;
	velocityLeft = v - c * wheelDist * w;
	velocityRight = v + (1 - c) * wheelDist * w;
	lock.unlock();
}

UnicycleWMR::Path::Path(double xInit, double xEnd, vector<Vector2d>(*pathGenerator)(double xInit, double xEnd))
{
	pathPoints = pathGenerator(xInit, xEnd);
}

void UnicycleWMR::Controller::ManualControl(UnicycleWMR::Model& robot, float speed)
{
	double vl = 0, vr = 0;
	const double w_max = 2 * speed / robot.wheelDist;
	if (KEY('I')) {
		vl = speed;
		vr = speed;
	}
	else if (KEY('K')) {
		vl = -speed;
		vr = -speed;
	}
	else if (KEY('J')) {
		vl = speed;
		vr = -speed;
	}
	else if (KEY('L')) {
		vl = -speed;
		vr = speed;
	}
	robot.SetCtrlInput(vl, vr);
}
#include "unicycle_sim.h"

void UnicycleRobot::SimStep(double dT)
{
	double v = (velocityLeft + velocityRight) / 2;
	double w = (velocityRight - velocityRight) / wheelDist;
	xDot[0] = v * cos(x[2]);
	xDot[1] = v * sin(x[2]);
	xDot[2] = w;
	for (int i = 0; i < x.size(); ++i)
	{
		x[i] = x[i] + xDot[i] * dT;
	}
	time += dT;
}

void UnicycleRobot::SetCtrlInput(const double vl, const double vr)
{
	velocityLeft = vl > maxSpeed ? maxSpeed : (vl < -maxSpeed ? -maxSpeed : vl);
	velocityRight = vr > maxSpeed ? maxSpeed : (vr < -maxSpeed ? -maxSpeed : vr);
}

void UnicycleRobot::SetCtrlInput2(const double v, const double w)
{
	double vl = (2 * v - wheelDist * w) / 2;
	double vr = (2 * v + wheelDist * w) / 2;
	SetCtrlInput(vl, vr);
}

void RobotSimulation::SetMode(SimMode mode)
{
	this->mode = mode;
}

void RobotSimulation::AddRobot(UnicycleRobot* robot, RobotController* controller)
{
	try
	{
		robotCtrlMap.insert({ std::unique_ptr<UnicycleRobot>(robot), std::unique_ptr<RobotController>(controller) });
	}
	catch (std::bad_alloc)
	{

	}
}

void RobotSimulation::AddObstacle(RoundObstacle* obstacle)
{
	try
	{
		obstacles.push_back(std::unique_ptr<RoundObstacle>(obstacle));
	}
	catch (std::bad_alloc)
	{

	}
}

void RobotSimulation::SimStep(double dT)
{
	for (auto i{ robots.begin() }; i != robots.end(); ++i)
	{
		(*i)->SimStep(dT);
	}
}

void RobotSimulation::SimLoop()
{
	while (true)
	{

	}
}

void Timer::stop()
{
	running = false;
	thread.join();
}

void RobotController::ManualControl(UnicycleRobot& robot, float speed)
{
	double vl = 0, vr = 0;
	const double w_max = 2 * speed / robot.wheelDist;
	if (KEY('I')) {
		vl = speed;
		vr=  speed;
	}
	if (KEY('K')) {
		vl = -speed;
		vr = -speed;
	}
	if (KEY('J')) {
		vl = -speed;
		vr = speed;
	}
	if (KEY('L')) {
		vl = speed;
		vr = -speed;
	}
	robot.SetCtrlInput(vl, vr);
}

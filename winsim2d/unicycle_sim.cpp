#include "unicycle_sim.h"

void Timer::Stop()
{
	if (running)
	{
		if (thread.joinable())
		{
			running = false;
			thread.join();
		}
	}
}

void Timer::SetInterval(const std::chrono::milliseconds& interval)
{
	this->interval = interval;
}

std::chrono::duration<double> Timer::ElapsedMillisecond()
{
	if (this->thread.joinable())
	{
		std::chrono::time_point<std::chrono::system_clock> endTime = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed = endTime - startTime;
		return elapsed;
	}
	return std::chrono::duration<double>();
}

void UnicycleRobot::SimStep(double dT)
{
	lock.lock();
	double v = (velocityLeft + velocityRight) / 2;
	double w = (velocityRight - velocityLeft) / wheelDist;
	xDot[0] = v * cos(x[2]);
	xDot[1] = v * sin(x[2]);
	xDot[2] = w;
	for (int i = 0; i < x.size(); ++i)
	{
		x[i] = x[i] + xDot[i] * dT;
	}
	time += dT;
	lock.unlock();
}

void UnicycleRobot::SetCtrlInput(const double vl, const double vr)
{
	lock.lock();
	velocityLeft = vl > maxSpeed ? maxSpeed : (vl < -maxSpeed ? -maxSpeed : vl);
	velocityRight = vr > maxSpeed ? maxSpeed : (vr < -maxSpeed ? -maxSpeed : vr);
	lock.unlock();
}

void UnicycleRobot::SetCtrlInput2(const double v, const double w)
{
	lock.lock();
	double vl = (2 * v - wheelDist * w) / 2;
	double vr = (2 * v + wheelDist * w) / 2;
	lock.unlock();
}

void RobotController::ManualControl(UnicycleRobot& robot, float speed)
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

void RobotController::AutoControl(UnicycleRobot& robot, float vl, float vr)
{
	robot.SetCtrlInput(vl, vr);
}

void RobotSimulation::AddRobot(UnicycleWMR::Robot* robot)
{
	try
	{
		robots.push_back(unique_ptr<UnicycleWMR::Robot>(robot));
	}
	catch (std::bad_alloc)
	{

	}
}

void RobotSimulation::AddObstacle(Obstacle* obstacle)
{
	try
	{
		obstacles.push_back(unique_ptr<Obstacle>(obstacle));
	}
	catch (std::bad_alloc)
	{

	}
}

void RobotSimulation::SimStep(double dT)
{
	for (auto i{ robots.begin() }; i != robots.end(); ++i)
	{
		(* i)->model.SimStep(dT);
	}
}

void RobotSimulation::ControlStep()
{
	for (auto i{ robots.begin() }; i != robots.end(); ++i)
	{
		(*i)->controller->ManualControl((*i)->model);
	}
}

void RobotSimulation::StartSim()
{
	auto f = [this]()
	{
		this->SimStep(this->simTimer.GetInterval().count());
	};
	simTimer.Start(f, simTimer.GetInterval());
}

void RobotSimulation::StopSim()
{
	simTimer.Stop();
}

void RobotSimulation::StartControl()
{
	auto f = [this]()
	{
		this->ControlStep();
	};
	ctrlTimer.Start(f, ctrlTimer.GetInterval());
}

void RobotSimulation::StopControl()
{
	ctrlTimer.Stop();
}

void RobotSimulation::CleanUp()
{
	robots.clear();
}




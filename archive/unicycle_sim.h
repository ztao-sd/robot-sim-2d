#ifndef UNICYCLE_SIM_H
#define UNICYCLE_SIM_H

#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <map>
#include <d2d1.h>

class Timer
{
private:
	std::thread thread{};
	std::atomic_bool running;
	std::chrono::milliseconds interval;

public:
	Timer() = default;
	~Timer() = default;
	template <typename F>
	void start(F&& callback, const std::chrono::milliseconds& interval);
	void stop();
	void SetInterval(const std::chrono::milliseconds& interval);
	void ElapsedMillisecond();
};

template<typename F>
inline void Timer::start(F&& callback, const std::chrono::milliseconds& interval)
{
	running = true;
	this->interval = interval;
	auto lambda = [=]()
	{
		while (running)
		{
			callback();
			std::this_thread::sleep_for(this->interval);
		}
	};
	this->thread = std::thread(lambda);
}

/*
* Class to simulate a unicycle robot
*/
class UnicycleRobot
{
private:
	// Dynamic params
	double maxSpeed = 30; // DIP per sec
	std::vector<double> x{ 0, 0, 0 }; // x, y, theta about absolute center of rotation
	std::vector<double> xDot{ 0, 0, 0 }; // x_dot, y_dot, theta_dot
	double velocityLeft = 0;
	double velocityRight = 0;
	double time = 0;

public:
	// Geometric params
	double wheelDist{ 121 };
	double collisionWidth;
	double collisionHeight;
	std::vector<double> relCenterRotation{ 0, 0 }; // from upper left corner of rectangle

	// Interface
	UnicycleRobot() = default;
	UnicycleRobot(double wheelDist = 121, double collisionWidth = 140, double collisionHeight = 300, std::vector<double> relCenterRotation = { 30, 70 })
		: wheelDist{ wheelDist }, collisionWidth{ collisionWidth }, collisionHeight{ collisionHeight }, relCenterRotation{ relCenterRotation } {}
	void SetMaxSpeed(double speed) { maxSpeed = abs(speed); }
	const std::vector<double>& State() const { return x; }
	const std::vector<double>& StateDot() const { return xDot; }
	void SimStep(const double dT);
	void SetCtrlInput(const double vl, const double vr);
	void SetCtrlInput2(const double v, const double w);

	// D2D1_POINT_2F GetPosition() const 
	// {
	// 	return D2D1_POINT_2F{ static_cast<float>(x[0]), static_cast<float>(x[1]) };
	// }
	// float GetRotation() const { return static_cast<float>(x[2]); }
	// D2D1_POINT_2F GetRelativeCenterRotation() const
	// {
	// 	return D2D1_POINT_2F{ static_cast<float>(relCenterRotation[0]), static_cast<float>(relCenterRotation[1]) };
	// }
};

struct RoundObstacle
{
	double radius = 50;
	double xPos = 0;
	double yPos = 0;
};

class RobotController
{
#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )
private:
	// Manual ctrl params

public:
	// Interface
	RobotController() = default;
	void ManualControl(UnicycleRobot& robot, float speed = 100.0f);
	void AutoControl();
};

enum class SimMode
{
	Manual,
	Auto
};

class RobotSimulation
{
private:
	Timer simTimer;
	Timer ctrlTimer;
	
	// Dynamics
	double time = 0;

	// Window size
	int winWidth = 640;
	int winHeight = 480;

	// Utility
	SimMode mode = SimMode::Auto;

public:
	// Simulation state
	std::vector<std::unique_ptr<RoundObstacle>> obstacles;
	std::map<std::unique_ptr<UnicycleRobot>, std::unique_ptr<RobotController>> robotCtrlMap;

	RobotSimulation() = default;
	RobotSimulation(int winWidth = 640, int winHeight = 480)
		:winWidth{ winWidth }, winHeight{ winHeight } {}
	void SetMode(SimMode mode);
	void AddRobot(UnicycleRobot* robot, RobotController* controller);
	void AddObstacle(RoundObstacle* obstacle);
	void SimStep(double dT);
	void SimLoop();
	void StartSim();
	void StopSim();
	void StartControl();
	void StopControl();
};




#endif
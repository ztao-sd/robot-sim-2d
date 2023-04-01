#ifndef UNICYCLE_SIM_H
#define UNICYCLE_SIM_H

#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <map>
#include <d2d1.h>
#include <mutex>
#include <iostream>

class Timer
{
private:
	std::thread thread{};
	std::atomic_bool running;
	std::chrono::milliseconds interval{ 10 };
	std::chrono::time_point<std::chrono::system_clock> startTime;

public:
	Timer(int interval = 10) : thread{}, interval { 10 } {}
	//~Timer() = default;
	template <typename F>
	void Start(F&& callback, const std::chrono::milliseconds& interval);
	void Stop();
	void SetInterval(const std::chrono::milliseconds& interval);
	const std::chrono::milliseconds& GetInterval() const { return interval; }
	std::chrono::duration<double> ElapsedMillisecond();
};

template<typename F>
inline void Timer::Start(F&& callback, const std::chrono::milliseconds& interval)
{
	if (!running)
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
		startTime = std::chrono::system_clock::now();

		this->thread = std::thread(lambda);
	}
}

/*
* Class to simulate a unicycle robot
*/
class UnicycleRobot
{
private:
	// Dynamic params
	double maxSpeed = 200; // DIP per sec
	std::vector<double> x{ 0, 0, 0 }; // x, y, theta about absolute center of rotation
	std::vector<double> xDot{ 0, 0, 0 }; // x_dot, y_dot, theta_dot
	double velocityLeft = 0;
	double velocityRight = 0;
	double time = 0;
	std::mutex lock{};

public:
	// Geometric params
	double wheelDist{ 121 };
	double collisionWidth;
	double collisionHeight;
	std::vector<double> relCenterRotation{ 0, 0 }; // from upper left corner of rectangle

	// Interface
	UnicycleRobot(std::vector<double> initState = {300, 100, 50}, double wheelDist = 70, double collisionWidth = 80, double collisionHeight = 150, std::vector<double> relCenterRotation = { -15, -40 })
		: x{initState}, wheelDist{ wheelDist }, collisionWidth{ collisionWidth }, collisionHeight{ collisionHeight }, relCenterRotation{ relCenterRotation } {}
	void SetMaxSpeed(double speed) { maxSpeed = abs(speed); }
	const std::vector<double>& State() const { return x;}
	std::vector<double>& State() { return x; }
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


struct RectObstacle
{
	double width = 100;
	double height = 100;
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
	void AutoControl(UnicycleRobot& robot, float vl = 0.1, float vr = 0.1);
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

public:
	// Simulation state
	std::vector<std::unique_ptr<RoundObstacle>> obstacles;
	std::vector<std::unique_ptr<UnicycleRobot>> robots;
	std::vector <std::unique_ptr<RobotController>> controllers;
	std::map<UnicycleRobot*, RobotController*> robotCtrlMap;

	RobotSimulation(int winWidth = 640, int winHeight = 480, int simInterval = 10, int ctrlInterval = 20)
		:winWidth{ winWidth }, winHeight{ winHeight }, simTimer{ simInterval }, ctrlTimer{ ctrlInterval } {}
	void AddRobot(UnicycleRobot* robot, RobotController* controller);
	void AddObstacle(RoundObstacle* obstacle);
	void SimStep(double dT);
	void ControlStep();
	void StartSim();
	void StopSim();
	void StartControl();
	void StopControl();
	void CleanUp();
};




#endif
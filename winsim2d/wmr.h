#ifndef WMR_H
#define WMR_H

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <mutex>
#include <vector>

namespace UnicycleWMR
{
	// Two-wheeled differential mobile robot

	using Eigen::Matrix;
	using Eigen::Vector2d;
	using Eigen::Vector3d;
	using Eigen::Vector4d;
	using std::vector;

	class Model
	{
	private:
		// Kinematics
		Vector3d genState{ {0.0, 0.0, 0.0} };		// xc, yc, theta
		Vector3d genStateDot{ {0.0, 0.0, 0.0} };
		Vector2d velocity{ {0.0, 0.0} };			// v, w
		Matrix<double, 3, 2> stateTransition{};
		double velocityLeft = 0;
		double velocityRight = 0;
		double time = 0;
		double maxSpeed = 200;					// DIP per sec

		// Geometric parameters
		double wheelDist = 70;
		double centerOffsetX = 0;
		double centerOffsetY = 0;
		double rectCollisionLeft = 40;
		double rectCollisionTop = 40;
		double rectCollisionRight = 40;
		double rectCollisionBottom = 110;
		
		// MISC
		std::mutex lock{};

	public:
		Model(Vector3d initState = { 300, 100, 50 }, double maxSpeed = 200, double wheelDist = 70, double centerOffsetX = 0, double centerOffsetY = 0,
			double rectCollisionLeft = 40, double rectCollisionTop = 40, double rectCollisionRight = 40, double rectCollisionBottom = 110)
			:genState{ initState }, maxSpeed{ maxSpeed }, wheelDist{ wheelDist }, centerOffsetX{ centerOffsetX }, centerOffsetY{ centerOffsetY },
			rectCollisionLeft{ rectCollisionLeft }, rectCollisionTop{ rectCollisionTop }, rectCollisionRight{ rectCollisionRight }, rectCollisionBottom{ rectCollisionBottom } {}
		void SetMaxSpeed(double speed) { maxSpeed = abs(speed); }
		const Vector3d& State() const { return genState; }
		Vector3d& State() { return genState; }
		const Vector3d& StateDot() const { return genStateDot; }
		void SimStep(const double dT);
		void SetCtrlInput(const double vl, const double vr);
		void SetCtrlInput2(const double v, const double w);
	};

	struct Path
	{
		vector<Vector2d> pathPoints; // x, y
		Path(vector<Vector2d> pathPoints) : pathPoints{ pathPoints } {}
		Path(double xInit, double xEnd, vector<Vector2d>(*pathGenerator)(double xInit, double xEnd));
	};

	struct Trajectory
	{
		Path path;
		vector<Vector4d> genStateRef;
		vector<Vector4d> genStateDotRef;
	};

	class PathPlanner
	{

	};

	class Controller
	{

	};

	struct Robot
	{
		Model model;
		
	};
}


#endif // !WMR_H




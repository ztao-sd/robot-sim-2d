#ifndef WMR_H
#define WMR_H

#include "hnswlib/hnswlib.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <iostream>
#include <vector>
#include <list>
#include <utility>
#include <d2d1.h>
#include <chrono>
#include <string>
#include <memory>

#define S_PI 3.14159265358979323846

template <typename Shape>
class ShapeList;

class ObstacleModel;

namespace RRT
{
	using namespace boost;
	using Eigen::MatrixXd;
	using Eigen::VectorXd;
	using Eigen::Vector2d;
	using Eigen::Vector3d;
	using Eigen::seq;
	using std::list;
	using std::vector;
	using std::pair;
	using std::tie;

	//template <class PredecessorMap>
	//class record_predecessors : public dijkstra_visitor<>
	//{
	//public:
	//	record_predecessors(PredecessorMap p)
	//		: m_predecessor(p) { }

	//	template <class Edge, class Graph>
	//	void edge_relaxed(Edge e, Graph& g) {
	//		// set the parent of the target(e) to source(e)
	//		put(m_predecessor, target(e, g), source(e, g));
	//	}
	//protected:
	//	PredecessorMap m_predecessor;
	//};

	//template <class PredecessorMap>
	//record_predecessors<PredecessorMap>
	//	make_predecessor_recorder(PredecessorMap p) {
	//	return record_predecessors<PredecessorMap>(p);
	//}

	struct DataPoint1D
	{
		double time;
		double value;
	};

	struct DataPoint2D
	{
		double time;
		double x;
		double y;
	};

	struct Position2D
	{
		double x;
		double y;
	};

	// Bundled properties
	struct Junction
	{
		Vector3d state;
		Vector3d stateDot;
		double time;
		int index;
	};

	struct Link
	{
		double distance;
		list<Vector3d> states;
		list<Vector3d> stateDots;
		list<double> times;
	};

	class Tree
	{

	public:

		typedef adjacency_list<listS, vecS, undirectedS, Junction, Link> UndirectedGraph;
		typedef graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
		typedef graph_traits<UndirectedGraph>::edge_descriptor Edge;
		typedef graph_traits<UndirectedGraph>::vertex_iterator VertexIterator;
		typedef graph_traits<UndirectedGraph>::edge_iterator EdgeIterator;
		typedef graph_traits<UndirectedGraph>::out_edge_iterator OutEdgeIterator;
		typedef property_map<UndirectedGraph, vertex_index_t>::type IndexMap;

		double horizontalRange;
		double verticalRange;
		UndirectedGraph tree;
		Vertex root;
		Vertex destination;
		double delta;
		vector<Vector2d> shortestPath;

		// Collision models
		ShapeList<ObstacleModel>* obstacles;
		double collisionPadding = 55;

		// ANN
		int dim = 2;
		int maxElements = 6000;
		int M = 8;
		int efConstruction = 20;
		hnswlib::L2Space space;
		hnswlib::HierarchicalNSW<float>* algHNSW;
		float* data = new float[dim * maxElements];
		int idx = 0;
		vector<Vertex> vertex_vec = vector<Vertex>(maxElements);

		Tree(Vector2d initPoint, double horizontalRange, double verticalRange, double delta);
		~Tree();
		Vertex NearestNeighbour(Vector2d point);
		Vector2d NewPoint(Vector2d randPoint, Vector2d nearestPoint, double delta);
		Vertex Extend(Vector2d point);
		bool FindTarget(Vertex vertex, Vector2d targetPoint, double distance);
		void ExploreN(int iteration);
		void ExploreTillTarget(Vector2d targetPoint, int explorationSteps);
		vector<Edge> DijkstraShortestPath(Vertex home, Vertex destination);
		const vector<Vector2d>& ShortestPath() const { return shortestPath; }
		const UndirectedGraph& Graph() const { return tree; }
		const Vertex& Home() const { return root; }
		const Vertex& Target() const { return destination; }
		bool SegmentIntersectionDetection(Vector2d source0, Vector2d target0, Vector2d source1, Vector2d target1);
		bool CheckCollisionRect(Vector2d source, Vector2d target, D2D1_RECT_F rect, Vector2d center, double rotation);
		bool CheckCollisionEllipse(Vector2d source, Vector2d target, D2D1_ELLIPSE ellipse);
		Vector2d Rotate2DVector(Eigen::Vector2d vector, double rotation);
		bool CheckBetween(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d p0);
		Vector2d VectorToLine2D(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d p0);
	};


	enum class EndCondition
	{
		Natural,
		Clamped,
		NotaKnot,
		Second,
		FirstSecond
	};

	class CubicSplineEndCondition
	{
	public:
		double firstDerivative = 0;
		double secondDerivative = 0;
		EndCondition endCondition;

		CubicSplineEndCondition(EndCondition endCondition = EndCondition::Natural) : endCondition{ endCondition } {}
		CubicSplineEndCondition(double firstDerivative, double secondDerivative, EndCondition endCondition = EndCondition::FirstSecond) :
			firstDerivative{ firstDerivative }, secondDerivative{ secondDerivative }, endCondition {
			endCondition
		} {}
	};

	class CubicSpline
	{
	public:
		vector<DataPoint1D> dataPoints;
		vector<vector<double>> coefficient;
		CubicSplineEndCondition startCondition;
		CubicSplineEndCondition endCondition;

		double slope(vector<DataPoint1D> dataPoints, int i, int j);
		CubicSpline() = default;
		CubicSpline(vector<DataPoint1D> dataPoints, CubicSplineEndCondition startCondition, CubicSplineEndCondition endCondition);
		int TimeIndex(double t);
		double operator()(double t);
		double FirstDerivative(double t);
		double SecondDerivative(double t);
	};

	class ParametricCurve2D
	{
	public:
		vector<DataPoint2D> dataPoints;
		CubicSpline splineX;
		CubicSpline splineY;
		double startTime;
		double endTime;
		double edgeTime;
		double maxVelocity;

		ParametricCurve2D(vector<Vector2d> path, double w0, double wn, double maxVelocity, double rampDistance);
		Vector3d operator()(double t);
		Vector3d Velocity(double t);
		double TrapezoidVelocity(double t);
		double TrapezoidTime(double distance);

	};
}

namespace UnicycleWMR
{
	
	// Two-wheeled differential mobile robot

	using Eigen::Matrix;
	using Eigen::Matrix3d;
	using Eigen::Vector2d;
	using Eigen::Vector3d;
	using Eigen::Vector4d;
	using Eigen::seq;
	using std::vector;
	using std::unique_ptr;

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
		
		// MISC
		std::mutex lock{};

	public:
		// Geometric parameters
		double maxSpeed = 2000;					// DIP per sec
		double wheelDist = 70;
		double centerOffsetX = 0;
		double centerOffsetY = 0;
		double rectCollisionLeft = 40;
		double rectCollisionFront = 110;
		double rectCollisionRight = 40;
		double rectCollisionBack = 40;

		Model(Vector3d initState = { 300, 100, 50 }, double maxSpeed = 200, double wheelDist = 35, double centerOffsetX = 0, double centerOffsetY = 0,
			double rectCollisionLeft = 20, double rectCollisionFront = 55, double rectCollisionRight = 20, double rectCollisionBack = 20)
			:genState{ initState }, maxSpeed{ maxSpeed }, wheelDist{ wheelDist }, centerOffsetX{ centerOffsetX }, centerOffsetY{ centerOffsetY },
			rectCollisionLeft{ rectCollisionLeft }, rectCollisionFront{ rectCollisionFront }, rectCollisionRight{ rectCollisionRight }, rectCollisionBack{ rectCollisionBack } {}
		void SetMaxSpeed(double speed) { maxSpeed = abs(speed); }
		const Vector3d& State() const { return genState; }
		Vector3d& State() { return genState; }
		const Vector3d& StateDot() const { return genStateDot; }
		const double& Time() const { return time; }
		double& Time() { return time; }
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
	public:
		unique_ptr<RRT::Tree> tree;
		unique_ptr<RRT::ParametricCurve2D> pathCurve;
		double horizontalRange;
		double verticalRange;

		PathPlanner(double horizontalRange, double verticalRange) : horizontalRange{ horizontalRange }, verticalRange{ verticalRange } {}
		bool PlanPath(Vector3d home, Vector3d target, ShapeList<ObstacleModel>* obstacles, double explorationDelta = 25, int explorationSteps = 5000,
			double maxVelocity = 50, double rampDistance = 100);
	};

	template <typename T>
	struct IAuxiliaryFunction
	{
		virtual T operator()(T time, T xe, T ye, T vd, T wd) = 0;
		virtual T FirstDerivative(T time, T xe, T ye, T xeDot, T yeDot, T vd, T wd) = 0;
		virtual T H(T time, T xe, T ye) = 0;
		virtual T HDot(T time, T xe, T ye, T xeDot, T yeDot) = 0;
		virtual T Alpha() = 0;
		virtual T AlphaDot() = 0;
		virtual void Calculate(T time, T xe, T ye, T xeDot, T yeDot, T vd, T wd) = 0;
		virtual void Reset() = 0;
	};

	class Alpha1 : public IAuxiliaryFunction<double>
	{
	private:
		double h0 = 8;
		double a = 1.0;
		double b = 1.0;
		double c = 1.0;

		double rho = 1.0;
		double rhoDot = 0.0;
		double time = 0.0;
		double h = 0.0;
		double hDot = 0.0;
		double alpha = 0.0;
		double alphaDot = 0.0;

	public:
		double operator()(double time, double xe, double ye, double vd, double wd);
		double FirstDerivative(double time,double xe,double ye,double xeDot,double yeDot,double vd,double wd);
		double H(double time, double xe, double ye);
		double HDot(double time, double xe, double ye, double xeDot, double yeDot);
		void Calculate(double time, double xe, double ye, double xeDot, double yeDot, double vd, double wd);
		double Alpha() { return alpha; }
		double AlphaDot() { return alphaDot; }
		void Reset();
	};

	class Controller
	{
	#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )
	private:
		// Unified parameters
		double k0 = 0.2;
		double k1 = 10.0;
		double k2 = 20.0;
		IAuxiliaryFunction<double>* alpha = new Alpha1();
		Vector3d RotationTransform(Vector3d state, const double theta);

	public:
		// Slow down time
		bool slowTimeFlag = true;
		UnicycleWMR::Model* robot;
		double beta = 1.0;
		double realTime = 0.0;
		double pathTime = 0.0;

		// Interface
		Controller() = default;
		Controller(double k0, double k1, double k2, UnicycleWMR::Model* robot) : k0{ k0 }, k1{ k1 }, k2{ k2 }, robot{ robot } { alpha = new Alpha1(); }
		void ManualControl(UnicycleWMR::Model& robot, float speed = 100.0f);
		void UnifiedControl(UnicycleWMR::Model& robot, RRT::ParametricCurve2D& desiredCurve);
		void DirectControl(UnicycleWMR::Model& robot, RRT::ParametricCurve2D& desiredCurve);
		void ApplyVelocityConstraint(RRT::ParametricCurve2D& desiredCurve, double& time, double& vd, double& wd, Vector3d& desiredState, Vector3d& desiredStateDot);
		void Reset(UnicycleWMR::Model& robot);
	};

	struct Robot
	{
		Model model;
		PathPlanner* pathPlanner;
		Controller* controller;

		Robot(Vector3d initState = { 300, 100, 50 }, double maxSpeed = 200, double wheelDist = 35, double centerOffsetX = 0, double centerOffsetY = 0,
			double rectCollisionLeft = 20, double rectCollisionFront = 55, double rectCollisionRight = 20, double rectCollisionBack = 10, 
			double horizontalRange = 1280, double verticalRange = 720)
			:model{ initState, maxSpeed, wheelDist, centerOffsetX, centerOffsetY, rectCollisionLeft, rectCollisionFront, rectCollisionRight, rectCollisionBack }, 
			pathPlanner{ new PathPlanner{horizontalRange, verticalRange} }, controller{} {}
	};
}




#endif // !WMR_H




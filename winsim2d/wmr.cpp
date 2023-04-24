#include "wmr.h"
#include "winsim2d.h"
#include "drawing_window.h"

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
	time += dT;
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
	double vl = v - c * wheelDist * w;
	double vr = v + (1 - c) * wheelDist * w;
	velocityLeft = vl > maxSpeed ? maxSpeed : (vl < -maxSpeed ? -maxSpeed : vl);
	velocityRight = vr > maxSpeed ? maxSpeed : (vr < -maxSpeed ? -maxSpeed : vr);
	lock.unlock();
	std::cout << "v = " << v << ", w = " << w << ", theta = " << genState(2) 
		<< ", time = " << time << '\n';
}

UnicycleWMR::Path::Path(double xInit, double xEnd, vector<Vector2d>(*pathGenerator)(double xInit, double xEnd))
{
	pathPoints = pathGenerator(xInit, xEnd);
}

UnicycleWMR::Vector3d UnicycleWMR::Controller::RotationTransform(Vector3d state, const double theta)
{
	Matrix3d transform;
	transform << cos(theta), sin(theta), 0,
		-sin(theta), cos(theta), 0,
		0, 0, 1;
	return transform * state;
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

void UnicycleWMR::Controller::UnifiedControl(UnicycleWMR::Model& robot, RRT::ParametricCurve2D& desiredCurve)
{
	double time = robot.Time();
	Vector3d desiredState;
	Vector3d desiredStateDot;
	double vd;
	double wd;
	if (slowTimeFlag) ApplyVelocityConstraint(desiredCurve, time, vd, wd, desiredState, desiredStateDot);
	else {
		desiredState = desiredCurve(time);
		desiredStateDot = desiredCurve.Velocity(time);
		vd = sqrt(pow(desiredStateDot(0), 2.0) + pow(desiredStateDot(0), 2.0));
		wd = desiredStateDot(2);
	}

	// Feedback control
	Vector3d error = robot.State() - desiredState;
	Vector3d errorDot = robot.StateDot() - desiredStateDot;
	error = RotationTransform(error, robot.State()(2));
	errorDot = RotationTransform(errorDot, robot.State()(2));
	this->alpha->Calculate(time, error(0), error(1), errorDot(0), errorDot(1), vd, wd);
	double alpha = this->alpha->Alpha();
	double alphaDot = this->alpha->AlphaDot();
	double f1 = (sin(error(2)) - sin(alpha)) / (error(2) - alpha + 1e-6);

	double v = -k1 * error(0) + vd * cos(error(2));
	double w = -k2 * (error(2) - alpha) + wd - k0 * vd * error(1) * f1 + alphaDot;
	robot.SetCtrlInput2(v, w);
	std::cout << "vd = " << vd << ", wd = " << wd << ", thetad = " << desiredState(2)
		<< ", alpha = " << alpha << ", alphadot = " << alphaDot << ", time = " << time << '\n';
}

void UnicycleWMR::Controller::DirectControl(UnicycleWMR::Model& robot, RRT::ParametricCurve2D& desiredCurve)
{
	double time = robot.Time();
	Vector3d desiredState = desiredCurve(time);
	Vector3d desiredStateDot = desiredCurve.Velocity(time);
	double vd = sqrt(pow(desiredStateDot(0), 2.0) + pow(desiredStateDot(0), 2.0));
	double wd = desiredStateDot(2);
	robot.SetCtrlInput2(vd, wd);
	std::cout << "vd = " << vd << ", wd = " << wd << ", thetad = " << desiredState(2)
		<< ", time = " << time << '\n';
}

void UnicycleWMR::Controller::ApplyVelocityConstraint(RRT::ParametricCurve2D& desiredCurve, double& time, double& vd, double& wd, Vector3d& desiredState, Vector3d& desiredStateDot)
{
	double dtau = time - realTime;
	double dt = dtau / beta;
	realTime += dtau;
	pathTime += dt;
	
	desiredState = desiredCurve(pathTime);
	desiredStateDot = desiredCurve.Velocity(pathTime);
	
	vd = sqrt(pow(desiredStateDot(0), 2.0) + pow(desiredStateDot(1), 2.0));
	wd = desiredStateDot(2);
	static const double c = (robot->wheelDist / 2 - robot->centerOffsetY) / robot->wheelDist;
	double vl = abs(vd - c * robot->wheelDist * wd);
	double vr = abs(vd + (1 - c) * robot->wheelDist * wd);

	if (vl > vr)
	{
		if (vl > robot->maxSpeed) beta = vl / robot->maxSpeed;
		else beta = 1.0;

	}
	else
	{
		if (vr > robot->maxSpeed) beta = vr / robot->maxSpeed;
		else beta = 1.0;
	}
	vd /= beta;
	wd /= beta;
	std::cout << "Real time: " << time << " ,Path time: " << pathTime << " ,beta: " << beta << '\n';
}

void UnicycleWMR::Controller::Reset(UnicycleWMR::Model& robot)
{
	robot.Time() = 0.0;
	robot.SetCtrlInput(0, 0);
	alpha->Reset();
	realTime = 0.0;
	pathTime = 0.0;
	beta = 1.0;
}

RRT::Tree::Tree(Vector2d initPoint, double horizontalRange, double verticalRange, double delta):
	horizontalRange{horizontalRange}, verticalRange{verticalRange}, delta{delta}, space(dim)
{
	root = add_vertex(tree);
	tree[root].state << initPoint(0), initPoint(1), 0;
	tree[root].stateDot << 0, 0, 0;
	tree[root].index = idx;
	tree[root].time = 0.0;
	vertex_vec[idx] = root;

	// ANN
	algHNSW = new hnswlib::HierarchicalNSW<float>(&space, maxElements, M, efConstruction);
	for (auto k = 0; k < dim; ++k)
	{
		data[idx * dim + k] = tree[root].state(k);
	}
	algHNSW->addPoint(data + idx * dim, idx);
	++idx;
}

RRT::Tree::~Tree()
{
	delete algHNSW;
	delete[] data;
}

double Distance(RRT::Vector2d& p1, RRT::Vector2d& p2)
{
	return sqrt(pow(p2(0) - p1(0), 2.0) + pow(p2(1) - p1(1), 2.0));
	//return abs(p2(0) - p1(0)) + abs(p2(1) - p1(1));
}

RRT::Tree::Vertex RRT::Tree::NearestNeighbour(Vector2d point)
{
	// ANN
	float pos[2];
	pos[0] = point(0);
	pos[1] = point(1);
	std::priority_queue<std::pair<float, hnswlib::labeltype>>  res = algHNSW->searchKnn(pos, 1);
	

	pair<VertexIterator, VertexIterator> vp;
	double min = horizontalRange + verticalRange;
	Vertex nearest = vertex_vec[res.top().second];
	//Vertex nearest = *vertices(tree).first;
	//for (vp = vertices(tree); vp.first != vp.second; ++vp.first)
	//{
	//	if (tree[*vp.first].index == res.top().second)
	//	{
	//		nearest = *vp.first;
	//	}
	//	//Vector2d v = tree[*vp.first].state(seq(0, 1));
	//	//double distance = Distance(point, v);
	//	//if (distance < min)
	//	//{
	//	//	min = distance;
	//	//	nearest = *vp.first;
	//	//}
	//}
	return nearest;
}

RRT::Vector2d RRT::Tree::NewPoint(Vector2d randPoint, Vector2d nearestPoint, double delta)
{
	const int tries = 10;
	auto unitDirection = (randPoint - nearestPoint).normalized();
	for (auto i = 0; i < tries; ++i)
	{
		bool collided = false;
		auto newPoint = nearestPoint + unitDirection * delta;
		for (auto& obstacle : obstacles->List())
		{
			if (obstacle->Ellipse())
			{
				if (CheckCollisionEllipse(nearestPoint, newPoint, obstacle->Ellipse()->Shape()))
				{
					delta /= 2;
					collided = true;
					break;
				}
			}
			else if (obstacle->Rectangle())
			{
				auto rectCenter = Vector2d{ {obstacle->Rectangle()->Origin().x, obstacle->Rectangle()->Origin().y} };
				if (CheckCollisionRect(nearestPoint, newPoint, obstacle->Rectangle()->Shape(), rectCenter, obstacle->Rectangle()->Rotation() * S_PI / 180.0))
				{
					delta /= 2;
					collided = true;
					break;
				}
			}
		}
		if (!collided) return newPoint;
	}
	return nearestPoint;
}

RRT::Tree::Vertex RRT::Tree::Extend(Vector2d point)
{
	Vertex nearest = NearestNeighbour(point);
	Vector2d nearestPoint = tree[nearest].state(seq(0, 1));
	//std::cout << "Nearest :" << nearestPoint(0) << ", " << nearestPoint(1) << "\n";
	Vector2d newPoint = NewPoint(point, nearestPoint, delta);
	if (nearestPoint == newPoint) return nearest;
	newPoint(0) = newPoint(0) <= 50 ? 50 : (newPoint(0) >= horizontalRange - 50 ? horizontalRange - 50: newPoint(0));
	newPoint(1) = newPoint(1) <= 50 ? 50 : (newPoint(1) >= verticalRange - 50 ? verticalRange - 50 : newPoint(1));
	
	Vertex newVertex = add_vertex(tree);
	Edge newEdge;
	bool found;
	tie(newEdge, found) = add_edge(nearest, newVertex, tree);
	tree[newVertex].state << newPoint(0), newPoint(1), 0;
	tree[newVertex].index = idx;
	tree[newEdge].distance = delta;
	vertex_vec[idx] = newVertex;

	// ANN
	for (auto k = 0; k < dim; ++k)
	{
		data[idx * dim + k] = newPoint(k);
	}
	algHNSW->addPoint(data + idx * dim, idx);
	++idx;

	return newVertex;
}

bool RRT::Tree::FindTarget(Vertex vertex, Vector2d targetPoint, double distance)
{
	// Iterate through vertices
	Vector2d vPoint = tree[vertex].state(seq(0, 1));
	if (distance > Distance(vPoint, targetPoint))
	{
		Vertex newVertex = add_vertex(tree);
		Edge newEdge;
		bool found;
		tie(newEdge, found) = add_edge(vertex, newVertex, tree);
		tree[newVertex].state << targetPoint(0), targetPoint(1), 0;
		tree[newVertex].index = idx;
		tree[newEdge].distance = delta;
		vertex_vec[idx] = newVertex;
		// ANN
		for (auto k = 0; k < dim; ++k)
		{
			data[idx * dim + k] = targetPoint(k);
		}
		algHNSW->addPoint(data + idx * dim, idx);
		++idx;
		destination = newVertex;
		return TRUE;
	}
	return FALSE;
}

void RRT::Tree::ExploreN(int steps)
{
	auto startTime = std::chrono::system_clock::now();
	for (auto i = 0; i < steps; ++i)
	{
		Vector2d randPoint = Vector2d::Random();
		randPoint(0) = randPoint(0) * horizontalRange / 2.0 + horizontalRange / 2.0;
		randPoint(1) = randPoint(1) * verticalRange / 2.0 + verticalRange / 2.0;
		//std::cout << "Random Point :" << randPoint(0) << ", " << randPoint(1) << "\n";
		Extend(randPoint);
	}
	std::chrono::time_point<std::chrono::system_clock> endTime = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed = endTime - startTime;
	//std::cout << "Duration: " << elapsed.count();
}

void RRT::Tree::ExploreTillTarget(Vector2d targetPoint, int explorationSteps)
{
	for (auto i = 0; i < explorationSteps; ++i)
	{
		Vector2d randPoint = Vector2d::Random();
		randPoint(0) = randPoint(0) * horizontalRange / 2.0 + horizontalRange / 2.0;
		randPoint(1) = randPoint(1) * verticalRange / 2.0 + verticalRange / 2.0;
		Vertex vertex = Extend(randPoint);
		if (FindTarget(vertex, targetPoint, delta)) break;
	}
}

RRT::vector<RRT::Tree::Edge> RRT::Tree::DijkstraShortestPath(Vertex home, Vertex destination)
{
	// Dijkstra shortest path algorithm
	pair<VertexIterator, VertexIterator> vp = vertices(tree);
	IndexMap indexMap = get(vertex_index, tree);
	vector<double> distances(num_vertices(tree));
	vector<Vertex> predecessors(num_vertices(tree));
	dijkstra_shortest_paths(tree, home, weight_map(get(&RRT::Link::distance, tree)).
		distance_map(make_iterator_property_map(distances.begin(), indexMap)).
		predecessor_map(make_iterator_property_map(predecessors.begin(), indexMap)));

	// Get a sequence of edges representing the shortest path
	int count = 0;
	shortestPath.clear();
	vector<Edge> pathEdges;
	OutEdgeIterator start;
	OutEdgeIterator end;
	while (destination != home || count > num_vertices(tree))
	{
		for (tie(start, end) = out_edges(destination, tree); start != end; ++start)
		{
			Edge edge = *start;
			Vertex targ = target(edge, tree);
			if (predecessors[indexMap[destination]] == targ)
			{
				pathEdges.insert(pathEdges.begin(), edge);
				shortestPath.insert(shortestPath.begin(), tree[destination].state(seq(0, 1)));
				destination = targ;
			}
		}
		++count;
	}
	shortestPath.insert(shortestPath.begin(), tree[home].state(seq(0, 1)));
	return pathEdges;
}

bool RRT::Tree::SegmentIntersectionDetection(Vector2d source0, Vector2d target0, Vector2d source1, Vector2d target1)
{
	auto unit0 = (target0 - source0).normalized();
	auto unit1 = (target1 - source1).normalized();
	if (unit0 == unit1 || unit0 == -unit1) return false;

	auto distance0 = (target0 - source0).norm();
	auto distance1 = (target1 - source1).norm();
	MatrixXd u(2, 2);
	u << unit0(0), -unit1(0), 
		unit0(1), -unit1(1);
	//std::cout << u << '\n';
	Vector2d c = source1 - source0;
	Vector2d d = u.colPivHouseholderQr().solve(c);

	if ((0 < d(0) && d(0) < distance0) && (0 < d(1) && d(1) < distance1))
	{
		return true;
	}
	//double slope0 = (target0(1) - source0(1)) / (target0(0) - source0(0));
	//double slope1 = (target1(1) - source1(1)) / (target1(0) - source1(0));
	//double intercept0 = source0(1) - slope0 * source0(0);
	//double intercept1 = source1(1) - slope1 * source1(0);
	//double x = (intercept0 - intercept1) / ((slope1 - slope0) + 1e-6);
	//if ((target0(0) >= source0(0) ? (source0(0) < x && x < target0(0)) : (target0(0) < x && x < source0(0))) &&
	//	(target1(0) >= source1(0) ? (source1(0) < x && x < target1(0)) : (target1(0) < x && x < source1(0))))
	//{
	//	return true;
	//}
	return false;
}

bool RRT::Tree::CheckCollisionRect(Vector2d source, Vector2d target, D2D1_RECT_F rect, Vector2d center, double rotation)
{
	rect.left = rect.left < rect.right ? rect.left - collisionPadding : rect.left + collisionPadding;
	rect.right = rect.left < rect.right ? rect.right + collisionPadding : rect.right - collisionPadding;
	rect.top = rect.top < rect.bottom ? rect.top - collisionPadding : rect.top + collisionPadding;
	rect.bottom = rect.top < rect.bottom ? rect.bottom + collisionPadding : rect.bottom - collisionPadding;

	// Rectangle info
	vector<Vector2d> rectVertices(4);
	rectVertices[0] = Vector2d{ {rect.left, rect.top} };
	rectVertices[1] = Vector2d{ {rect.right, rect.top} };
	rectVertices[2] = Vector2d{ {rect.left, rect.bottom} };
	rectVertices[3] = Vector2d{ {rect.right, rect.bottom} };
	vector<pair<Vector2d, Vector2d>> rectSegments(4);
	rectSegments[0] = pair<Vector2d, Vector2d>(rectVertices[0], rectVertices[1]);
	rectSegments[1] = pair<Vector2d, Vector2d>(rectVertices[0], rectVertices[2]);
	rectSegments[2] = pair<Vector2d, Vector2d>(rectVertices[3], rectVertices[1]);
	rectSegments[3] = pair<Vector2d, Vector2d>(rectVertices[3], rectVertices[2]);

	for (auto& segment : rectSegments)
	{
		Vector2d s = Rotate2DVector((segment.first - center), rotation) + center;
		Vector2d t = Rotate2DVector((segment.second - center), rotation) + center;
		if (SegmentIntersectionDetection(source, target, s, t))
		{
			return true;
		}
	}
	return false;
}

bool RRT::Tree::CheckCollisionEllipse(Vector2d source, Vector2d target, D2D1_ELLIPSE ellipse)
{
	Vector2d ellipseCenter{ {ellipse.point.x, ellipse.point.y} };
	double radius = ellipse.radiusX >= ellipse.radiusY ? ellipse.radiusX : ellipse.radiusY;
	radius += collisionPadding;
	if ((source - ellipseCenter).norm() <= radius) return true;
	if ((target - ellipseCenter).norm() <= radius) return true;
	Vector2d vector = VectorToLine2D(source, target, ellipseCenter);
	if (vector.norm() <= radius && CheckBetween(source, target, ellipseCenter + vector)) return true;
	return false;
}

RRT::Vector2d RRT::Tree::Rotate2DVector(Eigen::Vector2d vector, double rotation)
{
	Eigen::Matrix2d rotationMatrix;
	rotationMatrix << cos(rotation * S_PI / 180.0), -sin(rotation * S_PI / 180.0),
		sin(rotation * S_PI / 180.0), cos(rotation * S_PI / 180.0);
	vector = rotationMatrix * vector;
	return vector;
}

bool RRT::Tree::CheckBetween(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d p0)
{
	return ((point1(0) < point2(0)) ? (p0(0) >= point1(0) && p0(0) <= point2(0)) : (p0(0) >= point2(0) && p0(0) <= point1(0))) &&
		((point1(1) < point2(1)) ? (p0(1) >= point1(1) && p0(1) <= point2(1)) : (p0(1) >= point2(1) && p0(1) <= point1(1)));
}

RRT::Vector2d RRT::Tree::VectorToLine2D(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d p0)
{
	Vector2d n = (point2 - point1).normalized();
	return (point1 - p0) - ((point1 - p0).dot(n)) * n;
}

double RRT::CubicSpline::slope(vector<DataPoint1D> dataPoints, int i, int j)
{
	return (dataPoints[i].value - dataPoints[j].value) / (dataPoints[i].time - dataPoints[j].time);
}

RRT::CubicSpline::CubicSpline(vector<DataPoint1D> dataPoints, CubicSplineEndCondition startCondition, CubicSplineEndCondition endCondition) :
	dataPoints{ dataPoints }, startCondition{ startCondition }, endCondition{ endCondition }, coefficient{ vector<vector<double>>(dataPoints.size()) }
{
	const int num = dataPoints.size();
	if (num < 4) return;
	MatrixXd A(num, num);
	VectorXd x(num);
	VectorXd b(num);
	vector<double> h(num - 1);

	for (int i = 0; i < num - 1; ++i)
	{
		h[i] = dataPoints[i + 1].time - dataPoints[i].time;
	}

	for (int row = 0; row < num; ++row)
	{
		for (int col = 0; col < num; ++col)
		{
			if (row > 0 && row < num - 1)
			{
				if (row == col + 1) A(row, col) = h[row - 1];
				else if (row == col) A(row, col) = 2 * (h[row - 1] + h[row]);
				else if (row == col - 1) A(row, col) = h[row];
				else A(row, col) = 0.0;
			}
			else A(row, col) = 0.0;
		}

		if (row > 0 && row < num - 1) b[row] = 3 * (slope(dataPoints, row + 1, row) - slope(dataPoints, row, row - 1));
		else b[row] = 0.0;
	}

	// Start Condition
	if (startCondition.endCondition == EndCondition::Natural)
	{
		A(0, 0) = 1.0;
	}
	else if (startCondition.endCondition == EndCondition::Clamped)
	{
		A(0, 0) = 2 * h[0];
		A(0, 1) = h[0];
		b(0) = 3 * (slope(dataPoints, 1, 0) - startCondition.firstDerivative);
	}
	else if (startCondition.endCondition == EndCondition::NotaKnot)
	{
		A(0, 0) = h[1];
		A(0, 1) = -(h[0] + h[1]);
		A(0, 2) = h[0];
	}
	else if (startCondition.endCondition == EndCondition::FirstSecond)
	{
		A(0, 0) = 1.0;
		for (int col = 0; col < num; ++col)
		{
			if (col == 1) A(1, col) = 1.0;
			else A(1, col) = 0;
		}
		b(0) = startCondition.secondDerivative / 2;
		b(1) = (3 * (slope(dataPoints, 1, 0) - startCondition.firstDerivative) - startCondition.secondDerivative * h[0]) / h[0];
	}

	// End Condition
	if (endCondition.endCondition == EndCondition::Natural)
	{
		A(num - 1, num - 1) = 1.0;
	}
	else if (endCondition.endCondition == EndCondition::Clamped)
	{
		A(num - 1, num - 1) = 2 * h[num - 2];
		A(num - 1, num - 2) = h[num - 2];
		b(num - 1) = 3 * (endCondition.firstDerivative - slope(dataPoints, num - 1, num - 2));
	}
	else if (endCondition.endCondition == EndCondition::NotaKnot)
	{
		A(num - 1, num - 1) = h[num - 3];
		A(num - 1, num - 1) = -(h[num - 3] + h[num - 2]);
		A(num - 1, num - 1) = h[num - 2];
	}
	else if (endCondition.endCondition == EndCondition::FirstSecond)
	{
		A(num - 1, num - 1) = 1.0;
		for (int col = 0; col < num; ++col)
		{
			if (col == num - 2) A(num - 2, col) = 1.0;
			else A(num - 2, col) = 0;
		}
		b(num - 1) = endCondition.secondDerivative / 2;
		b(num - 2) = (3 * (endCondition.firstDerivative - slope(dataPoints, num - 1, num - 2)) - endCondition.secondDerivative * h[num - 2]) / h[num - 2];
	}
	
	// Solve for x
	x = A.colPivHouseholderQr().solve(b);
	//std::cout << x;
	for (int k = 0; k < num - 1; ++k)
	{
		coefficient[k] = vector<double>(4);
		coefficient[k][2] = x(k);
		coefficient[k][3] = (x(k + 1) - x(k)) / 3 / h[k];
		coefficient[k][1] = slope(dataPoints, k + 1, k) - h[k] / 3 * (2 * x(k) + x(k + 1));
		coefficient[k][0] = dataPoints[k].value;
	}
}

int RRT::CubicSpline::TimeIndex(double t)
{
	int idx = 0;
	if (t >= dataPoints.back().time)
	{
		return (int)(dataPoints.size() - 2);
	}
	for (int k = 0; k < dataPoints.size() - 1; ++k)
	{
		if (t >= dataPoints[k].time && t < dataPoints[k + 1].time)
		{
			idx = k;
			break;
		}
	}
	return idx;
}

double RRT::CubicSpline::operator()(double t)
{
	int idx = TimeIndex(t);
	double t0 = dataPoints[idx].time;
	double y = 0;
	for (int i = 0; i < 4; ++i)
	{
		y += coefficient[idx][i] * pow(t - t0, i);
	}
	return y;
}

double RRT::CubicSpline::FirstDerivative(double t)
{
	int idx = TimeIndex(t);
	double t0 = dataPoints[idx].time;
	double y = 0;
	for (int i = 1; i < 4; ++i)
	{
		y += i * coefficient[idx][i] * pow(t - t0, i - 1);
	}
	return y;
}

double RRT::CubicSpline::SecondDerivative(double t)
{
	int idx = TimeIndex(t);
	double t0 = dataPoints[idx].time;
	double y = 0;
	for (int i = 2; i < 4; ++i)
	{
		y += (i - 1) * i * coefficient[idx][i] * pow(t - t0, i - 2);
	}
	return y;
}

RRT::ParametricCurve2D::ParametricCurve2D(vector<Vector2d> path, double w0, double wn, double maxVelocity, double rampDistance) : dataPoints{ vector<DataPoint2D>(path.size()) }
{
	double totalDistance = 0.0;
	this->maxVelocity = maxVelocity;
	if (path.size() == 0) return;
	vector<double> distances(path.size());
	distances[0] = 0.0;
	for (int i = 0; i < path.size() - 1; ++i)
	{
		totalDistance += sqrt(pow(path[i + 1](0) - path[i](0), 2.0) + pow(path[i + 1](1) - path[i](1), 2.0));
		distances[i + 1] = totalDistance;
	}
	startTime = 0.0;
	edgeTime = 2 * rampDistance / maxVelocity;
	double deltaTime = (totalDistance - 2 * rampDistance) / maxVelocity + 2 * edgeTime;
	endTime = startTime + deltaTime;

	vector<DataPoint1D> dataPointsX(path.size());
	vector<DataPoint1D> dataPointsY(path.size());

	// Calculate times
	for (int i = 0; i < path.size(); ++i)
	{
		dataPointsX[i].value = path[i](0);
		dataPointsX[i].time = TrapezoidTime(distances[i]);

		dataPointsY[i].value = path[i](1);
		dataPointsY[i].time = TrapezoidTime(distances[i]);

		dataPoints[i].x = path[i](0);
		dataPoints[i].y = path[i](1);
		dataPoints[i].time = TrapezoidTime(distances[i]);
	}
	double accelX0 = cos(w0) * maxVelocity / edgeTime;
	double accelY0 = sin(w0) * maxVelocity / edgeTime;
	double accelXn = cos(wn) * maxVelocity / edgeTime;
	double accelYn = sin(wn) * maxVelocity / edgeTime;
	splineX = CubicSpline(dataPointsX, CubicSplineEndCondition(0.0, accelX0), CubicSplineEndCondition(0.0, accelXn));
	splineY = CubicSpline(dataPointsY, CubicSplineEndCondition(0.0, accelY0), CubicSplineEndCondition(0.0, accelYn));
}

RRT::Vector3d RRT::ParametricCurve2D::operator()(double t)
{
	Vector3d state;
	if (t > endTime) {
		state(0) = splineX(endTime);
		state(1) = splineY(endTime);
		state(2) = atan2(splineY.FirstDerivative(endTime), splineX.FirstDerivative(endTime));
		return state;
	}
	state(0) = splineX(t);
	state(1) = splineY(t);
	state(2) = atan2(splineY.FirstDerivative(t), splineX.FirstDerivative(t));

	return state;
}

RRT::Vector3d RRT::ParametricCurve2D::Velocity(double t)
{
	Vector3d stateDot;
	if (t > endTime) {
		stateDot << 0.0, 0.0, 0.0;
		return stateDot;
	}
	static double theta = 0.0;
	double firstDerivativeY = splineY.FirstDerivative(t);
	double firstDerivativeX = splineX.FirstDerivative(t);
	if (abs(firstDerivativeY) > 0.001 && abs(firstDerivativeX) > 0.001)
	{
		theta = atan2(splineY.FirstDerivative(t), splineX.FirstDerivative(t));
	}
	stateDot(0) = splineX.FirstDerivative(t);
	stateDot(1) = splineY.FirstDerivative(t);
	double v = sqrt(pow(stateDot(0), 2.0) + pow(stateDot(1), 2.0));
	stateDot(2) = 1 / (1 + pow(tan(theta), 2.0)) * splineY.SecondDerivative(t) / pow(splineX.FirstDerivative(t), 2.0) * v * cos(theta);
	std::cout << "Xdot = " << stateDot(0) << " Ydot = " << stateDot(1) << " Theta = " << theta << '\n';
	return stateDot;
}

double RRT::ParametricCurve2D::TrapezoidVelocity(double t)
{
	if (t < startTime) return 0.0;
	else if (t > endTime) return 0.0;
	else if (t >= startTime && t < edgeTime) return maxVelocity / edgeTime * t;
	else if (t >= edgeTime < endTime - edgeTime) return maxVelocity;
	else return maxVelocity - maxVelocity / edgeTime * (t - endTime + edgeTime);
}

double RRT::ParametricCurve2D::TrapezoidTime(double distance)
{
	double slope = maxVelocity / edgeTime;
	if (distance < 0) return 0.0;
	else if (distance >= 0 && distance < maxVelocity * edgeTime / 2) return sqrt(2 * distance / slope);
	else if (distance >= maxVelocity * edgeTime / 2 && distance < maxVelocity * (edgeTime / 2 + endTime - 2 * edgeTime))
	{
		return edgeTime + (distance - maxVelocity * edgeTime / 2) / maxVelocity;
	}
	else if (distance >= maxVelocity * (edgeTime / 2 + endTime - 2 * edgeTime))
	{
		return endTime - edgeTime + sqrt(2 * (distance - (maxVelocity * (edgeTime / 2 + endTime - 2 * edgeTime))) / slope);
	}
	else return endTime;
}

bool UnicycleWMR::PathPlanner::PlanPath(Vector3d home, Vector3d target, ShapeList<ObstacleModel>* obstacles, double explorationDelta, int explorationSteps,
	double maxVelocity, double rampDistance)
{
	tree.reset(new RRT::Tree(home(seq(0,1)), horizontalRange, verticalRange, explorationDelta));
	tree->obstacles = obstacles;
	tree->ExploreTillTarget(target(seq(0,1)), explorationSteps);
	tree->DijkstraShortestPath(tree->Home(), tree->Target());
	vector<Vector2d> path = tree->ShortestPath();
	if (path.size() < 4) { return false; }
	pathCurve.reset(new RRT::ParametricCurve2D(path, home(2), target(2), maxVelocity, rampDistance));
	return true;
}

double UnicycleWMR::Alpha1::operator()(double time, double xe, double ye, double vd, double wd)
{
	double dt = time - this->time;
	rhoDot = -(abs(vd) + abs(wd)) * rho;
	rho += rhoDot * dt;
	h = H(time, xe, ye);
	this->time = time;

	return rho * h;
}

double UnicycleWMR::Alpha1::FirstDerivative(double time, double xe, double ye, double xeDot, double yeDot, double vd, double wd)
{
	hDot = HDot(time, xe, ye, xeDot, yeDot);
	return rhoDot * h + rho * hDot;
}

double UnicycleWMR::Alpha1::H(double time, double xe, double ye)
{
	double r = pow(xe, 2.0) + pow(ye, 2.0);
	return h0 * tanh(a * pow(r, b)) * sin(c * time);
}

double UnicycleWMR::Alpha1::HDot(double time, double xe, double ye, double xeDot, double yeDot)
{
	double r = pow(xe, 2.0) + pow(ye, 2.0);
	double rDot = 2 * xe * xeDot + 2 * ye * yeDot;
	return a * b * h0 * sin(c * time) * pow(r, b - 1) * rDot * pow(cosh(a * pow(r, b)), -2) + c * h0 * cos(c * time) * tanh(a * pow(r, b));

}

void UnicycleWMR::Alpha1::Calculate(double time, double xe, double ye, double xeDot, double yeDot, double vd, double wd)
{
	alpha = operator()(time, xe, ye, vd, wd);
	alphaDot = FirstDerivative(time, xe, ye, xeDot, yeDot, vd, wd);
}

void UnicycleWMR::Alpha1::Reset()
{
	time = 0.0;
	rho = 1.0;
	rhoDot = 1.0;
	h = 0.0;
	hDot = 0.0;
}

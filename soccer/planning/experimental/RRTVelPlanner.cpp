
#include "RRTVelPlanner.hpp"
#include <math.h>


using namespace std;
using namespace Planning;
using namespace Eigen;


static bool RRTVelPlanner::coordinateIsOnField(Eigen::Vector2d &pt) {
	return ABS(pt.x) < Field_Width - 2*Robot_Radius
			&& pt.y > Robot_Radius
			&& pt.y < Field_Length - Robot_Radius;
}

// static Eigen::Vector2d randomFieldPoint() {
// 	///	inset the field dimensions by @Robot_Radius on both sides
// 	static float w = Field_Width - 2*Robot_Radius;
// 	static float l = Field_Length - 2*Robot_Radius;

// 	//	note: drand48() returns a random double in [0.0, 1.0]
// 	float x = w * (drand48() - 0.5f);	//	x range: [-w/2, w/2]
// 	float y = l * drand48();			//	y range: [0.0, l]

// 	return Vector2d(x, y);
// }

Vector2d RRTVelPlanner::randomAcceleration() {
	float mag = drand48() * maxAcceleration;
	float dir = drand48() * 2 * M_PI;

	return Vector2d(mag*cosf(dir), mag*sinf(dir));
}

Planning::Path *RRTVelPlanner::run(
	const MotionState &startState,
	const MotionState &goalState,
	const ObstacleGroup *staticObstacles,
	const SystemState *systemState,
	const vector<OurRobot *> alreadyPlannedBots);
{
	_rrtTree.clear();

	//	FIXME: if start == goal, return

	//	FIXME: what do if the goal or the start is blocked by an obstacle?

	//	FIXME: what if we can do a direct-shot from a to b?  return a path of size 2

	//	at each iteration we pick a random Point in the tree and branch off from it
	//	with a random acceleration applied for @timeStep time
	for (int i = 0; i < maxIterations(); i++) {
		Point<MotionState> pt = _rrtTree.randomPoint();

		Vector2d acc = randomAcceleration();

		Vector2d pt = //FIXME: apply acc to pt's state.

		

		//	see if the point is valid
		if (coordinateIsOnField(pt)
			&& /** FIXME: check collisions */) {

			//	FIXME: create and add a new Point to @_rrtTree
			//	FIXME: see if @_rrtTree is within range of the goal
			//			if so, break;
		}
	}

	//	FIXME: build path
	//	FIXME: optimize
	
	//	FIXME: return MotionPath
}

RRTVelPlanner::RRTVelPlanner() {

	//	FIXME: these are both bullshit values
	maxAcceleration = .5;
	maxVelocity = 4;
}

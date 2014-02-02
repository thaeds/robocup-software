#pragma once


#include <planning/RRTTree.hpp>
#include "MotionPath.hpp"

#include <Eigen/Dense>


//	FIXME: I previously removed the stuff from the old planner that checked
//		points existing in the planned path for whether or not they hit obstacles
//		it would be good to add these back in to handle the case where the robot
//		starts out inside of an obstacle (like a ball avoid radius)

namespace Planning
{
	/**
	 * This is a rewrite of RRTPlanner that has a few major improvements:
	 * 
	 * #1: We take into account robot velocity and acceleration so that
	 *     generated paths physically make sense for the robot.
	 *
	 * #2: The planner can accept a MotionStateSource as a goal instead
	 *     of a static 2d point.  This allows us to do things like catching
	 *     a moving ball.
	 *
	 * #3: We account for moving robots.  Opponents are assumed to move in
	 *     a straight line at their current velocity indefinitely.  Robots
	 *     on our team that we have already planned paths for are accounted
	 *     for and robots that we haven't planned yet are ignored.
	 *
	 * The state of a robot is represented as a [pos, vel] tuple called MotionState
	 */
	class RRTVelPlanner {
	public:

		RRTVelPlanner();

		/**
		 * Executes the path planner with the given constraints.
		 *
		 * Basic algorithm:
		 * 1. Choose a random Point currently in the tree
		 * 2. Choose a random acceleration
		 * 3. Evaluate the motion of the bot for a fixed time step.  If it results
		 *    in a valid state (with valid in-between states), add the new Point to
		 *    the tree as a child of the Point chosen in 1.
		 * 4. See if the new Point is in reach of the goal node.  If not, go back to step 1.
		 *    If so, we'll build, then optimize the found Path.
		 *
		 * @param systemState The StateObject is passed in to generate moving
		 *                    obstacles from.  If NULL, moving obstacles are ignored.
		 * @return The best path that the planner could find
		 */
		Planning::Path *run(
			const MotionState &startState,
			const MotionState &goalState,
			const ObstacleGroup *staticObstacles,
			const SystemState *systemState,
			const vector<OurRobot *> alreadyPlannedBots);


		/**
		 * The timeStep is the amount of time separating each Point in the tree.
		 */
		float timeStep() const {
			return _timeStep;
		}
		void setTimeStep(float ts) {
			_timeStep = ts;
		}

		/**
		 * The maximum number of random points in the state-space that we will
		 * try before giving up on finding a path to the goal.
		 */

		/**
		 * These place physical constraints on the paths that the planner will
		 * generate.  Be sure to set them to appropriate values before calling
		 * the run() method.
		 */
		float maxVelocity, maxAcceleration;


	protected:
		float _timeStep;

		/**
		 * Checks to make sure @pt is within the bounds of the field.
		 * It takes into account the size of the robot and makes sure
		 * that @pt is @Robot_Radius away from any of the edges.
		 */
		static bool coordinateIsOnField(Eigen::Vector2d &pt);

		/**
		 * Randomly picks a point within the bounds of the field.
		 * It takes into account the size of the robot by only choosing
		 * points that are inset from the bounds of the field by @Robot_Radius.
		 */
		// static Eigen::Vector2d randomFieldPoint();

		/**
		 * Generates a random acceleration within the specs defined by @maxAcceleration
		 */
		Eigen::Vector2d randomAcceleration();

		FixedStepRRTTree<MotionState> _rrtTree;
	};
}

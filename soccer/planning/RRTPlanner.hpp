
#pragma once

#include <list>
#include <Geometry2d/Point.hpp>
#include <planning/Obstacle.hpp>
#include <planning/Path.hpp>

#include "RRTTree.hpp"

namespace Planning
{
	/**
	 * Generates a random state within the state-space
	 * For the position planner, this is a Geometry2d::Point
	 * For the velocity planner, this is a [pos, vel] tuple
	 */
	static Geometry2d::Point randomPoint();

	/**
	 * RRT: http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
	 * this plans the motion path for the robots
	 */
	class RRTPlanner {
	public:
		RRTPlanner();

		/**
		 * The maximum number of iterations for the RRT algorithm
		 */
		int maxIterations() const {
			return _maxIterations;
		}
		void maxIterations(int value) {
			_maxIterations = value;
		}

		
		/**
		 * Runs the path planner
		 *
		 * @param obstacles The non-moving obstacles that the planner should avoid.
		 *                  This could include other robots, goalie areas, etc.
		 * @return the best path that the planner could find
		 */
		Planning::Path *run(
			const Geometry2d::Point& start,
			const Geometry2d::Point& goal, 
			const ObstacleGroup* obstacles);

		
	protected:
		///	a tree from the start point
		FixedStepRRTTree<Geometry2d::Point> _fixedStepTree0;

		///	a tree from the goal point
		FixedStepRRTTree<Geometry2d::Point> _fixedStepTree1;
		
		///	maximum number of rrt iterations to run
		///	this does not include connect attempts
		unsigned int _maxIterations;
		
		///	latest obstacles
		const ObstacleGroup* _obstacles;
		
		/**
		 * makes a path from the last point of each tree
		 * If the points don't match up...fail!
		 * The final path will be from the start of tree0
		 * to the start of tree1.
		 * Once the path is created, it is optimized, then returned.
		 */
		void makePath();
		
		/**
		 * Optimize the path and stores the result in @path (since it's passed in by reference)
		 * It removes points from @path that are unneeded.  If the path is still
		 * valid and doesn't hit an obstacle when the point is removed, it is unneeded.
		 */
		static void optimize(Planning::Path &path, const ObstacleGroup *obstacles);
	};
}

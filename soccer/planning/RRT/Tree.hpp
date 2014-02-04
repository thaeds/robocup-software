#pragma once

#include <list>

#include <Geometry2d/Segment.hpp>
#include <planning/Path.hpp>

namespace RRT
{
	/**
	 * Base class for an rrt tree node
	 *
	 * The template parameter T is for storing the state that the Point node represents.
	 * The type T should implement the '-' operator for measuring the distance between 2 states.
	 */
	template<typename T>
	class Point {
	public:
		Point(T &state, Point<T> *parent);
		
		Point<T>* parent() const {
			return _parent;
		}
		
		bool isLeaf() const {
			return _leaf;
		}
		void setLeaf(bool isLeaf) {
			_leaf = isLeaf;
		}

		/**
		 * Gets the number of ancestors (parent, parent's parent, etc) that
		 * the node has.
		 * Returns 0 if it doesn't have a parent.
		 */
		int depth();

		/**
		 * The @state property is the point in the state-space that this
		 * Point/node represents.  Generally this is a vector (could be 2d, 3d, etc)
		 */
		T &state() const {
			return _state;
		}

	private:
		T _state;
		std::list<Point<T> *> _children;
		Point<T> *_parent;
		bool _leaf;
	};


	/**
	 * Base tree class for RRT trees.  This class provides the generic data structure
	 * for the tree and the nodes tha make it up and some general algorithms.  Because
	 * many parts of an RRT are implementation-/domain- specific, parts of it should be
	 * placed in callbacks (this is a TODO item for now).
	 * Note: The callbacks used in this class are C++ lambdas.  A good tutorial on them
	 * can be found here: http://www.cprogramming.com/c++11/c++11-lambda-closures.html.
	 *
	 * An RRT tree searches a state space by randomly filling it in and connecting
	 * points to form a branching tree.  Once a branch of the tree reaches the goal and
	 * satisifes all of the constraints, a solution is found and returned.
	 *
	 * The template parameter T is to specify the type that represents a state within
	 * the state-space that the tree is searching.  This could be a Geometry2d::Point or
	 * something else, but will generally be some sort of vector.
	 */
	template<typename T, typename P = Point<T> >
	class Tree {
	public:
		Tree();
		virtual ~Tree();


		/**
		 * Removes all Points from the tree so it can be run() again.
		 */
		void reset();
		
		/**
		 * Executes the RRT algorithm with the given start state.  The run()
		 * method calls reset() automatically before doing anything.
		 *
		 * @return a bool indicating whether or not it found the goal
		 */
		bool run(const T &start);

		/**
		 * The maximum number of random points in the state-space that we will try
		 * before giving up on finding a path to the goal.
		 */
		int maxIterations() const {
			return _maxIterations;
		}
		void setMaxIterations(int itr) {
			_maxIterations = itr;
		}
		
		/**
		 * Find the point of the tree closest to @state
		 */
		Point<T> *nearest(T state);
		
		/**
		 * Grow the tree in the direction of @pt
		 *
		 * @return the new tree Point (may be NULL if we hit Obstacles)
		 * @param base The Point to connect from.  If base == NULL, then
		 *             the closest tree point is used
		 */
		virtual Point<T> *extend(T state, Point<T> *base = NULL) = 0;
		
		/**
		 * Attempts to connect @state into the tree by repeatedly calling extend()
		 * to connect a series of new Points in series from the closest point already
		 * in the tree towards @state.
		 *
		 * @param state The state to try to connect into the tree
		 * @return true if the connection was successfully made, false otherwise
		 */
		virtual bool connect(const T &state) = 0;
		
		/**
		 * Get the path from the receiver's root point to the dest point
		 *
		 * @param path the Path object to append the series of states to
		 * @param reverse if true, the points added to @path will be from dest to the tree's root
		 */
		//	FIXME: remove dependency on Path
		void getPath(Planning::Path &path, Point<T> *dest, const bool reverse = false);
		
		/**
		 * @return The first point (the one passed to init()) or NULL if none
		 */
		Point<T> *rootPoint() const;
		
		/**
		 * @return The most recent Point added to the tree
		 */
		Point<T> *lastPoint() const;


		//
		//	Callbacks - These MUST be overridden before using the Tree
		//

		/**
		 * This callback determines if a given transition is valid.
		 */
		bool [](Point<T> startPt, T &newState) transitionValidCallback;

		/**
		 * Override this to provide a way for the Tree to generate random states.
		 *
		 * @return a state that is randomly chosen from the state-space
		 */
		T []() randomStateCallback;

		/**
		 * This callback accepts two states and returns the 'distance' between
		 * them.
		 */
		float [](T &stateA, T &stateB) distanceCallback;

		/**
		 * Callback to see if a given Point is at or near enough to the goal.  Note
		 * that the Tree never asks where the goal is, only if a given Point is near
		 * 
		 */
		bool [](Point<T> *pt) pointNearGoal;


	protected:
		/**
		 * A list of all Point objects in the tree.
		 */
		std::list<Point<T> *> points;

		int _maxIterations;
	};
	
	/**
	 * Tree that grows based on fixed distance step
	 */
	template<typename T>
	class FixedStepTree : public Tree<T> {
	public:
		FixedStepTree() {}
		
		Tree::Point* extend(T state, Tree::Point<T>* base = NULL);
		bool connect(T state);
		
		/**
		 * Tree step size...interpreted differently for different trees.
		 *
		 * In the FixedStepTree used in the position planner, represents
		 * the max distance (in cm) that one Point can be to its neighbor.
		 */
		float step;
	};
}

//	FIXME: these are domain-specific callback items
// bool Tree::stateIsValid(T state) {
// 	return !_obstacles->hit(state);
// }

// bool Tree::segmentIsValid(T from, T to) {
// 	return !_obstacles->hit(Geometry2d::Segment(from, to));
// }

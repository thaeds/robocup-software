#pragma once

#include <list>

#include <Geometry2d/Segment.hpp>
#include <planning/Path.hpp>

namespace Planning
{
	/**
	 * Base tree class for RRT trees.  This class provides the generic data structure
	 * for the tree and the nodes tha make it up and some general algorithms.  Because
	 * many parts of an RRT are implementation-/domain- specific, parts of it should be
	 * placed in callbacks (this is a TODO item for now).
	 *
	 * An RRT tree searches a state space by randomly filling it in and connecting
	 * points to form a branching tree.  Once a branch of the tree reaches the goal and
	 * satisifes all of the constraints, a solution is found and returned.
	 *
	 * The template parameter T is to specify the type that represents a state within
	 * the state-space that the tree is searching.  This could be a Geometry2d::Point or
	 * something else, but will generally be some sort of vector.
	 */
	template<typename T>
	class RRTTree {
		public:
			
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
				T state() const {
					return _state;
				}
				// void setState(T state) {
				// 	_state = state;
				// }

			private:
				T _state;
				std::list<Point<T> *> _children;
				Point<T> *_parent;
				bool _leaf;
			};

			
			RRTTree();
			virtual ~RRTTree();

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
			 * Removes all Points from the tree so it can be re-init()ed and run again.
			 */
			void clear();
			
			void init(const T &start);
			
			bool RRTTree::stateIsValid(T state) {
				return !_obstacles->hit(state);
			}

			bool RRTTree::segmentIsValid(T from, T to) {
				return !_obstacles->hit(Geometry2d::Segment(from, to));
			}

			/**
			 * Find the point of the tree closest to @state
			 */
			Point<T> *nearest(T state);
			
			/**
			 * Grow the tree in the direction of @pt
			 *
			 * @return the new tree point (may be NULL if we hit Obstacles)
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
			 * Make a path from the receiver's root point to the dest point
			 *
			 * @param path the Path object to append the series of states to
			 * @param rev if true, the points added to @path will be from dest to the tree's root (in reverse order)
			 */
			void addPath(Planning::Path &path, Point* dest, const bool rev = false);
			
			/**
			 * @return The first point (the one passed to init()) or NULL if none
			 */
			Point<T> *start() const;
			
			/**
			 * @return The most recent Point added to the tree
			 */
			Point<T> *last() const;
			
			/**
			 * A list of all Point objects in the tree.
			 */
			std::list<Point<T> *> points;

		protected:
			int _maxIterations;
	};
	
	/**
	 * Tree that grows based on fixed distance step
	 */
	template<typename T>
	class FixedStepRRTTree : public RRTTree<T> {
	public:
		FixedStepRRTTree() {}
		
		RRTTree::Point* extend(T state, RRTTree::Point<T>* base = NULL);
		bool connect(T state);
		
		/**
		 * Tree step size...interpreted differently for different trees.
		 *
		 * In the FixedStepRRTTree used in the position planner, represents
		 * the max distance (in cm) that one Point can be to its neighbor.
		 */
		float step;
	};
}

#pragma once

#include <list>

#include <Geometry2d/Segment.hpp>
#include <planning/Path.hpp>

namespace Planning
{
	/**
	 * Base tree class for rrt trees
	 * RRTTree can be grown in different ways
	 *
	 * An RRT tree searches a state space by randomly filling it in and connecting points
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
				 * Gets the number of ancestors (parent, paren'ts parent, etc) that
				 * the node has.
				 * Returs 0 if it doesn't have a parent.
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
			
			/** cleanup the tree */
			void clear();
			
			void init(const Geometry2d::Point &start, const ObstacleGroup *obstacles);
			

			bool RRTTree::stateIsValid(T state) {
				return !_obstacles->hit(state);
			}

			bool RRTTree::segmentIsValid(T from, T to) {
				return !_obstacles->hit(Geometry2d::Segment(from, to));
			}

			/** find the point of the tree closest to @state */
			Point<T> *nearest(T state);
			
			/**
			 * Grow the tree in the direction of @pt
			 *
			 * @return the new tree point (may be NULL if we hit Obstacles)
			 * @param base The Point to connect from.  If base == NULL, then
			 *             the closest tree point is used
			 */
			virtual Point<T> *extend(Geometry2d::Point pt, Point<T> *base = NULL) = 0;
			
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
			 * @param path the Path object to append the series of points to
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
			 * Tree step size...interpreted differently for different trees.
			 *
			 * In the FixedStepRRTTree used in the position planner, represents
			 * the max distance (in cm) that one Point can be to its neighbor.
			 */
			float step;
			
			/**
			 * A list of all Point objects in the tree.
			 */
			std::list<Point<T> *> points;
			
		protected:
			// const ObstacleGroup* _obstacles;
	};
	
	/**
	 * Tree that grows based on fixed distance step
	 */
	class FixedStepRRTTree : public RRTTree {
	public:
		FixedStepRRTTree() {}
		
		RRTTree::Point* extend(Geometry2d::Point pt, RRTTree::Point* base = 0);
		bool connect(Geometry2d::Point pt);
	};
}

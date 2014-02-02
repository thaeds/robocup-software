#include "RRTTree.hpp"

#include <Utils.hpp>

#include <stdio.h>
#include <iostream>
#include <boost/foreach.hpp>

using namespace Planning;
using namespace std;


#pragma mark RRTTree::Point

RRTTree::Point::Point(const T &state, RRTTree::Point *parent) :
	pos(p)
{
	_parent = parent;
	_leaf = true;
	_state = state;
	
	if (_parent)
	{
		_parent->_children.push_back(this);
		_parent->_leaf = false;
	}
}

int RRTTree::Point::depth() {
	int n = 0;
	for (Point<T> *ancestor = _parent; ancestor != NULL; ancestor = ancestor->_parent) {
		n++;
	}
	return n;
}


#pragma mark RRTTree

RRTTree::RRTTree()
{
	setMaxIterations(100);
}

RRTTree::~RRTTree()
{
	clear();
}

void RRTTree::clear()
{
    // Delete all points
    for (Point<T> *pt : points) delete pt;
    points.clear();
}

void RRTTree::init(const T &start)
{
	clear();
	
	Point<T> *p = new Point(start, NULL);
	//	FIXME: throw exception if start isn't valid?
	points.push_back(p);
}

void RRTTree::addPath(Planning::Path<T> &path, Point<T> *dest, const bool rev)
{
	//	build a list of Points between @dest and the receiver's root Point
	int n = 0;
	list<Point<T> *> points;
	while (dest)
	{
		if (rev) {
			points.push_back(dest);
		} else {
			points.push_front(dest);
		}
		dest = dest->parent();
		++n;
	}
	
	//	add the points in @points to the given Path
	path.points.reserve(path.points.size() + n);
	for (Point<T> *pt : points)
	{
		path.points.push_back(pt->state());
	}
}

RRTTree::Point<T> *RRTTree::nearest(T &state)
{
	float bestDistance = -1;
    Point<T> *best = NULL;
    
    for (Point<T> *other : points)
    {
        float d = powf(other.state().distTo(state), 2);	//	magnitude squared
        if (bestDistance < 0 || d < bestDistance)
        {
            bestDistance = d;
            best = other;
        }
    }

    return best;
}

RRTTree::Point<T> *RRTTree::start() const
{
	if (points.empty())
	{
		return NULL;
	}
	
	return points.front();
}

RRTTree::Point<T> *RRTTree::last() const
{
	if (points.empty())
	{
		return NULL;
	}
	
	return points.back();
}


# pragma mark FixedStepRRTTree

RRTTree::Point<T> *FixedStepRRTTree::extend(T target, RRTTree::Point<T> *base)
{
	//	if we weren't given a base point, try to find a close point
	if (!base)
	{
		base = nearest(target);
		if (!base)
		{
			return NULL;
		}
	}
	
	T delta = target - base->target;
	float d = delta.mag();
	
	//	@intermediateState is the new point we will add to the tree
	//	we make sure its distance from @target is <= step
	T intermediateState;
	if (d < step) {
		intermediateState = target;
	} else {
		//	go in the direction of @target, but not as far
		intermediateState = base->state + delta / d * step;
	}
	
	//	abort if the segment isn't valid
	if (!segmentIsValid(base->state, target)) {
		return false;
	}
	
	// Add this point to the tree
	Point<T> *p = new Point<T>(intermediateState, base);
	points.push_back(p);
	return p;
}

bool FixedStepRRTTree::connect(T state)
{
	//	try to reach the goal state
	const unsigned int maxAttemps = 50;
	
	Point<T> *from = NULL;
	for (unsigned int i = 0; i < maxAttemps; ++i)
	{
		Point<T> *newPoint = extend(state, from);
		
		//	there's not a direct path from @from to @state; abort
		if (!newPoint) return false;
		
		//	we found a connection
		if (newPoint->state == state)
		{
			return true;
		}
		
		//	we found a waypoint to use, but we're not there yet
		from = newPoint;
	}
	
	//	we used all of our attempts and didn't find a connection; abort
	return false;
}

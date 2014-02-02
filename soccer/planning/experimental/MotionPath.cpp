
#include "MotionPath.hpp"

using namespace Eigen;


#pragma mark MotionState

MotionState() {}

MotionState(const Eigen::Vector2d &p, const Eigen::Vector2d &v) {
	pos = p;
	vel = v;
}

Vector2d MotionState::evaluateBezierPosition(MotionState &start, MotionState &end, float t) {
	if (t < 0 || t > 1) {
		throw error;	//	FIXME: what do we throw here?
		return;
	}

	return powf(1.0 - t, 3)*start.pos
			+ 3.0*powf(1.0 - t, 2)*start.vel
			+ 3*(1.0 - t)*powf(t, 2)*(-end.vel)
			+ powf(t, 3)*end.pos;
}

Vector2d MotionState::evaluateBezierVelocity(MotionState &start, MotionState &end, float t) {
	if (t < 0 || t > 1) {
		throw error;	//	FIXME: what do we throw here?
		return;
	}

	return -3.0*powf(1 - t, 2)*start.pos
			+ (3*powf(1 - t, 2) - 6*(1 - t)*t)*start.vel
			+ (6*(1 - t)*t - 3*powf(t, 2))*(-end.vel)
			+ 3*powf(t, 2)*end.pos;
}

MotionState MotionState::evaluateBezier(MotionState &start, MotionState &end, float t) {
	return MotionState(
		evaluateBezierPosition(start, end, t),
		evaluateBezierVelocity(start, end, t));
}


#pragma mark MotionPath

MotionPath::MotionPath() {}

MotionPath::MotionPath(MotionState &state) {
	_points.push_back(state);
}

MotionPath::MotionPath(int ptCount, ...) {
	va_list states;
	va_start(states, ptCount);
	_points.reserve(ptCount);	//	increase capacity to @ptCount
	for (int i = 0; i < ptCount; i++) {
		_points.push_back(va_arg(states, MotionState&));
	}
}

MotionPath::size() const {
	return _points.size();
}

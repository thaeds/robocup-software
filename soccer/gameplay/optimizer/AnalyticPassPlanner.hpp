/*
 * AnalyticPassPlanner.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#ifndef ANALYTICPASSPLANNER_HPP_
#define ANALYTICPASSPLANNER_HPP_

#include <iostream>
#include <fstream>
#include <gameplay/Robot.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>
#include <PassState.hpp>
#include <PassConfig.hpp>

#include <motion/planning/rrt.hpp>
#include <framework/Path.hpp>

typedef boost::ptr_vector<PassConfig> PassConfigVector;

namespace AnalyticPassPlanner {
	void generateAllConfigs(const Geometry2d::Point &ballPos, std::set<Gameplay::Robot *> &robots, PassConfigVector &passConfigResult);
	void evaluateConfigs(std::set<Gameplay::Robot *> &_robots, Gameplay::Robot** _opponents, PassConfigVector &passConfigs);
};

#endif /* ANALYTICPASSPLANNER_HPP_ */
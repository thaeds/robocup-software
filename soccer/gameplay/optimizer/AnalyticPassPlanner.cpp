/*
 * AnalyticPassPlanner.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#include <AnalyticPassPlanner.hpp>

using namespace Geometry2d;
using namespace Gameplay;
using namespace std;

#define TIME_TO_AIM_APPROX 0.3 // seconds it takes for the robot to pivot for an aim. Due to aiming time in kick behavior, even if we knew rot accel, this is an approximation.
#define BALL_KICK_AVG_VEL 1.0 // average speed of ball during a kick ((kick vel + end vel) / 2) very conservative due to inaccurate kick speeds and varying dynamics

namespace AnalyticPassPlanner {
	void generateAllConfigs(const Point &ballPos, set<Robot *> &robots, PassConfigVector &passConfigResult){
		Geometry2d::Point goalBallPos = Geometry2d::Point(0.0, Constants::Field::Length);

		Planning::Path path;
		ObstacleGroup og;
		float pathDist, pathTime;

		// for times and distances, use a conservative estimate of 45deg. travel
		Robot* rTmp = *(robots.begin());
		float maxVel = rTmp->packet()->config.motion.deg45.velocity;
		float timeStopToMaxVel = maxVel/ rTmp->packet()->config.motion.deg45.acceleration;
		float timeMaxVelToStop = maxVel / rTmp->packet()->config.motion.deg45.deceleration;
		float distStopToMaxVel = 0.5 * maxVel * timeStopToMaxVel;
		float distMaxVelToStop = 0.5 * maxVel * timeMaxVelToStop;

		BOOST_FOREACH(Robot *r1, robots){
			if(!r1->visible()){continue;} // don't use invisible robots

			BOOST_FOREACH(Robot *r2, robots){
				if(!r2->visible()){continue;} // don't use invisible robots
				if(r2->id()==r1->id()){continue;} // don't pass to self
				PassConfig* passConfig = new PassConfig();

				// setup calculations
				Point passVec = (r2->pos() - ballPos).normalized();
				float passAngle = passVec.angle();
				Point goalVec = (goalBallPos - r2->pos()).normalized();
				float goalAngle = goalVec.angle();

				// add initial state with no robots having the ball
				passConfig->addPassState(
						PassState(r1, r2, r1->pos(), r2->pos(), r1->angle(), r2->angle(),
						ballPos, PassState::INTERMEDIATE, 0));

				// add state with robot1 at a position to pass to robot2
				Point state2Robot1Pos = ballPos - passVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
				float state2Robot1Rot = passAngle;

				// calculate time
				og = r1->obstacles();
				Point r1pos = r1->pos();
				float r1angle = r1->angle();
				Point r1vel = r1->vel();
				Motion::RRT::Planner planner;
				cout << "Robot ID: " << r1->id() << endl;
				planner.run(r1pos,r1angle,r1vel,ballPos,&og,path); //Segfault here
				cout << "  6b" << endl;

				pathDist = path.length(0);

				if(pathDist < distStopToMaxVel + distMaxVelToStop){
					pathTime = (pathDist/(distStopToMaxVel + distMaxVelToStop))*(timeStopToMaxVel + timeMaxVelToStop);
				}else{
					pathTime = timeStopToMaxVel + timeMaxVelToStop + (pathDist - (distStopToMaxVel + distMaxVelToStop))/maxVel;
				}
				double state2Time = pathTime + TIME_TO_AIM_APPROX;
				//double state2Time = r1->pos().distTo(state2Robot1Pos) / APPROXROBOTVELTRANS;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, r2->pos(), state2Robot1Rot, r2->angle(),
						ballPos, PassState::KICKPASS, state2Time));
				// add state with robot2 receiving ball
				Point state3BallPos = r2->pos();
				Point state3Robot2Pos = state3BallPos + passVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
				float state3Robot2Rot = (passVec * -1.0f).angle();
				double state3Time = state2Time + r2->pos().distTo(state3Robot2Pos) / BALL_KICK_AVG_VEL;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, state3Robot2Pos, state2Robot1Rot, state3Robot2Rot,
						state3BallPos, PassState::RECEIVEPASS, state3Time));

				// add state with robot2 kicking a goal
				Point state4Robot2Pos = state3BallPos - goalVec * (float)(Constants::Robot::Radius + Constants::Ball::Radius);
				float state4Robot2Rot = goalAngle;
				double state4Time = state3Time + state3Robot2Pos.distTo(state4Robot2Pos) / TIME_TO_AIM_APPROX;
				//double state4Time = state3Time + state3Robot2Pos.distTo(state4Robot2Pos) / APPROXROBOTVELTRANS;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, state4Robot2Pos, state2Robot1Rot, state4Robot2Rot,
						state3BallPos, PassState::KICKGOAL, state4Time));

				// add state with ball in goal
				double state5Time = state4Time + state3BallPos.distTo(goalBallPos) / BALL_KICK_AVG_VEL;
				//double state5Time = state4Time + state3BallPos.distTo(goalBallPos) / APPROXBALLVEL;
				passConfig->addPassState(
						PassState(r1, r2, state2Robot1Pos, state4Robot2Pos, state2Robot1Rot, state4Robot2Rot,
						goalBallPos, PassState::INTERMEDIATE, state5Time));

				passConfigResult.push_back(passConfig);
			}
		}
	}

	void evaluateConfigs(set<Robot *> &robots, Robot** opponents, PassConfigVector &passConfigs){
		//
		// Weight configs
		//
		PassState prevState;

		Planning::Path path;Motion::RRT::Planner planner;
		ObstacleGroup og;
		float pathDist, pathTime;

		Robot* rTmp = *(robots.begin());
		float maxVel = rTmp->packet()->config.motion.deg45.velocity;
		float timeStopToMaxVel = maxVel/ rTmp->packet()->config.motion.deg45.acceleration;
		float timeMaxVelToStop = maxVel / rTmp->packet()->config.motion.deg45.deceleration;
		float distStopToMaxVel = 0.5 * maxVel * timeStopToMaxVel;
		float distMaxVelToStop = 0.5 * maxVel * timeMaxVelToStop;

		for(int i=0; i<(int)passConfigs.size(); i++){
			int numInteractions = 0;

			// calculate the total number of opponents that can touch the ball at each intermediate
			// ballPos (where the ball will be waiting for the robot to pivot and pass)
			for(int j=0; j<passConfigs[i].length(); j++){
				PassState thisState = passConfigs[i].getPassState(j);
				for (int i=0; i<Constants::Robots_Per_Team; ++i){
					Robot *opponentR = opponents[i];
					og = opponentR->obstacles();
					Motion::RRT::Planner planner;
					planner.run(opponentR->pos(),opponentR->angle(),opponentR->vel(),thisState.ballPos,&og,path);
					pathDist = path.length(0);
					if(pathDist < distStopToMaxVel + distMaxVelToStop){
						pathTime = (pathDist/(distStopToMaxVel + distMaxVelToStop))*(timeStopToMaxVel + timeMaxVelToStop);
					}else{
						pathTime = timeStopToMaxVel + timeMaxVelToStop + (pathDist - (distStopToMaxVel + distMaxVelToStop))/maxVel;
					}
					if(pathTime < thisState.timestamp){
						numInteractions++;
					}
				}
				if(thisState.stateType == PassState::KICKGOAL){
					break; // do not include the last state with ball in goal.
				}
			}

			// calculate the total number of opponents that currently intersect
			// the ball's path at this instant.
			for(int j=0; j<passConfigs[i].length(); j++){
				PassState thisState = passConfigs[i].getPassState(j);
				Line ballPath(thisState.ballPos,prevState.ballPos);
				for (int i=0; i<Constants::Robots_Per_Team; ++i){
					Robot *opponentR = opponents[i];
					// use 1.5*Radius to give "wiggle room"
					if(ballPath.distTo(opponentR->pos()) < (float)(Constants::Robot::Radius + 1.5*Constants::Ball::Radius)){
						numInteractions++;
					}
				}
				prevState = thisState;
			}

			passConfigs[i].setWeight(numInteractions);
		}

		passConfigs.sort();
	}
}
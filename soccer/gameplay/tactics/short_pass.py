import composite_behavior
import behavior
import skills
import constants
import robocup
import time
import main
import enum
import logging
import math
import role_assignment


class ShortPass(composite_behavior.CompositeBehavior):

    PassLength = 0.5
    KickPower = 13
    KickThreshold = 0.1

    class State(enum.Enum):
        setup = 0
        receive = 1

    def __init__(self):
        super().__init__(continuous=False)

        self.kicker = skills.pivot_kick.PivotKick()
        self.receiver = None

        self.add_state(ShortPass.State.setup, behavior.Behavior.State.running)
        self.add_state(ShortPass.State.receive, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            ShortPass.State.setup,
            lambda: True,
            'immediately')

        self.add_transition(ShortPass.State.setup,
            ShortPass.State.receive,
            lambda: self.kicker.is_done_running(),
            'ball kicked')

        self.add_transition(ShortPass.State.receive,
            behavior.Behavior.State.completed,
            lambda: self.receiver is not None and self.receiver.pos.dist_to(main.ball().pos) < ( constants.Robot.Radius + 0.05 ),
            'receiver gets ball')

        """
        TODO check whether the ball is moving towards or away from the receiver
        """
        self.add_transition(ShortPass.State.receive,
            behavior.Behavior.State.failed,
            lambda: self.receiver is None or self.receiver.pos.dist_to(main.ball().pos) > 1,
            'receiver misses ball')

        self.kicker.kick_power = ShortPass.KickPower
        self.kicker.aim_params['max_steady_ang_vel'] = 8
        self.kicker.aim_params['min_steady_duration'] = 0.05
        self.kicker.max_angle_vel = 3
        self.add_subbehavior(self.kicker, 'kicker', required=True)

        self.movers = [skills.move.Move(), skills.move.Move()]

    def on_enter_setup(self):
        self.kicker.restart()
        for i in range(2):
            mover = self.movers[i]
            self.add_subbehavior(mover, 'mover' + str(i), required=False)
            mover.threshold = 0.025

    def execute_setup(self):
        receive_points = self.get_receive_points()
        target = constants.Field.TheirGoalSegment.center()
        for i in range(len(receive_points)):
            if i < 2:
                self.movers[i].pos = receive_points[i]
                main.system_state().draw_circle(receive_points[i], constants.Robot.Radius, (255,255,255), "Debug")
                if self.movers[i].robot is not None:
                    self.movers[i].robot.face(main.ball().pos)


        receiver_number, self.kicker.target = self.pick_closest_receiver()
        if receiver_number != -1 and self.movers[receiver_number].pos.dist_to(self.movers[receiver_number].robot.pos) > ShortPass.KickThreshold:
            self.kicker.enable_kick = False
        else:
            self.kicker.enable_kick = True

    def on_enter_receive(self):
        # self.remove_subbehavior('kicker')
        self.remove_subbehavior('mover0')
        self.remove_subbehavior('mover1')
        self.add_subbehavior(skills.intercept.Intercept(), 'intercept', required=True)

    def execute_receive(self):
        r = self.subbehavior_with_name('intercept').robot
        if r is not None:
            r.set_dribble_speed(90)
            away_from_ball = (r.pos - main.ball().pos).normalized()
            if r.pos.dist_to(main.ball().pos) < 0.3:
                r.set_world_vel(away_from_ball * 0.5)

    def on_exit_receive(self):
        self.remove_subbehavior('intercept')

    def pick_closest_receiver(self):
        dist0 = self.movers[0].robot.pos.dist_to(self.movers[0].pos) if self.movers[0].robot is not None and self.movers[0].pos is not None else float("inf")
        dist1 = self.movers[1].robot.pos.dist_to(self.movers[1].pos) if self.movers[1].robot is not None and self.movers[1].pos is not None else float("inf")
        if dist0 == float('inf') and dist1 == float('inf'):
            self.receiver = None
            return (-1,constants.Field.TheirGoalSegment.center())
        self.receiver = self.movers[0].robot if dist0 < dist1 else self.movers[1].robot
        return (0 if dist0 < dist1 else 1, self.movers[0].robot.pos if dist0 < dist1 else self.movers[1].robot.pos)

    def get_receive_points(self):
        receive_points = list()
        receiver_arc = robocup.Circle(main.ball().pos, ShortPass.PassLength)
        # main.system_state().draw_circle(receiver_arc.center, receiver_arc.get_radius(), (255,0,0), "Debug")
        angle = 0
        start = -1
        last = -1
        while angle <= math.pi:
            seg = robocup.Segment(main.ball().pos, main.ball().pos + robocup.Point.direction(angle)*ShortPass.PassLength)
            is_open = True
            for robot in main.their_robots():
                if seg.dist_to(robot.pos) <= constants.Robot.Radius:
                    is_open = False
                    break
            if (not is_open) or abs(angle - math.pi) < 0.2:
                # main.system_state().draw_line(seg, (255,0,0), "Debug")
                if start is not -1:
                    if (last-start) > math.pi/6:
                        avg = (start + last) / 2
                        avg_seg = robocup.Segment(main.ball().pos, main.ball().pos + robocup.Point.direction(avg)*ShortPass.PassLength)
                        receive_points.append(main.ball().pos + robocup.Point.direction(avg)*ShortPass.PassLength)
                        # main.system_state().draw_line(avg_seg, (0,255,0), "Debug")
                    start = -1
            elif start is -1:
                start = angle

            last = angle
            
            # main.system_state().draw_line(seg, (0,0,255) if is_open else (255,0,0), "Debug")

            angle += math.pi/30.

        return receive_points


    # def role_requirements(self):
    #     reqs = super().role_requirements()
    #     if 'intercept' in reqs:
    #         for r in role_assignment.iterate_role_requirements_tree_leaves(reqs['intercept']):
    #             r.previous_shell_id = self.receiver.shell_id()
    #     return reqs

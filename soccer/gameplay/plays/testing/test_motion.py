import robocup
import play
import behavior
import skills.move
import constants
import main
import enum


## A demo play written during a teaching session to demonstrate play-writing
# Three robots form a triangle on the field and pass the ball A->B->C->A and so on.
class test_motion(play.Play):

    class State(enum.Enum):
        ## 2 robots get on the corners of a triangle,
        # while a third fetches the ball
        setup = 1

        ## The robots continually pass to each other
        passing = 2


    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running,
                            lambda: True,
                            'immediately')

        # register states
        self.add_state(test_motion.State.setup,
            behavior.Behavior.State.running)

        self.motion_points = [
            robocup.Point(0, constants.Field.Length/2.0),
            robocup.Point(0, constants.Field.Length/4.0),
        ]


    def all_subbehaviors_completed(self):
        # for bhvr in self.all_subbehaviors():
        #     if not bhvr.is_done_running():
        #         return False
        # return True

        return all([bhvr.is_done_running() for bhvr in self.all_subbehaviors()])


    def on_enter_setup(self):
        closestPt = min(self.motion_points,
            key=lambda pt: pt.dist_to(main.ball().pos))

        self.add_subbehavior(skills.move.Move(self.motion_points[0]), 'move1')
        self.add_subbehavior(skills.move.Move(self.motion_points[1]), 'move2')


    def on_exit_setup(self):
        self.remove_all_subbehaviors()

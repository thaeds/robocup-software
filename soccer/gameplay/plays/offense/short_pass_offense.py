import play
import tactics.short_pass
import behavior
import role_assignment
import enum
import main
import constants
import skills

class ShortPassOffense (play.Play):

    CloseToOppGoalField = 1.5

    class State(enum.Enum):
        passing = 0,
        shooting = 1

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(ShortPassOffense.State.passing, behavior.Behavior.State.running)
        self.add_state(ShortPassOffense.State.shooting, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            ShortPassOffense.State.passing,
            lambda: True,
            'immediately')

        self.add_transition(ShortPassOffense.State.passing,
            ShortPassOffense.State.shooting,
            lambda: main.ball().pos.y > constants.Field.Length - ShortPassOffense.CloseToOppGoalField,
            'ball close to opp goal')

        self.add_transition(ShortPassOffense.State.shooting,
            ShortPassOffense.State.passing,
            lambda: main.ball().pos.y < constants.Field.Length - ShortPassOffense.CloseToOppGoalField,
            'ball far from opp goal')

        self.prev_receiver = None

        self.pass_behavior = None

    def on_enter_passing(self):
        self.pass_behavior = tactics.short_pass.ShortPass()

        self.add_subbehavior(self.pass_behavior, 'pass', required=True)

        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.failed,
            lambda: self.pass_behavior.state is behavior.Behavior.State.failed,
            'pass failed')

    def execute_passing(self):
        if self.pass_behavior.is_done_running():
            self.prev_receiver = self.pass_behavior.receiver
            self.pass_behavior.restart()

    def on_exit_passing(self):
        self.remove_subbehavior('pass')
        self.prev_receiver = self.pass_behavior.receiver

    def on_enter_shooting(self):
        kicker = skills.pivot_kick.PivotKick()
        kicker.target = constants.Field.TheirGoalSegment
        self.add_subbehavior(kicker, 'kicker', required = True)

    def on_exit_shooting(self):
        self.remove_subbehavior('kicker')
        self.prev_receiver = None

    @classmethod
    def handles_goalie(self):
        return True

    def role_requirements(self):
        reqs = super().role_requirements()
        if self.prev_receiver is not None:
            for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
                req.previous_shell_id = None
            if 'pass' in reqs and 'kicker' in reqs['pass']:
                for req in role_assignment.iterate_role_requirements_tree_leaves(reqs['pass']['kicker']):
                    req.previous_shell_id = self.prev_receiver.shell_id()
            if 'kicker' in reqs:
                for req in role_assignment.iterate_role_requirements_tree_leaves(reqs['kicker']):
                    req.previous_shell_id = self.prev_receiver.shell_id()

        return reqs

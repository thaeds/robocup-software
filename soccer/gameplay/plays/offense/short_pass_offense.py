import play
import tactics.short_pass
import behavior
import role_assignment

class ShortPassOffense (play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        self.prev_receiver = None

        self.pass_behavior = tactics.short_pass.ShortPass()

        self.add_subbehavior(self.pass_behavior, 'pass', required=True)

        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.failed,
            lambda: self.pass_behavior.state is behavior.Behavior.State.failed,
            'pass failed')

    def execute_running(self):
        if self.pass_behavior.is_done_running():
            self.prev_receiver = self.pass_behavior.receiver
            self.pass_behavior.restart()

    @classmethod
    def handles_goalie(self):
        return True

    def role_requirements(self):
        reqs = super().role_requirements()
        if self.prev_receiver is not None:
            for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
                req.previous_shell_id = None
            if 'kicker' in reqs['pass']:
                for req in role_assignment.iterate_role_requirements_tree_leaves(reqs['pass']['kicker']):
                    req.previous_shell_id = self.prev_receiver.shell_id()

        return reqs

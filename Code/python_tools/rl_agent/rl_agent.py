from rl_agent.agent import Agent
import numpy as np
from rl_agent.utils.common import defaultdict2


class TabularAgent(Agent):
    """
    Base tabular RL agent class. Provides:
     - Q(s,a) table, TabularQ type
     - An epsilon-greedy exploration policy

        >>> agent = TabularAgent(env)
    """

    def __init__(self, env, gamma=0.99, epsilon=0):
        super().__init__(env)
        self.gamma = gamma
        self.epsilon = epsilon
        self.Q = TabularQ(env)

    def pi_eps(self, state, info):
        """
        Epsilon-greedy policy. Take a random action by ```env.action_space_sample()``` if below epsilon.
         therwise, take the optimal action.
        The optimal action is provided from ```TabularQ.get_optimal_action(state)```.
        """
        if np.random.rand() < self.epsilon:
            return self.env.action_space_sample()
        else:
            return self.Q.get_optimal_action(state)


def _masked_actions(action_space, mask):
    return [a for a in range(action_space.n) if mask[a - action_space.start] == 1]


class TabularQ:
    """
    Create a TabularQ object with: ```Q = TabularQ(env)``` \\
    Access Q-value with: `value = Q[s,a]` \\
    Store Q-value with: `Q[s,a] = q_value`
    """
    def __init__(self, env):
        """
        Initialize the table. It requires a gym environment to know how many actions there are for each state."""
        self._known_masks = {}

        def q_default(s):
            """
            This is not needed currently"""
            if s in self._known_masks:
                return {a: 0 for a in range(self.env.action_space.n) if self._known_masks[s][a - self.env.action_space.start] == 1}
            else:
                return {a: 0 for a in range(self.env.action_space.n)}
        # This dictionary holds every state-action pair with their corresponding Q-value.
        self.q_ = defaultdict2(dict) #(lambda s: q_default(s))
        self.env = env

    def get_optimal_action(self, state, info_s=None):
        """
        Return the optimal action for a given state."""
        actions, Qa = self.get_Qs(state, info_s)
        # index of the action with the highest Q-value
        a_ = np.argmax(np.asarray(Qa) + np.random.rand(len(Qa)) * 1e-8)
        return actions[a_]

    def get_Qs(self, state, info_s=None):
        if info_s is not None and 'mask' in info_s:
            if state not in self._known_masks:
                self._known_masks[state] = info_s['mask']
                # Probably a good idea to check the Q-values are okay...
                avail_actions = _masked_actions(
                    self.env.action_space, info_s['mask'])
                self.q_[state] = {a: self.q_[state][a] for a in avail_actions}

        (actions, Qa) = zip(*self.q_[state].items())
        return tuple(actions), tuple(Qa)

    def _chk_mask(self, s, a):
        if s in self._known_masks:
            mask = self._known_masks[s]
            if mask[a - self.env.action_space.start] == 0:
                raise Exception(
                    f" Invalid action. You tried to access Q[{s}, {a}], however the action {a} has been previously masked and therefore cannot exist in this state. The mask for {s} is mask={mask}.")

    def __getitem__(self, state_comma_action):
        """
        Return the Q-value for a given state and action.
        - param: state_comma_action: (state, action)
        - return: a float value

        """
        s, a = state_comma_action
        self._chk_mask(s, a)
        return self.q_[s][a]

    def __setitem__(self, state_comma_action, q_value):
        """
        Set the Q-value for a given state and action.
        - param: state_comma_action: (state, action)
        - q_value: float value

        Then the self.q_ becomes:
        {(x1,y1): {a1: q1, a2:q2}, (x2,y2): {a1: q3}}
        """
        s, a = state_comma_action # (x,y), a
        self._chk_mask(s, a)
        self.q_[s][a] = q_value # q_ = {(x1,y1): {a1: q1, a2:q2}, (x2,y2): {a1: q3}}

    def to_dict(self):
        """
        This helper function converts the known Q-values to a dictionary. This function is only used for
        visualization purposes in some of the examples.

        :return: A dictionary ``q`` of all known Q-values of the form ``q[s][a]``
        """
        # Convert to a regular dictionary
        d = {}
        for s, Qs in self.q_.items():
            inner_dict = {}
            for a, Q in Qs.items():
                inner_dict[a] = Q
            d[s] = inner_dict
        return d


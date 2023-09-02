from rl_agent.rl_agent import TabularAgent

class QAgent(TabularAgent):
    """
    Implement the Q-learning agent (SB18, Section 6.5)
    Note that the Q-datastructure already exist, as do helper functions useful to compute an epsilon-greedy policy.
    You can access these as

    > self.Q[s,a] = 31 # Set a Q-value.

    See the TabularAgent class for more information.
    """
    def __init__(self, env, gamma=1.0, alpha=0.5, epsilon=0.1):
        self.alpha = alpha
        super().__init__(env, gamma, epsilon)

    def pi(self, s, k, info=None): 
        """
        Return current action using epsilon-greedy exploration. You should look at the TabularAgent class for ideas.
        """
        # TODO: 1 lines missing.
        action = self.pi_eps(s, info=info)
        # raise NotImplementedError("Implement the epsilon-greedy policy here.")
        return action

    def train(self, s, a, r, sp, done=False, info_s=None, info_sp=None): 
        """
        Implement the Q-learning update rule, i.e. compute a* from the Q-values.
        As a hint, note that self.Q[sp,a] corresponds to q(s_{t+1}, a) and
        that what you need to update is self.Q[s, a] = ...

        You may want to look at self.Q.get_optimal_action(state) to compute a = argmax_a Q[s,a].
        """
        # TODO: 3 lines missing.
        if not done:
            a_star = self.Q.get_optimal_action(sp, info_sp)
        self.Q[s, a] += self.alpha * (r + self.gamma * (0 if done else self.Q[sp, a_star]) - self.Q[s,a])
        # raise NotImplementedError("Update the Q[s,a]-values here.")

    def __str__(self):
        return f"QLearner_{self.gamma}_{self.epsilon}_{self.alpha}"
import typing

class Agent:
    def __init__(self, env):
        self.env = env
    def pi(self, state):
        """
        Evaluate the Agent's policy (i.e., compute the action the agent want to take) at time step ``k`` in state ``s``.
        
        This correspond to the environment being in a state evaluating :math:`x_k`, and the function should compute the next
        action the agent wish to take:
                
        .. math::
            u_k = \mu_k(x_k)
        
            
        """
        return self.env.action_space_sample()

    def train(self, state, action, reward, state_prime, done=False):
        """
        This method will be called in the general train() function.
        The implementation of this method is in the child class.
        """
        pass    

def train(env,
          agent=None,
          experiment_name=None,
          num_episodes=1,
          verbose=True,
          reset=True,
          max_steps=1e10,
          max_runs=None,
          return_trejectory=True,
          resume_stats=None,
          log_interval=1,
          ):
    raise NotImplementedError
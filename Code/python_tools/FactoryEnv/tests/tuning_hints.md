# Hyperparameters

the max_ep_steps should give a relatively larger value, so that the agent can explore more at the beginning. Otherwise, it will never be able to reach the goal and therefore never had a chance to get positive reward.

if gamma_e gives a large number, i.e. 3.0, it will penalize too much on the agent for being away from the path, which will lead to failure of training.

lagging_frac can push the agent to move. Larger numbers, i.e. 20 can make the learning converge faster and eariler
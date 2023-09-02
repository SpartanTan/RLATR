from utils.common import defaultdict2
from rl_agent import TabularQ

q = defaultdict2(dict)
s = (1,2)
a = 1
q_value = 5
a2 = 2
q_value2 = 6
# q[s][a] = q_value
# q[s][a2] = q_value2
# print(q)

qtable = TabularQ(None)
qtable[s,a] = q_value
qtable[s,a2] = q_value2
print(qtable[s,a2])
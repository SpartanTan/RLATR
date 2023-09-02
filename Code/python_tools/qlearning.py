from rl_agent.q_agent import QAgent
from map.Nona2Grid import Nona2Grid
import matplotlib.pyplot as plt


if __name__ == "__main__":
    map_dir = "./map/config/nona_description.json"
    path_dir = "./map/config/nodes_tuve.json"
    grid = Nona2Grid(map_dir, path_dir, grid_resolution=0.2) # ndarrary 40 * 65, upside down
    grid.render()

    # agent = QAgent(env, gamma=0.9, alpha=0.1, epsilon=0.1)

    plt.show()

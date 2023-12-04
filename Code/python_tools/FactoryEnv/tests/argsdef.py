import argparse

def strtobool(val):
    """
    Convert a string representation of truth to true (1) or false (0).
    
    True values are 'y', 'yes', 't', 'true', 'on', and '1'.
    False values are 'n', 'no', 'f', 'false', 'off', and '0'.
    Raises ValueError if 'val' is anything else.
    """
    val = val.lower()
    if val in ('y', 'yes', 't', 'true', 'on', '1'):
        return 1
    elif val in ('n', 'no', 'f', 'false', 'off', '0'):
        return 0
    else:
        raise ValueError("Invalid truth value %r" % (val,))
    
def parse_args(parser:argparse.ArgumentParser):
    parser.add_argument('--which_test', type=int, default=0, help='Which test to perform? 0: manual, 1: benchmark, 2: test agent')
    parser.add_argument('--which_option', type=int, default=0, help='Which test to perform? 0: run, 1: save, 2: load')
    
    parser.add_argument('--agent_dir', type=str, default="agents/", help='Which agent directory to load?')
    parser.add_argument('--agent_file', type=str, default=None, help='Which agent file to load?')
    
    parser.add_argument('--No', type=int, default=8, help='Description for No')
    parser.add_argument('--Nw', type=int, default=8, help='Description for Nw')
    parser.add_argument('--Lp', type=int, default=6, help='Description for Lp')
    parser.add_argument('--mu_r', type=float, default=0.25, help='Description for mu_r')
    parser.add_argument('--sigma_d', type=float, default=0.5, help='Description for sigma_d')
    parser.add_argument('--shift_distance', type=int, default=1, help='Description for shift_distance')
    parser.add_argument('--extend_length', type=float, default=2.0, help='Description for extend_length')
    parser.add_argument('--look_ahead_distance', type=float, default=0.5, help='Description for look_ahead_distance')
    parser.add_argument('--target_finishing_time', type=int, default=30, help='Description for target_finishing_time')
    parser.add_argument('--without_walls', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    parser.add_argument('--how_many_points_forward', type=int, default=2, help='Description for how_many_points_forward')
    parser.add_argument('--max_ep_steps', type=int, default=500, help='Description for max_ep_steps')
    
    parser.add_argument('--dt', type=float, default=0.1, help='time step')
    parser.add_argument('--wheel_radius', type=float, default=0.125, help='Description for wheel_radius')
    parser.add_argument('--track_width', type=float, default=0.48, help='Description for track_width')
    parser.add_argument('--atr_linear_vel_max', type=float, default=0.2, help='Description for atr_linear_vel_max')
    

    parser.add_argument('--full_plot', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    parser.add_argument('--rangefinder', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    
    parser.add_argument('--distance_threshold', type=float, default=0.8, help='Description for distance_threshold')
    parser.add_argument('--sensor_angle', type=int, default=240, help='Description for sensor_angle')
    parser.add_argument('--nsectors', type=int, default=20, help='Description for nsectors')
    parser.add_argument('--num_points_on_walls', type=int, default=5, help='Description for num_points_on_walls')
    parser.add_argument('--num_points_on_obstacles', type=int, default=5, help='Description for num_points_on_obstacles')
    parser.add_argument('--narrow_angle', type=float, default=60.0, help='Description for narrow_angle')
    parser.add_argument('--angle_inbetween', type=float, default=120.0, help='Description for angle_inbetween')
    parser.add_argument('--resolution', type=float, default=6.0, help='Description for resolution')
    parser.add_argument('--Sr', type=float, default=1.0, help='Description for Sr')

    return parser
import numpy as np

def reward_calc(Lambda, recent_speeds, params, low_speed_duration, index, previous_trajectory_index, obstacles_obs, course_error, eta, total_reward):
    r_oa = 0.0
    r_pf = 0.0
    r_lagging = 0.0
    r_exists = - Lambda * (2 * params.reward_param.alpha_r + 1)
    den = 0.0
    num = 0.0
    
    # Update the list of recent speeds
    recent_speeds.append(atr.linear_velocity)  # atr.linear_velocity needs to be passed as a parameter
    if len(recent_speeds) > params.reward_param.vel_window:
        recent_speeds.pop(0)  # Remove the oldest speed

    # Calculate the average speed
    average_speed = sum(abs(speed) for speed in recent_speeds) / len(recent_speeds)

    # Determine if the speed is below the threshold
    if abs(average_speed) < params.reward_param.low_speed_threshold:
        low_speed_duration += params.reward_param.duration
    else:
        low_speed_duration = 0

    speed_factor = max(0, 1 - abs(average_speed) / params.reward_param.low_speed_threshold)
    low_speed_penalty = -params.reward_param.low_speed_penalty_rate * low_speed_duration * speed_factor
    if low_speed_penalty < -params.reward_param.max_low_speed_penalty:
        low_speed_penalty = -params.reward_param.max_low_speed_penalty

    backward_penalty = 0
    if average_speed < 0:
        backward_penalty = params.reward_param.backward_penalty_rate * abs(average_speed)

    if index > previous_trajectory_index:
        trajectory_progress_reward = params.reward_param.trajectory_progress_reward
    else:
        trajectory_progress_reward = 0
    previous_trajectory_index = index

    if len(obstacles_obs) == 0:          
        r_oa -= (1 + np.abs(params.reward_param.gamma_theta * np.pi)) ** -1 * (params.reward_param.gamma_x * 5**2) ** -1
    else:
        for idx, data in enumerate(obstacles_obs):
            num += ((1 + np.abs(params.reward_param.gamma_theta * data[1])) ** -1) * ((params.reward_param.gamma_x * np.maximum(data[0], 0.1)**2) ** -1)
            den += (1 + np.abs(params.reward_param.gamma_theta * data[1])) ** -1
        den = 1
        r_oa = - num/den
        r_oa /= params.reward_param.r_oa_frac

    r_pf = -1 + (1.0 * np.cos(course_error)+1) * (np.exp(-params.reward_param.gamma_e * np.abs(eta[1])) + 1)
    r_lagging = - abs(eta[0]) * params.reward_param.lagging_frac

    if not params.path_param.without_anything:
        reward = Lambda * r_pf + (1 - Lambda) * r_oa + r_exists + r_lagging
    else:
        reward = r_pf + r_exists + r_lagging

    total_reward += reward
    return r_pf, r_oa, low_speed_penalty, reward, total_reward

def reward_function(params):
    '''
    Example of rewarding the agent to follow the center line and maintain speed
    while navigating the track.
    
    Parameters:
    - params (dict): Dictionary containing information about the car's current state,
                     such as track width, distance from center, and speed.

    Returns:
    - reward (float): Calculated reward value based on the car's behavior.
    '''
    
    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    speed = params['speed']

    # Calculate 3 markers that are increasingly further away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward = 1
    elif distance_from_center <= marker_2:
        reward = 0.5
    elif distance_from_center <= marker_3:
        reward = 0.1
    else:
        reward = 1e-3  # likely crashed/close to off track

    # Speed Adjustment
    speed_threshold = 2
    if speed < speed_threshold:
        reward = reward + 0.5
    else:
        reward = reward + 1.5

    return reward

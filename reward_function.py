import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        #import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
        # Handle the case where the end index is before the start index
            if end < start:
            # Calculate the effective end index by adding the array length
                end += array_len
            else:
                pass

            # Generate a list of indexes within the cyclical range
            index_list = [(index + array_len) % array_len for index in range(start, end)]

            return index_list


        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-3.53491, -0.04164, 1.61149, 0.10619],
                        [-3.55675, -0.21069, 1.71125, 0.09961],
                        [-3.55707, -0.38238, 1.67072, 0.10276],
                        [-3.53697, -0.55561, 1.52052, 0.11469],
                        [-3.49643, -0.72944, 1.52052, 0.11739],
                        [-3.43486, -0.9028, 1.52052, 0.12099],
                        [-3.35126, -1.07443, 1.52052, 0.12555],
                        [-3.2408, -1.24205, 1.52052, 0.13203],
                        [-3.09482, -1.40053, 1.52052, 0.14171],
                        [-2.90173, -1.53673, 2.57282, 0.09184],
                        [-2.69142, -1.66153, 2.95218, 0.08284],
                        [-2.46704, -1.77659, 3.36727, 0.07489],
                        [-2.23059, -1.88358, 3.7, 0.07014],
                        [-1.98413, -1.98478, 3.57343, 0.07456],
                        [-1.72805, -2.081, 3.28969, 0.08315],
                        [-1.4671, -2.16955, 3.05037, 0.09034],
                        [-1.20509, -2.2486, 2.69674, 0.10149],
                        [-0.94278, -2.31739, 2.39377, 0.11329],
                        [-0.6805, -2.37521, 2.14241, 0.12536],
                        [-0.41851, -2.42062, 2.14241, 0.12411],
                        [-0.15713, -2.45177, 2.14241, 0.12287],
                        [0.10323, -2.46679, 2.14241, 0.12173],
                        [0.36182, -2.46167, 2.14241, 0.12072],
                        [0.61738, -2.43153, 2.14241, 0.12011],
                        [0.86774, -2.37053, 2.29607, 0.11223],
                        [1.11206, -2.28261, 2.48337, 0.10456],
                        [1.34993, -2.17145, 2.62932, 0.09986],
                        [1.58084, -2.03922, 2.73758, 0.0972],
                        [1.80402, -1.88733, 2.85063, 0.0947],
                        [2.0187, -1.71722, 2.97376, 0.09211],
                        [2.22411, -1.53054, 3.09996, 0.08954],
                        [2.41956, -1.32905, 3.22038, 0.08717],
                        [2.60438, -1.11458, 3.17381, 0.08921],
                        [2.77795, -0.88886, 2.9885, 0.09528],
                        [2.93977, -0.65364, 2.75156, 0.10376],
                        [3.08906, -0.41037, 2.51576, 0.11345],
                        [3.22504, -0.16048, 2.2733, 0.12515],
                        [3.346, 0.09494, 2.06897, 0.13659],
                        [3.45022, 0.3545, 1.811, 0.15445],
                        [3.53547, 0.61666, 1.56556, 0.17609],
                        [3.59876, 0.87954, 1.37574, 0.19654],
                        [3.63683, 1.14068, 1.37574, 0.19182],
                        [3.64596, 1.39682, 1.37574, 0.18631],
                        [3.6231, 1.64414, 1.37574, 0.18054],
                        [3.56276, 1.87678, 1.37574, 0.1747],
                        [3.45845, 2.08546, 1.37574, 0.16958],
                        [3.30602, 2.25616, 1.47172, 0.1555],
                        [3.11932, 2.38743, 1.61327, 0.14147],
                        [2.90971, 2.4815, 1.79562, 0.12795],
                        [2.68556, 2.54267, 1.94177, 0.11966],
                        [2.45193, 2.57365, 2.09578, 0.11245],
                        [2.21253, 2.57714, 2.23747, 0.10701],
                        [1.96999, 2.55525, 2.40107, 0.10142],
                        [1.7263, 2.51031, 2.56207, 0.09672],
                        [1.4829, 2.44428, 2.74008, 0.09204],
                        [1.24089, 2.35912, 2.94846, 0.08701],
                        [1.00109, 2.25694, 3.19414, 0.08161],
                        [0.76419, 2.13999, 3.38606, 0.07802],
                        [0.53101, 2.01072, 2.93631, 0.0908],
                        [0.3026, 1.87201, 2.93631, 0.09101],
                        [0.08052, 1.72679, 2.93631, 0.09037],
                        [-0.14071, 1.5909, 2.93631, 0.08842],
                        [-0.36534, 1.46352, 2.93631, 0.08794],
                        [-0.59482, 1.34778, 2.93631, 0.08753],
                        [-0.83071, 1.24732, 3.16669, 0.08096],
                        [-1.07197, 1.15995, 2.84109, 0.09032],
                        [-1.31838, 1.08541, 2.50257, 0.10287],
                        [-1.56977, 1.0237, 2.25335, 0.11488],
                        [-1.82595, 0.97498, 1.99057, 0.131],
                        [-2.08646, 0.93957, 1.73632, 0.15141],
                        [-2.33148, 0.89234, 1.3, 0.19195],
                        [-2.5637, 0.83151, 1.3, 0.18466],
                        [-2.78046, 0.75545, 1.3, 0.17671],
                        [-2.97929, 0.66347, 1.3, 0.16852],
                        [-3.15664, 0.55471, 1.3, 0.16003],
                        [-3.3077, 0.42856, 1.3, 0.15139],
                        [-3.4153, 0.28143, 1.40895, 0.12937],
                        [-3.48907, 0.12323, 1.50645, 0.11587]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 13
        FASTEST_TIME = 7
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 13  # seconds (time that is easily done by model)
        FASTEST_TIME = 7  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
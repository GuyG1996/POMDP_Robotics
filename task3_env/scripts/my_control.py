#!/usr/bin/env python3
import rospy
import os
from task3_env.srv import navigate, navigateResponse, pick, pickResponse, place, placeResponse, info, infoResponse  #
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
nav_rate = [0.5, 0.7, 0.8, 0.85, 0.95]
accuracy_rate = [0.99, 0.7, 0.8, 0.8, 0.95]
placing_probabillity = {
    "green": [0.1, 0.05, 0.8, 0.05],
    "blue": [0.7, 0.1, 0.1, 0.1],
    "black": [0.25, 0.25, 0.25, 0.25],
    "red": [0.25, 0.25, 0.25, 0.25]
}
placed_balls = 0
pick_calls = 0
nav_calls = 0


def terminate():
    if (nav_calls >= 8 or pick_calls >= 6):
        return True
    return False

def get_next_target_index():
    # Find the maximum value and its index
    max_value = None
    max_index = None

    for key, value in placing_probabillity.items():
        current_max = max(value)
        if max_value is None or current_max > max_value:
            max_value = current_max
            max_index = value.index(max_value)
    return max_index


def get_next_toy_color(index):
    # Get the probabilities at the specified index
    probabilities_at_index = [probabilities[index]
    for probabilities in placing_probabillity.values()]
    
    # Create a list of colors sorted by their probabilities at the specified index
    sorted_colors = sorted(placing_probabillity.keys(), key=lambda x: placing_probabillity[x][index], reverse=True)

    # Return the two colors with the highest probabilities
    return sorted_colors[:2]

def update_dict_fail(color,index):
    global placing_probabillity
    values = placing_probabillity[color]
    original_value = values[index]
    values[index] = 0.1
    remaining_indices = [i for i in range(len(values)) if i != index and values[i] != 0]
    if(len(remaining_indices) > 0):
        spread_value = (original_value - 0.1) / len(remaining_indices)
        for i in remaining_indices:
            values[i] += spread_value
    
# given color and index, we will update the placing probapillty of the color to zero (so we wont look for it again) and update the given index at each color to zero, while remmembering to distribute the index probabillity to the other indexes.
def update_dict_found(color, index):
    global placing_probabillity
    # Set the probability of the given color to zero
    placing_probabillity[color] = [0] * len(placing_probabillity[color])

    # update the other colors
    for key in placing_probabillity.keys():
        if (key != color):
            # Get the probability value of the colorat the index before setting it to zero
            color_prob_before_zero = placing_probabillity[key][index]
            # update the index to zero
            placing_probabillity[key][index] = 0
            # count the number of indexes that are not zero:
            num_non_zero = sum(
                1 for prob in placing_probabillity[key] if prob != 0)
            if(num_non_zero>0):
                distribute_value = color_prob_before_zero / num_non_zero
                # updating the values of the current key
                for i in range(len(placing_probabillity[key])):
                    # Add the distributed value to non-zero indexes
                    if placing_probabillity[key][i] > 0:
                        placing_probabillity[key][i] += distribute_value

def navigate_to_location(location):
    global nav_calls
    if(pick_calls >= 6 or nav_calls >= 8):
        return False
    nav_calls += 1
    rospy.wait_for_service('/navigate')
    try:
        navigate_service = rospy.ServiceProxy('/navigate', navigate)
        response = navigate_service(location)
        return response.success
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed:", e)

def pick_toy(toy_type):
    global pick_calls
    if(pick_calls >= 6 or nav_calls >= 8):
        return False
    pick_calls += 1
    rospy.wait_for_service('/pick')
    try:
        pick_service = rospy.ServiceProxy('/pick', pick)
        response = pick_service(toy_type)
        rospy.loginfo("Picking up toy of type {} completed with success: {}".format(toy_type, response.success))
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call to pick failed: {}".format(e))

def place_toy():
    rospy.wait_for_service('/place')
    try:
        place_service = rospy.ServiceProxy('/place', place)
        response = place_service()
        rospy.loginfo("Placing toy completed with success: {}".format(response.success))
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call to place failed: {}".format(e))




if __name__ == "__main__":
    rospy.loginfo("Starting my_control node...")
    rospy.init_node('my_control')
    
    rospy.loginfo("checking if all the balls are at the baby or we did all of our steps")
    while placed_balls != 4 and (not terminate()):
        print(placing_probabillity)
        next_dest = get_next_target_index()
        #rospy.loginfo("trying to go to index %d", next_dest) 
        success_nav = navigate_to_location(next_dest)
        #rospy.loginfo("navigation succeded? %s" , success_nav) 
        if not success_nav or nav_rate[next_dest] < 0.8:
            #rospy.loginfo("trying to go again to index %d", next_dest) 
            success_nav = navigate_to_location(next_dest)
        picks = get_next_toy_color(next_dest)
        first_pick = picks[0]
        second_pick = picks[1]
        success_pick1 = pick_toy(first_pick)
        if not success_pick1:
            update_dict_fail(first_pick, next_dest)
            success_pick2 = pick_toy(second_pick)
            if not success_pick2:
                update_dict_fail(second_pick, next_dest)
                continue
            else:
                update_dict_found(second_pick, next_dest)
        else:
            update_dict_found(first_pick, next_dest)
        success_nav = navigate_to_location(4)
        if not success_nav:
            success_nav = navigate_to_location(4)
        success_place = place_toy()
        if(success_place):
            placed_balls += 1
    rospy.logwarn("GAME OVER!")
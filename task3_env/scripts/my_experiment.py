#!/usr/bin/env python3
import rospy
import os
import subprocess
import time
import signal
from task3_env.srv import navigate, navigateResponse, pick, pickResponse, place, placeResponse, info, infoResponse

total_rewards = 0

def navigate_to_location(location):
    rospy.wait_for_service('/navigate')
    try:
        navigate_service = rospy.ServiceProxy('/navigate', navigate)
        response = navigate_service(location)
        return response.success
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed:", e)

def get_info():
    rospy.wait_for_service('/info')
    try:
        info_service = rospy.ServiceProxy('/info', info)
        response = info_service()
        return response.internal_info
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed:", e)


if __name__ == "__main__":
    rospy.loginfo("Starting my_experiment node...")
    rospy.init_node('my_experiment')
    experiment_counter = 0
    exp_out_array = []

    while experiment_counter < 10:
        # Call navigate to location 4
        
        success_nav = navigate_to_location(4)

        # Call navigate to location 4 again
        success_nav = navigate_to_location(4)
        
        # Terminate skills_server.py
        os.system("rosnode kill skills_server_node")
        if(experiment_counter > 0):
            process_skills.kill()
        

        # Launch skills_server.py
        # Run the command in a separate process
        command = "rosrun task3_env skills_server.py"
        process_skills = subprocess.Popen(command, shell=True)
        
        # Launch my_control.py
        os.system("rosrun task3_env my_control.py")

        # Call /info service and save output string into exp_out_array
        info_output = get_info()
        exp_out_array.append(info_output)
        
        experiment_counter += 1
       
        rospy.sleep(2) #sleep before the next experiment

    rospy.logwarn("experiment over! ")

    # Iterate through each string in exp_out_array
    for result in exp_out_array:
        # Find the index of "total rewards:"
        index = result.find("total rewards:")
        if index != -1:
            # Extract the substring starting from the index of "total rewards:"
            # and convert it to an integer
            reward_str = result[index + len("total rewards:"):]
            reward = int(reward_str.strip())
            total_rewards += reward

    # Calculate the average reward
    average_reward = total_rewards / len(exp_out_array)

    try:
        with open("experiment_output.txt", 'w') as f:
            # Write data to the file
            for item in exp_out_array:
                f.write("%s\n" % item)
            f.write("Average Rewards of all experiments: {}\n".format(average_reward))

    except Exception as e:
        print("Error:", e)

    # Terminate process_skills if it's defined
    if process_skills is not None:  
        process_skills.kill()
    
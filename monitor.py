# Module: Paul Glotfelter & Yousef Emam
# 02/26/2019
# Description: Oversees commands sent from MATLAB api to the robots.
# Run using 'python3 monitor.py -port 1884 -host 192.168.1.8' on Robotarium 
import json
import argparse
import vizier.node as vizier_node
import time
import queue
import numpy as np
from barriersQP import create_robust_barriers
import time

# Constants
MAX_QUEUE_SIZE = 100

def create_node_descriptor(end_point,api_link,tracker_link,all_ids):
    """Returns a node descriptor for the robot based on the end_point.

    The server_alive link is for the robot to check the MQTT connection periodically.

    Args:
        end_point (str): The ID of the robot.

    Returns:
        dict: A node descriptor of the vizier format for the robot.

    Example:
        >>> node_descriptor(1)

    """
    node_descriptor = \
        {
            'end_point': end_point,
            'links': {'/{}'.format(id) : {'type': 'STREAM'} for id in all_ids},
            'requests':
            [
                {
                    'link': api_link,
                    'type': 'STREAM',
                    'required': False
                },
                {
                    'link': tracker_link,
                    'type': 'STREAM',
                    'required': False
                },
            ]
        }    

    return node_descriptor

def main():

    # Parse Command Line Arguments
    parser = argparse.ArgumentParser()
    # parser.add_argument("node_descriptor", help=".json file node information")
    parser.add_argument("-port", type=int, help="MQTT Port", default=8080)
    parser.add_argument("-host", help="MQTT Host IP", default="localhost")

    args = parser.parse_args()

    #endpoint = "matlab_api"
    endpoint = "monitor"
    api_link = 'matlab_api/inputs'
    # TODO: Get the ids somehow
    all_ids = list(range(50))
    tracker_link = 'overhead_tracker/all_robot_pose_data'

    # Ensure that Node Descriptor File can be Opened
    node_descriptor = create_node_descriptor(endpoint,api_link,tracker_link,all_ids)
    # try:
    #     f = open(args.node_descriptor, 'r')
    #     node_descriptor = json.load(f)
    #     f.close()
    # except Exception as e:
    #     print(repr(e))
    #     print("Couldn't open given node file " + args.node_descriptor)
    #     return -1

    # Start the Node
    

    started = False
    robot_node = None
    while (not started):
        node = vizier_node.Node(args.host, args.port, node_descriptor)
        try:
            node.start()
            started = True
        except Exception as e:
            node.stop()
        
        time.sleep(1)

    # Get the links for Publishing/Subscribing
    # publishable_link = list(node.publishable_links)[0]

    # Queues for STREAM links
    inputs = node.subscribe(api_link)
    poses = node.subscribe(tracker_link)
    vel_commands = np.zeros([2,len(all_ids)])
    last_input_msg = None
    message_timestamp = 0

    robust_barriers = create_robust_barriers()

    # Main Loop
    while True:
        # Process input commands
        input_msg = None
        poses_msg = None
        # Make sure that the queue has few enough messages
        # if(inputs.qsize() > MAX_QUEUE_SIZE):
            # logger.critical('Queue of motor messages is too large.')

        # TODO: Add Timeout in poses.get()
        try:
            poses_msg = poses.get(timeout=0.1)
        except:
            poses_msg = None
        
        try:
            # Clear out the queue
            while True:
                poses_msg = poses.get_nowait()
        except queue.Empty:
            pass
        
        try:
            # Clear out the queue
            while True:
                input_msg = inputs.get_nowait()
        except queue.Empty:
            pass

        if(poses_msg is not None) :
            try:
                poses_msg = json.loads(poses_msg.decode(encoding='UTF-8'))
            except Exception as e:
                # logger.warning('Got malformed JSON motor message ({})'.format(poses_msg))
                # logger.warning(e)
                # Set this to None for the next checks
                poses_msg = None
        
        if(input_msg is not None):
            try:
                input_msg = json.loads(input_msg.decode(encoding='UTF-8'))
            except Exception as e:
                # logger.warning('Got malformed JSON motor message ({})'.format(input_msg))
                # logger.warning(e)
                # Set this to None for the next checks
                input_msg = None

        # If we got a valid JSON input msg, look for appropriate commands
        if(input_msg is not None):
            last_input_msg = input_msg
            message_timestamp = time.time()

        else: 
            input_msg = last_input_msg
            
        if (poses_msg is not None) and (time.time() - message_timestamp) < 0.1:
            active_ids = input_msg['inputs'].keys() # IDs that got inputs
            last_poses_msg = poses_msg
            tracked_ids = poses_msg.keys()
            # TODO: Watchout for instances when an active_id isn't being tracked!
            
            active_poses = np.zeros([3,len(tracked_ids)])
            pre_inputs = np.zeros([2,len(tracked_ids)]) # Pre Barrier Inputs
            left_leds = np.zeros([3,len(tracked_ids)]) # Pass the LED Inputs 
            right_leds = np.zeros([3,len(tracked_ids)]) # Pass the LED Inputs 
            inactive_poses = np.zeros([2,len(tracked_ids)])
            active_ids_2 = [] # IDs that are tracked and active
            inactive_ids = [] # IDs that are tracked and did not get inputs

            counters = [0,0]
            
            
            
            print('Input Message:')
            print(input_msg['inputs'])
            
            for id in tracked_ids:
                if id in active_ids and input_msg['inputs'][id] != [0,0]:
                    active_ids_2.append(id)
                    active_poses[:,counters[0]] = [poses_msg[id]['x'],poses_msg[id]['y'],poses_msg[id]['theta']]
                    pre_inputs[:,counters[0]] = input_msg['inputs'][id]
                    left_leds[:,counters[0]] = input_msg['leds'][id][0]
                    right_leds[:,counters[0]] = input_msg['leds'][id][1]
                    counters[0] += 1
                else:
                    inactive_ids.append(id)
                    inactive_poses[:,counters[1]] = [poses_msg[id]['x'],poses_msg[id]['y']]
                    counters[1] += 1

            error_ids = [] # Active ids that are not being tracked
            for id in active_ids:
                if id not in active_ids_2:
                    error_ids.append(id)    
                    
            # Reshapes & Calls the Barrier Function
            # Truncates Back To Right-Size
            pre_inputs = pre_inputs[:,:counters[0]]
            active_poses = active_poses[:,:counters[0]]
            if len(inactive_ids) == 0:
                inactive_poses = np.array([])
            else:
                inactive_poses = inactive_poses[:,:counters[1]]

            # Post-Barrier Inputs
            start = time.time()
            vel_commands = robust_barriers(pre_inputs,active_poses,inactive_poses)
            print('Time for BC: {}'.format(time.time() - start))

            # Iterate & Publish
            for i in range(len(active_ids_2)):
                id = active_ids_2[i]
                input_dict = {'v':vel_commands[0,i],'w':vel_commands[1,i],'left_led':left_leds[:,i].tolist(),'right_led':right_leds[:,i].tolist()}
                node.publish(endpoint+'/{}'.format(id), json.dumps(input_dict))
                

        else: # Didn't get Poses, publish 0 velocities as a safety measures.
            # Iterate & Publish
            for i in range(len(all_ids)):
                id = all_ids[i]
                input_dict = {'v':0,'w':0,'left_led':[0,0,0],'right_led':[0,0,0]}
                node.publish(endpoint+'/{}'.format(id), json.dumps(input_dict))
        
    node.stop()


if(__name__ == "__main__"):
    main()

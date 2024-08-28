#!/usr/bin/env python3

#std python packages 
from math import sqrt
from typing import List
import numpy as np 

#ros2 packages
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult 
from nav_msgs.msg import OccupancyGrid  
from geometry_msgs.msg import PoseStamped
from rclpy import node
from rclpy.duration import Duration
from rclpy.executors import time
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformListener
from tf2_ros import TransformBroadcaster 
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import matplotlib.pyplot as plt 

import asyncio 

class Navigator(BasicNavigator):
    def __init__(self, node_name='basic_navigator', namespace=''):
        super().__init__(node_name, namespace)
        
        self.create_subscription(Clock, 'clock', self.time_callback, 10)

        self.create_subscription(OccupancyGrid, "global_costmap/costmap", self.costmap_callback, 10)
        
        self.create_subscription(TFMessage, "tf", self.tf_callback, 10) 
        self.create_subscription(TFMessage, "tf_static", self.tf_callback, 10)
        
        self.tf_buffer = TFMessage()
        
        self.navigation_frame_id = "map"
    
    def time_callback(self, time_msg):
        self.time = Clock()
        self.time = time_msg
        self.get_logger().debug(f"{self.time.clock.sec}")

    def get_clock_time(self):
        time_msg = Time(seconds=self.time.clock.sec, nanoseconds=self.time.clock.nanosec) 
        return time_msg.to_msg()

    def initiate_pickup(self): 
        pass


    def costmap_callback(self, msg):
        # print(msg.info)
        self.costmap = (np.array(msg.data).reshape((msg.info.height, msg.info.width,1)))
        self.cmap_height = msg.info.height
        self.cmap_width = msg.info.width
        self.cmap_origin = msg.info.origin
        self.cmap_resolution = msg.info.resolution
        
        # adding coordinate layers to costmap, does not suport rotated costmaps 
        costmap_x_position = np.cumsum(np.ones_like(self.costmap),axis=1)*self.cmap_resolution + self.cmap_origin.position.x 
        costmap_y_position = np.cumsum(np.ones_like(self.costmap),axis=0)*self.cmap_resolution + self.cmap_origin.position.y 
        # plt.imshow(np.array(msg.data).reshape((msg.info.height, msg.info.width)))
        # plt.imshow(costmap_x_position+self.costmap)
        # plt.imshow(costmap_y_position+self.costmap)
        # plt.show()

        self.costmap = np.vstack([self.costmap, costmap_x_position, costmap_y_position]).reshape((3, msg.info.height, msg.info.width)).transpose(1,2,0)
        print(self.costmap.shape)
        # plt.imshow(self.costmap[:,:,1], 'plasma')
        # plt.show()
        np.save("sim_costmap.np", self.costmap)

    def tf_callback(self, tf_msg):
        """
        callback function that fills the buffer with the latest transforms, based on the recived oder
        if there is a new transform, it is appended to the list, 
        if it is an exisitng transform, it is overridden with the latest
        """
        if len(self.tf_buffer.transforms) == 0:
            self.tf_buffer = tf_msg
        # print(f"tf recived {len(tf_msg.transforms)}")
        did_tf_frame_match = False
        for tf in tf_msg.transforms:
            frame_id = tf.header.frame_id
            for n, bf_tf  in enumerate(self.tf_buffer.transforms):
                bf_frame_id = bf_tf.header.frame_id
                # print(f"n {n}")
                if bf_frame_id == frame_id and bf_tf.child_frame_id == tf.child_frame_id:
                    self.tf_buffer.transforms[n] = tf 
                    did_tf_frame_match = True
            if not did_tf_frame_match:
                self.tf_buffer.transforms.append(tf)
        # print(f"tfs in buffer {len(self.tf_buffer.transforms)}")
        # print(self.tf_buffer.transforms)

    def get_transform(self, parrent_frame, child_frame):
        """
        get a transform from the transform buffer 
        searces the list of transforms to get a matrching parrent and child frame 
        can only find direct transforms, there is no tree. 
        """
        tf_found = False
        for tf in self.tf_buffer.transforms:
            # print(f"parrent {tf.header.frame_id}, child {tf.child_frame_id}")
            if tf.header.frame_id == parrent_frame and tf.child_frame_id == child_frame:
                tf_found = True
                return tf.transform 
        if not tf_found:
            # return f"no transform from {parrent_frame} to {child_frame}, transforms in buffer {len(self.tf_buffer.transforms)}"
            return False

        

    def generate_waypoints(self, initial_pose : np.ndarray, goal_pose : np.ndarray, item_position : np.ndarray, costmap : np.ndarray) -> List[PoseStamped]:        
        # generate a list of trial points from a radius of the item position
        # find the point on the radius that is closest to the specified initial_pose and does - 
        # and the line generated from this point does not cross the cspace 
        
        trial_resolution = 360 #the amount of points on the radius to try 

        attack_angle = 200 # the angle to the line perpendicular to the tangent of the trial value,

        radius = 2 # the radius from the item to generate trial points 

        cspace_radius = radius + 3 # the area to look for overlapping with cspace 
        
        pickup_path_length = 3 # the pickup_path_length ie how far the robot should go in a straight line 

        path_line_with = 0.2 # the with of the line used to check for overlapping with the cspace 

        points_on_radius = np.linspace(0,359,trial_resolution)

        initial_headings = points_on_radius+attack_angle

        trial_values = np.c_[points_on_radius, initial_headings]

        selected_cspace_idx = (costmap[:,:,1] - item_position[0])**2 + (costmap[:,:,2]-item_position[1])**2 <= cspace_radius 
        
        
        print(selected_cspace_idx.shape)
        print(selected_cspace_idx[:,0].shape)

        costmap_reduced = np.zeros_like(costmap)
        costmap_reduced[selected_cspace_idx] = costmap[selected_cspace_idx]

        # costmap[selected_cspace_idx[1],selected_cspace_idx[0],:] 
               
        print(costmap_reduced.shape)
        costs = []
        line_params_list = []

        for val in trial_values:
            heading_angle = (val[1]/180)*np.pi
            angle_from_item = (val[0]/180) *np.pi
            known_intersect = np.array([np.cos(angle_from_item), np.sin(angle_from_item)])*radius + item_position 

            # ax.scatter(known_intersect[0], known_intersect[1])
            line_params =  [np.sin(heading_angle) /np.cos(heading_angle),  
                            known_intersect[1] - np.sin(heading_angle) /np.cos(heading_angle) *known_intersect[0] ] # y = ax+b, as [a, b]
            
            line_params_list.append(line_params)
            # validate the line, ie. dose it pass through the seleceted config space correctly 
            


            line_idx_1 = (costmap_reduced[:,:,2] > (costmap_reduced[:,:,1] * line_params[0] + line_params[1] - (np.sqrt(1 + line_params[0]**2)*path_line_with) ))
            line_idx_2 = (costmap_reduced[:,:,2] < (costmap_reduced[:,:,1] * line_params[0] + line_params[1] + (np.sqrt(1 + line_params[0]**2)*path_line_with) ))
            
            line_idx = line_idx_1 == line_idx_2

            
            costmap_idx = costmap_reduced[:,:,0] > 0 

            # plt.imshow(costmap_idx.astype(np.uint8) + line_idx.astype(np.uint8) == 1)
            # plt.show()

            line_config_overlap = np.sum(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8) > 1)
            
            if line_config_overlap == 0: 
                cost = np.sqrt(np.sum((initial_pose-known_intersect)**2)) + line_config_overlap
                plt.imshow(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8))
                plt.show()
                costs.append(cost)
            else:
                costs.append(np.inf)

        print(costs)

        best_values = trial_values[np.argmin(np.array(costs))]

        heading_angle = best_values[1]/180 * np.pi 
        angle_from_item = best_values[0]/180 * np.pi

        coordinate = np.array([np.cos(angle_from_item), np.sin(angle_from_item)])*radius + item_position
        
        pre_grasp_pose = self.cast_waypoint(coordinate[0], 
                                            coordinate[1], 
                                            np.cos(heading_angle/2), 
                                            np.sin(heading_angle/2))
        

        post_grasp_coordinate = coordinate + np.array([np.cos(heading_angle), np.sin(heading_angle)])*pickup_path_length 



        post_grasp_pose = self.cast_waypoint(post_grasp_coordinate[0],
                                             post_grasp_coordinate[1],
                                             np.cos(heading_angle/2),
                                             np.sin(heading_angle/2))

        self.get_logger().info(f"generated pre and post grasp poses sucsessfully with coordinates pre: {coordinate}, post: {post_grasp_coordinate}")

        unstamped_poses = [pre_grasp_pose, post_grasp_pose]

        return unstamped_poses 

    def cast_waypoint(self, x, y, w, z):
        waypoint = PoseStamped()
        waypoint.header.frame_id = self.navigation_frame_id 
        waypoint.header.stamp = self.get_clock_time()
        waypoint.pose.position.x = x 
        waypoint.pose.position.y = y 
        waypoint.pose.orientation.w = w
        waypoint.pose.orientation.z = z 

        return waypoint 

def main():
    rclpy.init()

    
    navigator = Navigator()
    item_tf_pub = StaticTransformBroadcaster(node = navigator)
    item_tf_pub_bro = TransformBroadcaster(node = navigator)
    
    # item_tf_sub = TransformListener(buffer=navigator.item_tf_buffer, node = navigator)
    # print(item_tf_buffer.lookup_transform(target_frame="item", source_frame="map", time=Time(seconds=0, nanoseconds=0)))
    
    item_tf = TransformStamped()
    item_tf.header.frame_id = "map"
    item_tf.child_frame_id = "item"
    item_tf.transform.translation.x = 3.0
    item_tf.transform.translation.y = -1.7
    item_tf.transform.translation.z =  1.0 
 
    item_tf_pub.sendTransform(item_tf)

    # item_tf_pub_bro.sendTransform(item_tf)

    time.sleep(1)
    

    #     item_tf.header.stamp = navigator.get_clock().now().to_msg()
    #     item_tf_pub_bro.sendTransform(item_tf)
    #
    #     # try:
    #     #     print(
    #     #         navigator.item_tf_buffer.lookup_transform(source_frame="map", target_frame="item", time=navigator.get_clock().now()))
    #     # except Exception as e: 
    #     #     print(e)
    #     time.sleep(0.5)



    navigator.waitUntilNav2Active()
   

    print(navigator.get_transform("map","item"))
  

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock_time()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    item_tf_pub.sendTransform(item_tf)
    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses
    goal_poses = []
    # goal_pose1 = PoseStamped()
    # goal_pose1.header.frame_id = 'map'
    # goal_pose1.header.stamp = navigator.get_clock_time()
    # goal_pose1.pose.position.x = 3.5
    # goal_pose1.pose.position.y = 3.55
    # goal_pose1.pose.orientation.w = 0.707
    # goal_pose1.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose1)
    #
    #
    #
    # goal_pose2 = PoseStamped()
    # goal_pose2.header.frame_id = 'map'
    # goal_pose2.header.stamp = navigator.get_clock_time()
    # goal_pose2.pose.position.x = -3.5
    # goal_pose2.pose.position.y = 3.55
    # goal_pose2.pose.orientation.w = 0.707
    # goal_pose2.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose2)




    

    # generating the pickup poses ie. pre grasp and post grasp pose. 

    initial_pose_ar = np.array([initial_pose.pose.position.x, initial_pose.pose.position.y])
    
    goal_pose_ar = np.array([1.1,1.1]) #dummy values, not in use yet

    while navigator.costmap.any() == None:
        print("waiting for costmap")
        time.sleep(1)

    costmap_ar = navigator.costmap

    item_pos_transform = navigator.get_transform("map", "item")
    while not item_pos_transform:
        item_pos_transform = navigator.get_transform("map", "item")

    item_transform_ar = np.array([item_pos_transform.translation.x, item_pos_transform.translation.y])

    print(item_transform_ar)

    pickup_waypoints = navigator.generate_waypoints(initial_pose=initial_pose_ar, goal_pose=goal_pose_ar, costmap=costmap_ar, item_position=item_transform_ar)

    print(f"pregrasp waypoint at {[pickup_waypoints[0].pose.position.x, pickup_waypoints[0].pose.position.y]}")

    for n,wp in enumerate(pickup_waypoints):
        stamped_wp = wp
        stamped_wp.header.stamp = navigator.get_clock_time()
        stamped_wp.header.frame_id = "map"
        pickup_waypoints[n] = stamped_wp
        goal_poses.append(stamped_wp)
    
        




    # additional goals can be appended
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock_time()
    goal_pose3.pose.position.x = 3.5
    goal_pose3.pose.position.y = -3.75
    goal_pose3.pose.orientation.w = 0.707
    goal_pose3.pose.orientation.z = 0.707
    goal_poses.append(goal_pose3)
   #  goal_pose3 = PoseStamped()
   #  goal_pose3.header.frame_id = 'map'
   #  goal_pose3.header.stamp = navigator.get_clock_time()
   #  goal_pose3.pose.position.x = -3.6
   #  goal_pose3.pose.position.y = -4.75
   #  goal_pose3.pose.orientation.w = 0.707
   #  goal_pose3.pose.orientation.z = 0.707
   #  goal_poses.append(goal_pose3)
   # 
   #  goal_pose4 = PoseStamped()
   #  goal_pose4.header.frame_id = 'map'
   #  goal_pose4.header.stamp = navigator.get_clock_time()
   #  goal_pose4.pose.position.x = 4.6
   #  goal_pose4.pose.position.y = 3.75
   #  goal_pose4.pose.orientation.w = 0.707
   #  goal_pose4.pose.orientation.z = 0.707
   #  goal_poses.append(goal_pose4)


    # sanity check a valid path exists
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

    navigator.goThroughPoses(goal_poses, behavior_tree="/home/a/work/straight_line_pickup/ros_mir_ws/src/mir_robot/mir_navigation/behavior_trees/navigate_through_poses_bt.xml")

    i = 0
    while not navigator.isTaskComplete():
        # print(navigator.get_transform("map","item"))

        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=200.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose4.pose.position.x = -5.0
                goal_pose4.pose.position.y = -4.75
                goal_pose4.pose.orientation.w = 0.707
                goal_pose4.pose.orientation.z = 0.707
                navigator.goThroughPoses([goal_pose4])

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    # exit(0)


if __name__ == '__main__':
    main()

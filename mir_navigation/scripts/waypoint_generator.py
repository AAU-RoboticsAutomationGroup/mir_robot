from typing import List
from geometry_msgs.msg import PoseStamped 
from matplotlib.rcsetup import ls_mapper
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  
import numpy as np
import matplotlib.pyplot as plt 


class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", self.costmap_callback, 10)
        

    def costmap_callback(self, msg):
        print(msg.info)
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

    def display_costmap(self, costmap : np.ndarray):
        plt.imshow((costmap + np.array([0, np.abs(np.min(costmap[:,:,1])), np.abs(np.min(costmap[:,:,2])) ]))
                    *np.array([1/np.max(costmap[:,:,0]), 
                               1/np.max(costmap[:,:,1] - np.min(costmap[:,:,1])), 
                               1/np.max(costmap[:,:,2] - np.min(costmap[:,:,2]))]) 
                   )
        plt.show()
    
    def generate_waypoints(self, initial_pose : np.ndarray, goal_pose : np.ndarray, item_position : np.ndarray, costmap : np.ndarray) -> List[PoseStamped]:        
        # generate a list of trial points from a radius of the item position 
        # for each point on the radius try diffrent initial headings 
        # this generates a list containing where on the radius and the initial headings ie 360*360 combinations 
        points_on_radius = np.cumsum(np.ones((360,1)))        
        initial_headings = points_on_radius+40 
        
        trial_values = np.c_[points_on_radius, initial_headings]
        
        self.display_costmap(costmap)


        radius = 3

        # for val in trial_values:
        #     print(val)

        cspace_radius = radius + 3
        
        selected_cspace_idx = (costmap[:,:,1] - item_position[0])**2 + (costmap[:,:,2]-item_position[1])**2 <= cspace_radius 
        
        
        print(selected_cspace_idx.shape)
        print(selected_cspace_idx[:,0].shape)

        costmap_reduced = np.zeros_like(costmap)
        costmap_reduced[selected_cspace_idx] = costmap[selected_cspace_idx]

        # costmap[selected_cspace_idx[1],selected_cspace_idx[0],:] 
               
        print(costmap_reduced.shape)
        
        plt.imshow(costmap_reduced[:,:,0])
        plt.show()
        
         

        costs = []
        
        fig, ax = plt.subplots()
        
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
            threshold = 0.1


            line_idx_1 = (costmap_reduced[:,:,2] > (costmap_reduced[:,:,1] - threshold) * line_params[0] + line_params[1] ) 
            line_idx_2 = (costmap_reduced[:,:,2] < (costmap_reduced[:,:,1] + threshold) * line_params[0] + line_params[1] )
            
            line_idx = line_idx_1 == line_idx_2

            
            costmap_idx = costmap_reduced[:,:,0] > 0 

            # plt.imshow(costmap_idx.astype(np.uint8) + line_idx.astype(np.uint8) == 1)
            # plt.show()

            line_config_overlap = np.sum(line_idx.astype(np.uint8) + costmap_idx.astype(np.uint8) > 1)
            
            if line_config_overlap == 0: 
                cost = np.sqrt(np.sum((initial_pose-known_intersect)**2))
                
                costs.append(cost)
            else:
                costs.append(np.inf)

        print(costs)

        best_values = trial_values[np.argmin(np.array(costs))]

        heading_angle = best_values[1]/180 * np.pi 
        angle_from_item = best_values[0]/180 * np.pi

        coordinate = np.array([np.cos(angle_from_item), np.sin(angle_from_item)])*radius + item_position

        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.pose.position.x = coordinate[0]
        pre_grasp_pose.pose.position.y = coordinate[1]
        pre_grasp_pose.pose.orientation.z = np.sin(heading_angle/2)
        pre_grasp_pose.pose.orientation.w = np.cos(heading_angle/2)
        
        pickup_path_length = 3

        post_grasp_coordinate = coordinate + np.array([np.cos(heading_angle), np.sin(heading_angle)])*pickup_path_length 

        post_grasp_pose = PoseStamped()
        post_grasp_pose.pose.position.x = post_grasp_coordinate[0]
        post_grasp_pose.pose.position.y = post_grasp_coordinate[1]
        post_grasp_pose.pose.orientation.z = np.sin(heading_angle/2)
        post_grasp_pose.pose.orientation.w = np.cos(heading_angle/2)

        self.get_logger().info(f"generated pre and post grasp poses sucsessfully with coordinates pre: {coordinate}, post: {post_grasp_coordinate}")

        unstamped_poses = [pre_grasp_pose, post_grasp_pose]

        return unstamped_poses 



            


            
               

            # lspc = np.linspace(-3,3,1000)
            # print(f"{known_intersect}")
            # ax.plot(lspc, line_params[0]*lspc+line_params[1]) 

        # ax.set_xlim(-3, 3)
        # ax.set_ylim(-3, 3)
        #
        # plt.show()
        
                

        pass





def main():
    rclpy.init()
    waypoint_generator = WaypointGenerator()
    waypoint_generator.generate_waypoints(np.array([0,0]),0,item_position=np.array([2,-2]),costmap=np.load("sim_costmap.np.npy"))
    rclpy.spin(waypoint_generator)


if __name__ == "__main__":
    main()


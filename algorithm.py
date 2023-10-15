from queue import PriorityQueue
import numpy as np
import math
import matplotlib.pyplot as plt
from collections import deque


class algorithm:
    def __init__(self,id,goal_region: tuple,start: list,timestep,obst_pos: list):
        self.id = id # id is unique to each vehicle (1,2,3)
        self.start = start
        self.goal_region = goal_region # goal location
        self.timestep = timestep
        self.obst_pos = obst_pos # List of all the obstacle positions and their lengths
        
    # We round the vehicle states to keep the calculations simple    
    def delivery_robot(self,state,ul,ur):
        # Performing the update
        # state = [x, y, theta] 
        q_new = []
        q_new.append(round(state[0] + (0.5/2)*(ul+ur)*math.cos(round(state[2] + (0.5/2)*(ur-ul)*self.timestep,1))*self.timestep,2))
        q_new.append(round(state[1] + (0.5/2)*(ul+ur)*math.sin(round(state[2] + (0.5/2)*(ur-ul)*self.timestep,1))*self.timestep,2))
        q_new.append(round(state[2] + (0.5/2)*(ur-ul)*self.timestep,1))
        return q_new
    def car(self,state,v,omega):
        # Performing the update
        # state = [x, y, theta, delta]
        q_new = [] 
        q_new.append(round(state[0] + v*math.cos(round(state[2] + v*(math.tan(round(state[3] + omega*self.timestep,1))/2.8)*self.timestep,1))*self.timestep,2))
        q_new.append(round(state[1] + v*math.sin(round(state[2] + v*(math.tan(round(state[3] + omega*self.timestep,1))/2.8)*self.timestep,1))*self.timestep,2))
        q_new.append(round(state[2] + v*(math.tan(round(state[3] + omega*self.timestep,1))/2.8)*self.timestep,1))
        q_new.append(round(state[3] + omega*self.timestep,1))
        return q_new
    def truck(self,state,v,omega):
        # Performing the update
        # state = [x, y, theta0, phi, theta1] 
        q_new = []
        q_new.append(round(state[0] + v*math.cos(round(state[2] + v*(math.tan(round(state[3] + omega*self.timestep,1))/3)*self.timestep,1))*self.timestep,2))
        q_new.append(round(state[1] + v*math.sin(round(state[2] + v*(math.tan(round(state[3] + omega*self.timestep,1))/3)*self.timestep,1))*self.timestep,2))
        q_new.append(round(state[2] + v*(math.tan(round(state[3] + omega*self.timestep,1))/3)*self.timestep,1))
        q_new.append(round(state[3] + omega*self.timestep,1))
        q_new.append(round(state[4] + v*(math.sin(round(state[2] + v*(math.tan(round(state[3] + omega*self.timestep,1))/3)*self.timestep,1)-state[4])/5)*self.timestep,1))
        return q_new

    def astar(self):
        start = self.start
        obst_boundary = self.obst_pos
        goal_region = self.goal_region
        
        # Action/control input set for different vehicles 
        if self.id == 1:
            action_inputs = [(10,10),(10,-10),(10,0),(-10,0),(0,10),(0,-10),(0,0)]
        elif self.id == 2 or self.id == 3:
            action_inputs = [(10,2),(-10,2),(10,-2),(-10,-2),(10,0.5),(-10,0.5),(10,-0.5),(-10,-0.5),(10,0),(-10,0),(0,0)]
        search_tree = {}
        
        # Priority queue helps us in choosing the next node with least distance cost
        queue = PriorityQueue()
        g = {}
        dist = {} #this is the total cost function that adds up all the costs 
        # prev dict keeps track of parent node for each node
        prev = {}
        visited = [] # this list contains the list of all visited nodes
        # The start node distance from iteself is set to 0 and added to queue
        g[tuple(start)] = 0
        # Initial cost function accoring to the vehicle type.
        if self.id == 1:
            dist[tuple(start)] = 1*g[tuple(start)] + 3*math.hypot((goal_region[0]-start[0]),(goal_region[1]-start[1]))+abs(start[2])
        elif self.id == 2 or self.id == 3:
            dist[tuple(start)] = 1*g[tuple(start)] + 3*math.hypot((goal_region[0]-start[0]),(goal_region[1]-start[1]))+abs(start[2])+0*start[3]
        queue.put((dist[tuple(start)], start))

        while (not queue.empty()) and len(search_tree)<50000:
            # Remove the vertex with least distance from the queue
            current_vertex = queue.get()[-1]
            search_tree[tuple(current_vertex)] = []
            
            # Check if we are in the goal region
            if goal_region[0]-0.5<=current_vertex[0]<=goal_region[0]+0.5 and goal_region[1]-0.5<=current_vertex[1]<=goal_region[1]+0.5:
                visited.append(current_vertex)
                break
            # If the current vertex or node is already visited we move on
            if current_vertex in visited:
                pass
            else:
                #Increasing the action inputs as we approach the goal
                if math.hypot((goal_region[0]-current_vertex[0]),(goal_region[1]-current_vertex[1]))<20: 
                    if self.id == 1:
                        action_inputs = [(10,10),(10,-10),(10,0),(-10,0),(0,10),(0,-10),(0,0),(10,5),(10,-5),(-10,-5),(-10,5),(5,5)]
                    elif self.id == 2 or self.id == 3:
                        action_inputs = [(10,2),(-10,2),(10,-2),(-10,-2),(10,0.5),(-10,0.5),(10,-0.5),(-10,-0.5),(10,0),(-10,0),(0,0)\
                                         ,(5,2),(-5,2),(5,-2),(-5,-2),(5,0.5),(-5,0.5),(5,-0.5),(-5,-0.5),(5,0),(-5,0)\
                                         ,(5,0.1),(-5,0.1),(5,-0.1),(-5,-0.1),(5,1),(-5,1),(5,-1),(-5,-1),(5,1.5),(-5,1.5)]
                
                # Add the current vertex to list of visited nodes
                visited.append(current_vertex)

                for action in action_inputs:
                    # Get the new vehicle state/node for each vehicle type
                    if self.id == 1:
                        q_new = self.delivery_robot(current_vertex,action[0],action[1])
                    elif self.id == 2:
                        q_new = self.car(current_vertex,action[0],action[1])
                    elif self.id == 3:
                        q_new = self.truck(current_vertex,action[0],action[1])
                    
                    
                    # Checking if the new state of the vehicle collides with an obstacle
                    if (not (obst_boundary[0][0]-20<=q_new[0]<=obst_boundary[0][0]+obst_boundary[0][2] and \
                            obst_boundary[0][1]<=q_new[1]<=obst_boundary[0][1]+obst_boundary[0][3])) and \
                            (not (obst_boundary[0][0]-10<=q_new[0]+30*math.cos(q_new[2])<=obst_boundary[0][0]+obst_boundary[0][2]\
                                    and obst_boundary[0][1]<=q_new[1]+30*math.sin(q_new[2])<=obst_boundary[0][1]+obst_boundary[0][3])) and \
                                (not (obst_boundary[1][0]<=q_new[0]<=obst_boundary[1][0]+obst_boundary[1][2]\
                                    and obst_boundary[1][1]<=q_new[1]<=obst_boundary[1][1]+obst_boundary[1][3]+10)) and \
                                    (not (obst_boundary[1][0]<=q_new[0]+30*math.cos(q_new[2])<=obst_boundary[1][0]+obst_boundary[1][2]\
                                    and obst_boundary[1][1]<=q_new[1]+30*math.sin(q_new[2])<=obst_boundary[1][1]+obst_boundary[1][3])) and \
                                        (not (obst_boundary[2][0]<=q_new[0]<=obst_boundary[2][0]+obst_boundary[2][2]\
                                    and obst_boundary[2][1]<=q_new[1]<=obst_boundary[2][1]+obst_boundary[2][3]+5)) and\
                                        (not (obst_boundary[2][0]<=q_new[0]+30*math.cos(q_new[2])<=obst_boundary[2][0]+obst_boundary[2][2]\
                                              and obst_boundary[2][1]<=q_new[1]+30*math.sin(q_new[2])<=obst_boundary[2][1]+obst_boundary[2][3])):
                        # If the new state is already visited we pass
                        if q_new in visited:
                            pass
                        else:
                            # the new state costs are calculated depending on the vehicle type and the state is added to the queue
                            search_tree[tuple(current_vertex)].append(q_new)
                            g[tuple(q_new)] = g[tuple(current_vertex)] + math.hypot((q_new[0]-current_vertex[0]),(q_new[1]-current_vertex[1]))
                            h = math.hypot((goal_region[0]-q_new[0]),(goal_region[1]-q_new[1]))
                            if self.id == 1:
                                dist[tuple(q_new)] = 2*g[tuple(q_new)] + 3*h + 1*(abs(q_new[2]-current_vertex[2]))+1*(abs(q_new[2])) 
                            elif self.id == 2:
                                dist[tuple(q_new)] = 2*g[tuple(q_new)] + 3*h + 0*(abs(q_new[2]-current_vertex[2])%(2*math.pi))+2*(abs(q_new[2]))+0*(abs(q_new[3]-current_vertex[3]))
                            elif self.id == 3:
                                dist[tuple(q_new)] = 2*g[tuple(q_new)] + 3*h + 0*(abs(q_new[2]-current_vertex[2])%(2*math.pi))+0*(abs(q_new[2]))+0*(abs(q_new[3]-current_vertex[3]))
                            queue.put((dist[tuple(q_new)],q_new))
                            prev[tuple(np.array(q_new))] = current_vertex
                 
        last_config = visited[-1]
        last_position = (last_config[0],last_config[1])
        # We will check if the search is a success or failure
        if goal_region[0]-0.5<=last_position[0]<=goal_region[0]+0.5 and goal_region[1]-0.5<=last_position[1]<=goal_region[1]+0.5:
            
            print("Successfully reached the goal")
        else:
            print("No path to the goal is found")

        final_path = deque() 
        print("number of visited/searched configurations",len(visited))
        print(last_config)
        # Using while loop we traverse back to start location by looking up the parent for
        # each node starting from the goal location.
        while last_config != start:
            next_config = prev[tuple(last_config)]
            final_path.append(next_config)
            last_config = next_config
        print("Number of configurations to the final path is",len(final_path))
        # The output final path contains the path from start to goal in reverse order.
        return visited, final_path
    
    def plot_vehicle(self,x,y,theta,ax):
        car_struct = np.array([[15,5],[15,-5],[-15,-5],[-15,5]])
        Rot = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        rotated_car = np.transpose(np.dot(Rot,car_struct.T))
        rotated_car+=np.array([x+15*math.cos(theta),y+15*math.sin(theta)]) # Rear axle center formula
        rotated_car = np.vstack((rotated_car,rotated_car[0,:]))

        drawing = ax.fill(rotated_car[:,0], rotated_car[:,1],facecolor="green", edgecolor='black')
        
        return drawing
    def plot_truck(self,x,y,theta0,theta1,ax):
        car_struct = np.array([[15,5],[15,-5],[-15,-5],[-15,5]])
        Rot_car = np.array([[np.cos(theta0),-np.sin(theta0)],[np.sin(theta0),np.cos(theta0)]])
        rotated_car = np.transpose(np.dot(Rot_car,car_struct.T))
        rotated_car+=np.array([x+15*math.cos(theta0),y+15*math.sin(theta0)])
        rotated_car = np.vstack((rotated_car,rotated_car[0,:]))

        trailer_struct = np.array([[15,5],[15,-5],[-15,-5],[-15,5]])
        Rot_trailer = np.array([[np.cos(theta1),-np.sin(theta1)],[np.sin(theta1),np.cos(theta1)]])
        rotated_trailer = np.transpose(np.dot(Rot_trailer,trailer_struct.T))
        # Both car and trailer are located at their respective rear axles
        rotated_trailer+=np.array([x-20*math.cos(theta1),y-20*math.sin(theta1)])
        rotated_trailer = np.vstack((rotated_trailer,rotated_trailer[0,:]))
        drawing_car = ax.fill(rotated_car[:,0], rotated_car[:,1],facecolor="green", edgecolor='black')
        drawing_trailer = ax.fill(rotated_trailer[:,0], rotated_trailer[:,1],facecolor="yellow", edgecolor='black')
        return drawing_car, drawing_trailer
    
    def simulate(self,ax):
        if self.id == 1:
            visited, path = self.astar()
            for i in path: # Plot the path of the vehicle
                plt.plot(i[0],i[1],'co--', markersize=1)
            for i in reversed(path):
                vehicle = self.plot_vehicle(i[0],i[1],i[2],ax)
                plt.draw()
                plt.pause(0.00001)
                for v in vehicle:
                    v.remove()
            last_pos = path[0]
            last_vehicle = self.plot_vehicle(last_pos[0],last_pos[1],last_pos[2],ax)
            plt.draw()
            plt.show()
        elif self.id == 2:
            visited, path = self.astar()
            for i in path: # Plot the path of the vehicle
                plt.plot(i[0],i[1],'co--', markersize=1)
            for i in reversed(path):
                vehicle = self.plot_vehicle(i[0],i[1],i[2],ax)
                plt.draw()
                plt.pause(0.00001)
                for v in vehicle:
                    v.remove()
            last_pos = path[0]
            last_vehicle = self.plot_vehicle(last_pos[0],last_pos[1],last_pos[2],ax)
            plt.draw()
            plt.show()
        elif self.id == 3:
            visited, path = self.astar()
            for i in path: # Plot the path of the vehicle
                plt.plot(i[0],i[1],'co--', markersize=1)
            for i in reversed(path):
                car, trailer = self.plot_truck(i[0],i[1],i[2],i[4],ax)
                plt.draw()
                plt.pause(0.00001)
                for c in car:
                    c.remove()
                for t in trailer:
                    t.remove()
            last_pos = path[0]
            last_car, last_trailer = self.plot_truck(last_pos[0],last_pos[1],last_pos[2],last_pos[4],ax)
            plt.draw()
            plt.show()
        
from matplotlib import pyplot as plt
import numpy as np
import plot
from algorithm import *

if __name__ == "__main__":
    fig, ax = plt.subplots()
    xlim = np.array([-18, 305])
    ylim = np.array([-5, 180]) 
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    map = plot.create_map(ax) # create a map and place obstacles
    vehic_obst1 = plot.add_vehicleobst(ax,[5,20],70,30)
    vehic_obst2 = plot.add_vehicleobst(ax,[145,20],60,20)
    obst1 = plot.add_obstacle(ax,[60,85],70,50)

    #----------------------------------------------
    # Uncomment the three lines below to simulate the car
    # CAR 
    # goal = (110,30)
    # planner = algorithm(2,goal,[40,130,-1.57,0],0.1,[[60,85,70,50],[5,20,70,30],[145,20,60,20]])
    # planner.simulate(ax)

    #------------------------------------------------------------------
    # Uncomment the three lines below to simulate the truck
    # TRUCK 
    goal = (110,30)
    planner = algorithm(3,goal,[30,130,-1.57,0,0],0.1,[[60,85,70,50],[5,20,70,30],[145,20,60,20]])
    planner.simulate(ax)
    #----------------------------------------------------------------------------
    # Uncomment the three lines below to simulate the delivery robot
    # DELIVERY ROBOT 
    # goal = (110,30)
    # planner = algorithm(1,goal,[40,130,-1.57],0.1,[[60,85,70,50],[5,20,70,30],[145,20,60,20]])
    # planner.simulate(ax)
    


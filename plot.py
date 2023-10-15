from matplotlib import pyplot as plt

def create_map(ax):
    map = plt.Rectangle((0, 0), 300, 175, facecolor="None", edgecolor='black')
    ax.add_artist(map)
    return map

def add_obstacle(ax,position,length,width):
    obstacle = plt.Rectangle((position[0], position[1]), length,width,facecolor="black", edgecolor='black')
    ax.add_artist(obstacle)
    return obstacle

def add_vehicleobst(ax,position,length,width):
    vehicle_obstacle = plt.Rectangle((position[0], position[1]), length,width,facecolor="red", edgecolor='black')
    ax.add_artist(vehicle_obstacle)
    return vehicle_obstacle
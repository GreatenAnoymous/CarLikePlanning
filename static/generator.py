import json
import numpy as np
import random

LF = 2
carWidth = 2
LB = 1
obsRadius = 1.1
yawDesnitation = [0, np.pi/2, np.pi, 3*np.pi/2]
warehouse_rectangles=[(10,40,10,20),(10,40,30,40),(10,40,60,70),(10,40,80,90),(60,90,10,20),(60,90,30,40),(60,90,60,70),(60,90,80,90)]

def read_maps(map_name):
    obstacles = []
    with open(map_name, "r") as file_content:
        lines = file_content.readlines()
        xmax, ymax = (int(lines[1].split()[1]), int(lines[2].split()[1]))
        obstacles = list()
        for y, line in enumerate(lines[4:]):
            for x, char in enumerate(line):
                if char != ".":
                    obstacles.append((x, y))
        return xmax, ymax, obstacles


def agentCollision(state1, state2):
    x1, y1, yaw1 = state1
    x2, y2, yaw2 = state2
    if (x1-x2)**2+(y1-y2)**2 < (2*LF)**2+carWidth**2:
        return True
    return False


def obsCollision(state1, obstacle):
    x1, y1, yaw1 = state1
    x2, y2 = obstacle
    obs = np.zeros((2, 2))
    rot = np.zeros((2, 2))
    rot[0, 0] = np.cos(-yaw1)
    rot[0, 1] = -np.sin(-yaw1)
    rot[1, 0] = np.sin(-yaw1)
    rot[1, 1] = np.cos(-yaw1)
    obs[0, 0] = x2-x1
    obs[1, 1] = y2-y1
    
    rotated_obs = np.dot(obs, rot)
    if (
        rotated_obs[0, 0] > -LB - obsRadius and
        rotated_obs[0, 0] < LF + obsRadius and
        rotated_obs[0, 1] > -carWidth / 2.0 - obsRadius and
        rotated_obs[0, 1] < carWidth / 2.0 + obsRadius
    ):
        return True
    return False


def warehouse_validator(state):
    x,y,yaw=state
    safe_range=1.2*np.sqrt(carWidth**2+(LB+LF)**2)/2
    for obs in warehouse_rectangles:
        xmin,xmax,ymin,ymax=obs
        if x+safe_range>=xmin and x-safe_range<=xmax and y+safe_range>=ymin and y-safe_range<=ymax:
            return False
        
    return True



def save_instance_json(filename, starts, goals, xmax, ymax, obstacles):
    with open(filename, "w") as f:
        data_json = dict()
        data_json["xmax"] = xmax
        data_json["ymax"] = ymax
        data_json["obstacles"] = obstacles
        data_json["starts"] = starts
        data_json["goals"] = goals
        json.dump(data_json, f)


# def generate_random_instance(filename, num_agents, xmax, ymax, obstacles=[], k=1):
#     yawd = 0
#     all_vertices = [(x, y) for x in range(xmax)
#                     for y in range(ymax) if (x, y) not in obstacles]
#     # print(all_vertices)
#     # print(random.sample(all_vertices, size=num_agents, replace=False))
#     starts = random.sample(all_vertices, num_agents)
#     for i in range(num_agents):
#         starts[i] = starts[i][0], starts[i][1], yawd

#     goals = [[] for i in range(num_agents)]

#     for i in range(num_agents):
#         for j in range(k-1):
#             gj = random.choice(all_vertices)
#             gj = gj[0], gj[1], yawd
#             goals[i].append(gj)
#     final_goals = random.sample(all_vertices, num_agents)
#     for i in range(num_agents):
#         goals[i].append((final_goals[i][0], final_goals[i][1], yawd))
#     data = dict()
#     data["xmax"] = xmax
#     data["ymax"] = ymax
#     # data["obstacles"]=obstacles
#     data["starts"] = starts
#     data["goals"] = goals
#     with open(filename, "w") as file:
#         json.dump(data, file, indent=4)

def generate_valid_state(xmax, ymax, obstacles, config):
    while True:
        x = np.random.rand()*xmax
        y = np.random.rand()*ymax
        yaw = random.choice(yawDesnitation)
        valid = True
        for obs in obstacles:
            if obsCollision((x, y, yaw), obs) == True:
                valid = False
                break
        if valid != False:
            for s in config:
                if agentCollision(s, (x, y, yaw)) == True:
                    valid = False
                    break
        if valid:
            return (x, y, yaw)
        

def generate_valid_state_warehouse(xmax, ymax,config):
    while True:
        x = np.random.rand()*xmax
        y = np.random.rand()*ymax
        yaw = random.choice(yawDesnitation)
        valid = True
        if warehouse_validator((x,y,yaw))==False:
            valid=False
        if valid != False:
            for s in config:
                if agentCollision(s, (x, y, yaw)) == True:
                    valid = False
                    break
        if valid:
            return (x, y, yaw)


def generate_valid_config(xmax, ymax, obstacles, num_agents):
    config = []
    while len(config) < num_agents:
        s = generate_valid_state(xmax, ymax, obstacles, config)
        config.append(s)
    return config

def generate_valid_config_warehouse(xmax, ymax, num_agents):
    config = []
    while len(config) < num_agents:
        s = generate_valid_state_warehouse(xmax, ymax, config)
        config.append(s)
    return config


def generate_empty_instances():
    xmax, ymax = 100, 100
    obstacles = []
    agents = [10, 20, 30, 40, 50,60]
    for agent in agents:
        for k in range(30):
            starts = generate_valid_config(xmax, ymax, obstacles, agent)
            goals = generate_valid_config(xmax, ymax, obstacles, agent)
            instance_name = "./instances/one_shot/empty/agents_" + \
                str(agent)+"_"+str(k)+".json"

            save_instance_json(instance_name, starts,
                               goals, xmax, ymax, obstacles)
            print(instance_name)


def generate_empty_demo():
    xmax, ymax = 60, 20
    num_obstacles=10
    obstacles=[]
    print(num_obstacles)
    for i in range(num_obstacles):
        x=np.random.random()*xmax
        y=np.random.random()*ymax
        obstacles.append((x,y))
    agents = [10]
    for agent in agents:
        for k in range(5):
            starts = generate_valid_config(xmax, ymax, obstacles, agent)
            goals = generate_valid_config(xmax, ymax, obstacles, agent)
            instance_name = "./instances/demo/agents_" + \
                str(agent)+"_"+str(k)+".json"

            save_instance_json(instance_name, starts,
                               goals, xmax, ymax, obstacles)
            print(instance_name)


def generate_obstacle_instances():
    xmax, ymax = 100, 100
    obstacles = []
    num_obstacles=50
    print(num_obstacles)
    for i in range(num_obstacles):
        x=np.random.random()*xmax
        y=np.random.random()*ymax
        obstacles.append((x,y))
    agents = [10, 20, 30, 40, 50,60]
    for agent in agents:
        for k in range(30):
            starts = generate_valid_config(xmax, ymax, obstacles, agent)
            goals = generate_valid_config(xmax, ymax, obstacles, agent)
            instance_name = "./instances/one_shot/random/agents_" + \
                str(agent)+"_"+str(k)+".json"

            save_instance_json(instance_name, starts,
                               goals, xmax, ymax, obstacles)
            print(instance_name)

def generate_warehouse_instances():
    xmax, ymax = 100, 100
    obstacles = []
    agents = [10, 20, 30, 40, 50,60]
    for agent in agents:
        for k in range(30):
            starts = generate_valid_config_warehouse(xmax, ymax,agent)
            goals = generate_valid_config_warehouse(xmax, ymax, agent)
            instance_name = "./instances/one_shot/warehouse/agents_" + \
                str(agent)+"_"+str(k)+".json"

            save_instance_json(instance_name, starts,
                               goals, xmax, ymax, obstacles)
            print(instance_name)  


if __name__ == "__main__":
    # generate_empty_instances()
    # generate_obstacle_instances()
    # generate_warehouse_instances()
    generate_empty_demo()
    

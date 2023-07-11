import json
import numpy as np
import random

def read_maps(map_name):
    obstacles=[]
    with open(map_name, "r") as file_content:
        lines = file_content.readlines()
        xmax,ymax =(int(lines[1].split()[1]), int(lines[2].split()[1]))
        obstacles = list()
        for y, line in enumerate(lines[4:]):
            for x, char in enumerate(line):
                if char != ".":
                    obstacles.append((x, y))
        return xmax,ymax, obstacles


def generate_random_instance(filename,num_agents,xmax,ymax,obstacles=[],k=1):
    yawd=0
    all_vertices=[(x,y) for x in range(xmax) for y in range(ymax) if (x,y) not in obstacles]
    # print(all_vertices)
    # print(random.sample(all_vertices, size=num_agents, replace=False))
    starts=random.sample(all_vertices, num_agents)
    for i in range(num_agents):
        starts[i]=starts[i][0],starts[i][1],yawd

    goals=[[] for i in range(num_agents)]
    
    for i in range(num_agents):
        for j in range(k-1):
            gj=random.choice(all_vertices)
            gj=gj[0],gj[1],yawd
            goals[i].append(gj)
    final_goals=random.sample(all_vertices, num_agents)
    for i in range(num_agents):
        goals[i].append((final_goals[i][0],final_goals[i][1],yawd))
    data=dict()
    data["xmax"]=xmax
    data["ymax"]=ymax
    # data["obstacles"]=obstacles
    data["starts"]=starts
    data["goals"]=goals
    with open(filename,"w") as file:
        json.dump(data,file,indent=4)





if __name__=="__main__":
    xmax,ymax, obstacles=read_maps("./maps/random-32-32-10.map")
    generate_random_instance("./testInstanceMultiple.json",2,xmax,ymax,obstacles,5)


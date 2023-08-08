import pygame

import numpy as np

import json
import time
import yaml


carPNG = "./car.png"
obsPNG = "./obs.png"
HEIGHT = 800
WIDTH = 800

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
OBSTACLE_RADIUS = 1


def read_maps_from_yaml(mapYaml):
    with open(mapYaml, 'r') as file:
        data = yaml.safe_load(file)

    dimensions = data['map']['dimensions']
    starts = [agent['start'] for agent in data['agents']]
    goals = [agent['goal'] for agent in data['agents']]
    obstacles = data['map']['obstacles']

    return dimensions[0], dimensions[1], starts, goals, obstacles


def read_maps_from_json(mapJson):
    pass


def read_paths_from_json(pathJson):
    pass


def read_paths_from_yaml(pathYaml):
    with open(pathYaml, 'r') as file:
        data = yaml.safe_load(file)

    paths = []
    for agent, path_data in data['schedule'].items():
        path = [(step['x'], step['y'], step['yaw']) for step in path_data]
        paths.append(path)

    return paths


class Animator:

    def __init__(self, xmax, ymax, obstacles, paths) -> None:
        self.xmax = xmax
        self.ymax = ymax
        self.obstacles = obstacles
        self.paths = paths
        self.carLength = 3
        self.carWidth = 2
        self.deltaT = 0.5  # each point takes 0.5 seconds
        self.carImage = pygame.image.load(carPNG)
        self.scaling = WIDTH/self.xmax
        self.locations = []
        self.carImage=pygame.transform.smoothscale(self.carImage, (int(self.carLength*self.scaling), int(self.carWidth*self.scaling)))
        self.obstacleImage = pygame.image.load(obsPNG)
        self.obstacleImage = pygame.transform.smoothscale(self.obstacleImage, (int(2*OBSTACLE_RADIUS*self.scaling), int(2*OBSTACLE_RADIUS*self.scaling)))
        self.makepsan=0
        for path in self.paths:
            self.locations.append(path[0])
            self.makepsan=max(len(path),self.makepsan)


    def getStateAtTime(self, agentId, t):
        if t >= len(self.paths[agentId]):
            return self.paths[agentId][-1]
        return self.paths[agentId][t]

    def getState(self, lastState, nextState, t):
        xLast, yLast, yawLast = lastState
        xNext, yNext, yawNext = nextState
        if (yawLast - yawNext) > np.pi:
            yawLast = yawLast - 2 * np.pi
        elif (yawNext - yawLast) > np.pi:
            yawLast = yawLast + 2 * np.pi
        xt=(xNext-xLast)*t/self.deltaT+xLast
        yt=(yNext-yLast)*t/self.deltaT+yLast
        yawt=(yawNext-yawLast)*t/self.deltaT+yawLast
        return xt,yt,yawt
        

    def createAnim(self):
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.screen.fill(WHITE)
        pygame.display.set_caption("Moving Objects")
        self.init_settings()
        clock = pygame.time.Clock()
        running = True
        step = 1
        startTime=pygame.time.get_ticks()
        while running:
            print(step)
            self.screen.fill(WHITE)
            self.screen.blit(self.static_surface, (0, 0))
            dt=(pygame.time.get_ticks()-startTime)/1000.
            self.simulate(step,dt)
            if dt>self.deltaT:
                step+=1
                startTime=pygame.time.get_ticks()
            self.drawCars()
            pygame.display.flip()
            clock.tick(30)
            self.screen.blit(self.static_surface, (0, 0))
            # if step>self.makepsan:
            #     break

    def draw_goals(self):
        pass

    def simulate(self,i,dt):
        for k in range(len(self.locations)):
            lastState=self.getStateAtTime(k,i-1)
            nextState=self.getStateAtTime(k,i)
            nextLoc=self.getState(lastState,nextState,dt)
            self.locations[k]=nextLoc


            
            

    def drawCars(self):
        for loc in self.locations:
            x, y, theta = loc
            x=x*self.scaling
            y=y*self.scaling
            car = pygame.transform.rotate(
                self.carImage, 180.0 * theta / np.pi)
            self.screen.blit(car, (x, y))

    def init_settings(self):
        self.static_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

        for obs in self.obstacles:
            self.static_surface.blit(
                self.obstacleImage, (obs[0]*self.scaling, obs[1]*self.scaling))
        
            # pygame.draw.circle(
            #     self.screen, BLACK, (obs[0]*self.scaling, obs[1]*self.scaling), OBSTACLE_RADIUS*self.scaling)




if __name__ == "__main__":
    xmax,ymax,starts,goals,obstacles=read_maps_from_yaml("../testobs.yaml")
    paths=read_paths_from_yaml("../outputobs.yaml")
    anim=Animator(xmax,ymax,obstacles,paths)
    anim.createAnim()
    # print(paths)``

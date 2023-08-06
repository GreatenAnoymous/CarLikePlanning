import pygame

import numpy as np

import json
import time
import yaml


carPNG = "./car.png"
obsPNG = "./obstacle.png"
HEIGHT = 800
WIDTH = 800

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
OBSTACLE_RADIUS = 0.8


def read_maps_from_yaml(mapYaml):
    pass


def read_maps_from_json(mapJson):
    pass


def read_paths_from_json(pathJson):
    pass


def read_paths_from_yaml(pathYaml):
    pass


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
        self.carImage-pygame.transform.smoothscale(self.carImage, (int(
            self.carLength*self.scaling), int(self.carWidth*self.scaling)))
        self.obstacleImage = pygame.image.load(obsPNG)
        self.obstacleImage = pygame.transform.smoothscale(self.obstacleImage, (int(
            OBSTACLE_RADIUS*self.scaling), int(OBSTACLE_RADIUS*self.scaling)))
        
    def getStateAtTime(self,agentId,t):
        if t>=self.paths[agentId]:
            return self.paths[agentId][-1]
        return self.paths[agentId][t]
    

    def getState(self, lastState, nextState, t):
        pass


    def createAnim(self):
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.screen.fill(WHITE)
        pygame.display.set_caption("Moving Objects")
        self.init_settings()
        clock = pygame.time.Clock()
        running = True
        step=1
        laststep=0
        while running:
            self.screen.fill(WHITE)
            self.screen.blit(self.static_surface, (0, 0))
            pygame.time.get_ticks()
            self.simulate()
            pygame.display.flip()
            clock.tick(30)
            self.screen.blit(self.static_surface, (0, 0))

    def draw_goals(self):
        pass

    def simulate(self):
        pass

    def drawCars(self):
        for loc in self.locations:
            x, y, theta = loc
            car = pygame.transform.rotate(
                self.carImage, - 180.0 * theta / np.pi - 90)
            self.screen.blit(car, (x, y))

    def init_settings(self):
        self.static_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

        for obs in self.obstacles:
            self.static_surface.blit(self.obstacleImage,(obs[0]*self.scaling,obs[1]*self.scaling))
            # pygame.draw.circle(
            #     self.screen, BLACK, (obs[0]*self.scaling, obs[1]*self.scaling), OBSTACLE_RADIUS*self.scaling)

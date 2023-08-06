import pygame


import numpy as np

import json

import yaml


carImage="./car.png"

def read_maps(map_name):
    pass


def read_paths_from_json(inputJson):
    pass

def read_paths_from_yaml(inputYaml):
    pass



class Animator:

    def __init__(self, xmax,ymax, obstacles, paths) -> None:
        self.xmax=xmax
        self.ymax=ymax
        self.obstacles=obstacles
        self.paths=paths



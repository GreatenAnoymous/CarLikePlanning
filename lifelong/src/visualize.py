#!/usr/bin/env python3
from matplotlib import animation
import matplotlib.animation as manimation
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import math
import os
import argparse
import numpy as np
import yaml
import json
import matplotlib
# matplotlib.use("Qt5Agg")

PLOTLINE = False  # True
WAREHOUSE = False
warehouse_rectangles = [(10, 40, 10, 20), (10, 40, 30, 40), (10, 40, 60, 70), (
    10, 40, 80, 90), (60, 90, 10, 20), (60, 90, 30, 40), (60, 90, 60, 70), (60, 90, 80, 90)]

carWidth = 2.0
LF = 2.0
LB = 1.0
obsRadius = 1
framesPerMove = 3


def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)


class Animation:
    def __init__(self, map, schedule):
        self.map = map
        self.schedule = schedule
        self.num_agents = len(map["starts"])
        self.all_tasks = map["goals"]
        aspect = map["xmax"] / map["ymax"]
        self.labels = [0 for i in range(self.num_agents)]

        self.fig = plt.figure(frameon=False, figsize=(8 * aspect, 8))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(
            left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.lines = []
        self.list_xdata = [[] for _ in range(0, self.num_agents)]
        self.list_ydata = [[] for _ in range(0, self.num_agents)]
        self.agents = dict()
        self.agent_names = dict()
        self.goal_artists = []
        # create boundary patch
        xmin = -1
        ymin = -1
        xmax = map["xmax"] + 0.5
        ymax = map["ymax"] + 0.5

        # self.ax.relim()
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        # plt.xlim(10, 20)
        # plt.ylim(10, 20)
        # self.ax.set_xticks([])
        # self.ax.set_yticks([])
        # plt.axis('off')
        # self.ax.axis('tight')
        self.ax.axis('off')

        self.patches.append(Rectangle(
            (xmin, ymin), width=xmax - xmin, height=ymax - ymin, angle=0, facecolor='none', edgecolor='red'))
        if WAREHOUSE == False:
            try:
                for o in map["obstacles"]:
                    x, y = o[0], o[1]
                    self.patches.append(
                        Circle((x, y), obsRadius, facecolor='grey', edgecolor='grey'))
            except:
                pass
        else:
            for o in warehouse_rectangles:
                x1, x2, y1, y2 = o
                self.patches.append(
                    Rectangle((x1, y1), width=x2-x1, height=y2-y1, angle=0, facecolor='grey'))

        # create agents:
        self.T = 0
        cmap = get_cmap(self.num_agents+1)
        # to do : draw the intermeidate waypoints
        # draw goals first
        for i in range(self.num_agents):
            start = self.map["starts"][i]
            final_goal = self.map["goals"][i][0]
            cw = carWidth
            lb = LB
            lf = LF
            goal_art = Rectangle(
                (final_goal[0] - math.sqrt(cw/2*cw/2+lb*lb) * math.sin(math.atan2(lb, cw/2)+final_goal[2]),
                 final_goal[1] - math.sqrt(cw/2*cw/2+lb*lb) * math.cos(math.atan2(lb, cw/2)+final_goal[2])),
                lb+lf, cw, -final_goal[2] / math.pi * 180,
                facecolor='none', edgecolor=cmap(i+1),  alpha=0.7, lw=1, linestyle=":")
            self.patches.append(goal_art)
            self.goal_artists.append(goal_art)
            self.list_xdata[i].append(start[0])
            self.list_ydata[i].append(start[1])
            line, = self.ax.plot(
                self.list_xdata[i], self.list_ydata[i], color=cmap(i+1),  alpha=0.3, lw=2, linestyle="-.")
            self.lines.append(line)

        for i in range(self.num_agents):
            # name = d["name"]
            name = i
            start = self.map["starts"][i]

            self.agents[name] = Rectangle(
                (start[0], start[1]), width=(lb+lf), height=carWidth, angle=0.0, edgecolor='black', alpha=0.7)
            self.agents[name].original_face_color = cmap(i+1)
            self.patches.append(self.agents[name])
            self.T = max(self.T, len(schedule["paths"][i])-1)
            self.agent_names[name] = self.ax.text(
                start[0], start[1], str(name), fontsize=6)
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append(self.agent_names[name])

        # self.ax.set_axis_off()
        # self.fig.axes[0].set_visible(False)
        # self.fig.axes.get_yaxis().set_visible(False)

        # self.fig.tight_layout()
        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=int(self.T+1) *
                                            framesPerMove,
                                            interval=100,
                                            repeat=False,
                                            blit=True)

        self.anim.save('./demo_obs_ecbs.mp4',fps=10)

    def save(self, file_name, speed):
        self.anim.save(
            file_name,
            "ffmpeg",
            fps=10 * speed,
            dpi=300),
        # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

        '''uncomment if want to save as gif'''
        # self.anim.save(file_name.replace('.mp4', '') +
        #                '.gif', writer='imagemagick', fps=15)

    def show(self):
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def stateEqual(self, s1, s2):
        x1, y1, yaw1 = s1
        x2, y2, yaw2 = s2
        dyaw = (np.cos(yaw2)-np.cos(yaw1))**2+(np.sin(yaw2)-np.sin(yaw2))**2
        dist = np.sqrt((x1-x2)**2+(y1-y2)**2+dyaw)
        return (dist < 1e-1)

    def animate_func(self, i):
        for id, path in enumerate(self.schedule["paths"]):
            # agent = schedule["paths"][agent_name]
            pos = self.getState(i / framesPerMove, path)
            goalUpdate = False
            print("agent", id, self.all_tasks[id][self.labels[id]], pos)
            if self.stateEqual(tuple(pos), tuple(self.all_tasks[id][self.labels[id]])):
                goalUpdate = True
                print("updated")
            p = (pos[0], pos[1])

            # print(pos[0], pos[1], -pos[2] / 3.15159 * 180)'
            cw = carWidth
            lb = LB
            lf = LF
            if goalUpdate:
                self.labels[id] += 1
                gx, gy, gyaw = self.all_tasks[id][self.labels[id]]
                self.goal_artists[id]._x0 = gx - \
                    math.sqrt(cw/2*cw/2+lb*lb) * \
                    math.sin(math.atan2(lb, cw/2)+gyaw)
                self.goal_artists[id]._y0 = gy - \
                    math.sqrt(cw/2*cw/2+lb*lb) * \
                    math.cos(math.atan2(lb, cw/2)+gyaw)
                self.goal_artists[id]._x1 = self.goal_artists[id]._x0+lb+lf
                self.goal_artists[id]._y1 = self.goal_artists[id]._y0+cw
                self.goal_artists[id].angle = -gyaw/math.pi*180

            self.agents[id]._x0 = pos[0] - \
                math.sqrt(cw/2*cw/2+lb*lb) * \
                math.sin(math.atan2(lb, cw/2)+pos[2])
            self.agents[id]._y0 = pos[1] - \
                math.sqrt(cw/2*cw/2+lb*lb) * \
                math.cos(math.atan2(lb, cw/2)+pos[2])
            self.agents[id]._x1 = self.agents[id]._x0 + lb+lf
            self.agents[id]._y1 = self.agents[id]._y0 + cw
            self.agents[id].angle = -pos[2] / math.pi * 180
            self.agent_names[id].set_position(p)
            if PLOTLINE:
                self.list_xdata[id].append(pos[0])
                self.list_ydata[id].append(pos[1])
                self.lines[id].set_data(
                    self.list_xdata[id], self.list_ydata[id])

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        return self.patches + self.artists+self.lines

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        return self.patches + self.artists+self.lines

    def getState(self, t, d):
        idx = 0
        while idx < len(d) and idx < t:
            idx += 1
        if idx == 0:
            return np.array([float(d[0][0]), float(d[0][1]), float(d[0][2])])
        elif idx < len(d):
            yawLast = float(d[idx-1][2])
            yawNext = float(d[idx][2])
            if (yawLast - yawNext) > math.pi:
                yawLast = yawLast - 2 * math.pi
            elif (yawNext - yawLast) > math.pi:
                yawLast = yawLast + 2 * math.pi
            posLast = np.array(
                [float(d[idx-1][0]), float(d[idx-1][1]), yawLast])
            posNext = np.array(
                [float(d[idx][0]), float(d[idx][1]), yawNext])
            # dt = d[idx]["t"] - d[idx-1]["t"]
            dt = 1
            t = (t - (idx-1)) / dt
            pos = (posNext - posLast) * t + posLast
            return pos
        else:
            return np.array([float(d[-1][0]), float(d[-1][1]), float(d[-1][2])])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", default=None,
                        help="input file containing map")
    parser.add_argument("-s", "--schedule", default=None,
                        help="schedule for agents")
    parser.add_argument("-v", "--video", dest='video', default=None,
                        help="output video file (or leave empty to show on screen)")
    parser.add_argument("--speed", type=int,
                        default=1, help="speedup-factor")
    args = parser.parse_args()

    with open(args.map) as map_file:
        # map = yaml.load(map_file, Loader=yaml.FullLoader)
        map = json.load(map_file)

    with open(args.schedule) as states_file:
        # schedule = yaml.load(states_file, Loader=yaml.FullLoader)
        schedule = json.load(states_file)

    # try:
    #     with open(os.path.abspath(os.path.join(
    #             os.getcwd(), ".."))+"/src/config.yaml") as config_file:
    #         carConfig = yaml.load(config_file, Loader=yaml.FullLoader)
    #         # global carWidth, LF, LB, obsRadius, framesPerMove
    #         carWidth = carConfig["carWidth"]
    #         LF = carConfig["LF"]
    #         LB = carConfig["LB"]
    #         obsRadius = carConfig["obsRadius"]-0.1
    # except IOError:
    #     # do things with your exception
    #     print("ERROR loading config file", os.path.abspath(os.path.join(
    #         os.getcwd(), ".."))+"/src/config1.yaml", " using default param to plot")

    # print(schedule)
    animation = Animation(map, schedule)

    if args.video:
        matplotlib.use("Agg")
        animation.save(args.video, args.speed)
    else:
        # matplotlib.use("Qt5Agg")
        animation.show()

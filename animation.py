import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.style.use('ggplot')


class AnimationMaker():
    def __init__(self, theta1file, theta2file, l1, l2):
        self.loadData(theta1file, theta2file)

        self.l1 = l1
        self.l2 = l2

        self.trajectory_x = []
        self.trajectory_y = []

        self.figure()

    def loadData(self, theta1file, theta2file):
        theta1 = np.loadtxt(theta1file, delimiter=',', unpack=True)
        theta2 = np.loadtxt(theta2file, delimiter=',', unpack=True)
        self.time = theta1[0]
        self.theta1 = theta1[1]
        self.theta2 = theta2[1]

    def figure(self):
        self.fig, ax = plt.subplots(figsize=(8, 8))

        self.line_trajectory, = ax.plot(
            [], [], color='tab:blue', label='trajectory')
        self.line_l1, = ax.plot(
            [], [], color='tab:gray', label='l1', linewidth=3)
        self.line_l2, = ax.plot(
            [], [], color='tab:gray', label='l2', linewidth=3)

        ax.set_xlabel('x [mm]')
        ax.set_ylabel('y [mm]')
        maxlen = self.l1 + self.l2 + 10
        ax.set_xlim(-maxlen, maxlen)
        ax.set_ylim(-maxlen, maxlen)
        ax.set_aspect('equal')
        # ax.legend(loc='lower left')
        plt.subplots_adjust(left=0.1, right=0.97, bottom=0.04, top=0.97)

    def _update(self, i):
        theta1 = math.radians(self.theta1[i])
        theta2 = math.radians(self.theta2[i])

        x1 = self.l1 * math.cos(theta1)
        y1 = self.l1 * math.sin(theta1)
        x2 = x1 + self.l2 * math.cos(theta1 + theta2)
        y2 = y1 + self.l2 * math.sin(theta1 + theta2)

        self.trajectory_x.append(x2)
        self.trajectory_y.append(y2)

        self.line_l1.set_data([0, x1], [0, y1])
        self.line_l2.set_data([x1, x2], [y1, y2])
        self.line_trajectory.set_data(self.trajectory_x, self.trajectory_y)

        plt.title('t = {:3.2f} [s], (x, y) = ({:3.0f}, {:3.0f})'.format(
            i / len(self.time), x2, y2))

    def makeAnimation(self):
        return animation.FuncAnimation(self.fig, self._update,
                                       interval=100, frames=len(self.time))


if __name__ == '__main__':
    # animationMaker = AnimationMaker('theta1example.csv', 'theta2example.csv', 150, 150)
    animationMaker = AnimationMaker('theta1.csv', 'theta2.csv', 150, 150)
    ani = animationMaker.makeAnimation()
    ani.save('animation.gif', writer='pillow')

    # plt.plot(animationMaker.trajectory_x, animationMaker.trajectory_y)
    plt.savefig('trajectory.png')

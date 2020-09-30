import math
import bisect
import pprint
import numpy as np
import matplotlib.pyplot as plt
import manipurator
import seaborn as sns

sns.set()


class Curve():
    def __init__(self, x0, xf, v0, vf, t0, tf, a0=0, af=0):
        dt = tf - t0
        self.t0 = t0
        self.a = np.zeros(6)
        self.a[0] = x0
        self.a[1] = v0
        self.a[2] = a0 / 2
        self.a[3] = (20 * (xf - x0) - (8 * vf + 12 * v0) *
                     dt - (3 * a0 - af) * dt**2) / (2 * dt**3)
        self.a[4] = (30 * (x0 - xf) + (14 * vf + 16 * v0) *
                     dt + (3 * a0 - 2 * af) * dt**2) / (2 * dt**4)
        self.a[5] = (12 * (xf - x0) - 6 * (vf + v0) *
                     dt - (a0 - af) * dt**2) / (2 * dt**5)

    def calc(self, t):
        t -= self.t0
        return self.a[0] + self.a[1] * t + self.a[2] * t**2 + \
            self.a[3] * t**3 + self.a[4] * t**4 + self.a[5] * t**5


class Planning():
    def __init__(self, t, x, delta=0.05, dt=0.01):
        self.tList = t
        self.xList = x
        self.delta = delta
        self.dt = dt
        self.params = []

        self._getAuxiliaryPoints()
        self.getCource()

        self.t0 = self.pt.reshape(-1, 1).copy()
        self.t0[0] = 0
        self.t0[-1] = t[-1]
        # print(self._getIndex(0.5))
        # print(np.array(self.params))
        # print(self.t0)

    def _getIndex(self, t):
        return bisect.bisect(self.t0, t) - 1

    #  補助点を求める
    def _getAuxiliaryPoints(self):
        self.p = np.zeros((len(self.xList), 2))
        self.pt = np.zeros((len(self.xList), 2))
        for i in range(len(self.xList)):
            if i == 0:
                v = (self.xList[i + 1] - self.xList[i]) / \
                    (self.tList[i + 1] - self.tList[i])

                self.p[i][0] = self.xList[i]
                self.p[i][1] = self.xList[i] + v * self.delta
                self.pt[i][0] = self.tList[i] + self.delta
                self.pt[i][1] = self.tList[i] + 2 * self.delta

            elif i == len(self.xList) - 1:
                v = (self.xList[i] - self.xList[i - 1]) / \
                    (self.tList[i] - self.tList[i - 1])

                self.p[i][0] = self.xList[i] - v * self.delta
                self.p[i][1] = self.xList[i]
                self.pt[i][0] = self.tList[i] - 2 * self.delta
                self.pt[i][1] = self.tList[i] - self.delta

            else:
                v1 = (self.xList[i] - self.xList[i - 1]) / \
                    (self.tList[i] - self.tList[i - 1])
                v2 = (self.xList[i + 1] - self.xList[i]) / \
                    (self.tList[i + 1] - self.tList[i])

                self.p[i][0] = self.xList[i] - v1 * self.delta
                self.p[i][1] = self.xList[i] + v2 * self.delta
                self.pt[i][0] = self.tList[i] - self.delta
                self.pt[i][1] = self.tList[i] + self.delta

    # 軌跡を求める
    def getCource(self):
        for i in range(len(self.xList)):
            # 曲線の最初と最後の速度を設定
            if i == 0:
                v0 = 0
                vf = (self.p[i + 1][0] - self.p[i][1]) / \
                    (self.pt[i + 1][0] - self.pt[i][1])
                t0 = 0
                tf = self.pt[i][1]
            elif i == len(self.xList) - 1:
                v0 = (self.p[i][0] - self.p[i - 1][1]) / \
                    (self.pt[i][0] - self.pt[i - 1][1])
                vf = 0
                t0 = self.pt[i][0]
                tf = self.tList[-1]
            else:
                v0 = (self.xList[i] - self.xList[i - 1]) / \
                    (self.tList[i] - self.tList[i - 1])
                vf = (self.xList[i + 1] - self.xList[i]) / \
                    (self.tList[i + 1] - self.tList[i])
                t0 = self.pt[i][0]
                tf = self.pt[i][1]

            # 曲線を計算
            curve = Curve(self.p[i][0], self.p[i][1], v0, vf, t0, tf)
            self.params.append(curve.a)

            # 終了処理
            if i == len(self.xList) - 1:
                break

            # 速さを設定
            if i == 0 or i == len(self.xList) - 2:
                v = (self.p[i + 1][0] - self.p[i][1]) / \
                    (self.pt[i + 1][0] - self.pt[i][1])
            else:
                v = (self.xList[i + 1] - self.xList[i]) / \
                    (self.tList[i + 1] - self.tList[i])

            # 直線を計算
            self.params.append(np.array([self.p[i][1], v, 0, 0, 0, 0]))

    def calc(self, t):
        i = self._getIndex(t)
        i = np.clip(i, 0, len(self.params) - 1)
        t -= self.t0[i]
        return self.params[i][0] + self.params[i][1] * t + self.params[i][2] * t**2 + \
            self.params[i][3] * t**3 + self.params[i][4] * \
            t**4 + self.params[i][5] * t**5

    def plot(self, ax, marker=''):
        t = np.arange(0, self.tList[-1] + self.dt, self.dt)
        x = np.array([self.calc(t1) for t1 in t]).reshape(-1, 1)
        ax.scatter(self.tList, self.xList, marker='x', color='tab:gray')
        ax.plot(t, x, marker=marker)
        ax.scatter(self.pt.reshape(-1, 1), self.p.reshape(-1, 1), marker='+', color='tab:red')


class Plot():
    def __init__(self, t, x, y, theta1, theta2,
                 planning_x, planning_y, marker=''):
        self.fig, self.ax = plt.subplots(5, 1, sharex=True, figsize=(10, 16))

        planning_x.plot(self.ax[0], marker)
        self.ax[0].set_ylabel('x [mm]')
        planning_y.plot(self.ax[1], marker)
        self.ax[1].set_ylabel('y [mm]')
        self.plot_velocity(self.ax[2], t, x, y, marker)
        self.plot_theta(self.ax[3], t, theta1, theta2, marker)
        self.plot_angularVelocity(self.ax[4], t, theta1, theta2, marker)
        plt.xlabel('time [s]')
        plt.subplots_adjust(bottom=0.05, top=0.95)
        self.fig.align_labels()

    def plot_velocity(self, ax, t, x, y, marker, dt=0.01):
        vxList = np.diff(x) / dt
        vyList = np.diff(y) / dt
        v = [math.sqrt(vx**2 + vy**2) for vx, vy in zip(vxList, vyList)]
        v = np.insert(v, 0, 0)
        ax.plot(t, v, marker=marker)
        ax.set_ylabel('velocity [mm/s]')

    def plot_theta(self, ax, t, theta1, theta2, marker, dt=0.01):
        ax.plot(t, theta1, label='theta1', marker=marker)
        ax.plot(t, theta2, label='theta2', marker=marker)
        ax.set_ylabel('theta [degree]')
        ax.set_yticks(np.arange(-180, 180 + 1, 90))
        ax.legend()

    def plot_angularVelocity(self, ax, t, theta1, theta2, marker, dt=0.01):
        omega1 = np.diff(theta1) / dt / 6
        omega1 = np.insert(omega1, 0, 0)
        omega2 = np.diff(theta2) / dt / 6
        omega2 = np.insert(omega2, 0, 0)
        ax.plot(t, omega1, label='theta1', marker=marker)
        ax.plot(t, omega2, label='theta2', marker=marker)
        ax.set_ylabel('angular velocity [rpm]')
        ax.set_ylim(-100, 100)
        ax.legend()

    def show(self):
        self.fig.show()

    def savefig(self, filename):
        self.fig.savefig(filename)


class PlanningXY():
    def __init__(self, waypoints_x, waypoints_y, vmax=0.3e3, delta=0.05, dt=0.01):
        self.vmax = vmax
        self.delta = delta
        self.dt = dt
        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y
        self.waypoints_t = self.setWaypointTiming()

    def setWaypointTiming(self):
        waypoints_t = [0]
        for i in range(len(waypoints) - 1):
            dx = self.waypoints_x[i + 1] - self.waypoints_x[i]
            dy = self.waypoints_y[i + 1] - self.waypoints_y[i]
            d = math.sqrt(dx**2 + dy**2)
            dt = d / self.vmax
            if i == 0 or i == len(waypoints) - 2:
                dt += self.delta
            waypoints_t.append(waypoints_t[-1] + dt)
        return waypoints_t

    def getCource(self):
        self.planning_x = Planning(self.waypoints_t, self.waypoints_x)
        self.planning_y = Planning(self.waypoints_t, self.waypoints_y)
        self.t = np.arange(0, self.waypoints_t[-1] + self.dt, self.dt)
        self.x = np.array([self.planning_x.calc(t1) for t1 in self.t]).ravel()
        self.y = np.array([self.planning_y.calc(t1) for t1 in self.t]).ravel()
        return self.t, self.x, self.y

    def savePathGraph(self, filename):
        plt.figure(figsize=(20, 10))
        plt.plot(self.waypoints_x, self.waypoints_y, marker='x', color='tab:gray')
        plt.scatter(self.planning_x.p.reshape(-1, 1), self.planning_y.p.reshape(-1, 1), marker='+', color='tab:red')
        plt.plot(self.x, self.y, marker='.')
        plt.gca().set_aspect('equal')
        plt.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
        plt.savefig(filename)


if __name__ == '__main__':
    offset = 130
    waypoints = np.array([
        [-160, 10 + offset],
        [   0, 10 + offset],
        # [-50, 40 + offset],
        [   0, 80 + offset],
        [ 160, 80 + offset],
    ])

    planningXY = PlanningXY(waypoints[:, 0], waypoints[:, 1])
    t, x, y = planningXY.getCource()
    planningXY.savePathGraph('graph/path.png')

    l1 = 140
    l2 = 160
    manipurator = manipurator.Manipurator(l1, l2)

    # Calculate the joint angle from the hand position
    theta1List, theta2List = [], []
    for i in range(len(x)):
        theta1, theta2 = manipurator.inverseKinematics(x[i], y[i])
        theta1List.append(math.degrees(theta1))
        theta2List.append(math.degrees(theta2))

    # Save to file
    theta1 = np.stack([t, theta1List], axis=1)
    theta2 = np.stack([t, theta2List], axis=1)
    trajectory = np.stack([t, -np.array(theta1List), theta2List], axis=1)
    np.savetxt('data/theta1.csv', theta1, delimiter=',', fmt='%04f')
    np.savetxt('data/theta2.csv', theta2, delimiter=',', fmt='%04f')
    np.savetxt('data/trajectory.csv', trajectory, delimiter=',', fmt='%04f')

    plot = Plot(
        t,
        x,
        y,
        theta1List,
        theta2List,
        planningXY.planning_x,
        planningXY.planning_y,
        marker='')
    plot.savefig('graph/graphs.png')

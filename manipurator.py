import math
import numpy as np
import matplotlib.pyplot as plt


class Manipurator():
    def __init__(self, l1, l2, d0=0):
        self.l1 = l1
        self.l2 = l2
        self.d0 = d0

    def forwardKinematics(self, theta1, theta2):
        x1 = self.l1 * math.cos(theta1)
        y1 = self.l1 * math.sin(theta1)
        x2 = x1 + self.l2 * math.cos(theta2)
        y2 = y1 + self.l2 * math.sin(theta2)
        return x2, y2

    def inverseKinematics(self, x, y, eps=1e-10):
        tmp = (self.l1**2 + x**2 + (y - self.d0)**2 - self.l2**2) / \
            (2 * self.l1 * math.sqrt(x**2 + (y - self.d0)**2) + eps)
        tmp = np.clip(tmp, -1, 1)
        theta1 = math.atan2(y - self.d0, x + eps) - math.acos(tmp)

        tmp2 = (self.l1**2 + self.l2**2 - x**2 - (y - self.d0)**2) / \
            (2 * self.l1 * self.l2)
        tmp2 = np.clip(tmp2, -1, 1)
        theta2 = math.pi - math.acos(tmp2)

        return theta1, theta2


class PathPlanner():
    def __init__(self, acc, vmax, dt=0.01):
        self.acc = acc
        self.vmax = vmax
        self.dt = dt

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def angle(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def planning(self, x1, y1, x2, y2):
        angle = self.angle(x1, y1, x2, y2)
        print(math.degrees(angle))
        print(self.vmax ** 2 / self.acc / 2)

        t, v = 0, 0
        x, y = x1, y1
        diff = self.distance(x1, y1, x2, y2)
        diff0 = diff
        distance = 0

        tList, xList, yList, vList = [0], [x1], [y1], [0]
        while diff > 0.1:
            t += self.dt
            if t > 2:
                break
            print(diff0 - distance)

            if diff0 - distance < self.vmax ** 2 / self.acc / 2:
                v -= self.acc * self.dt
            elif v < self.vmax:
                v += self.acc * self.dt
            if v < 0:
                break
            v = np.clip(v, 0, self.vmax)

            x += v * self.dt * math.cos(angle)
            y += v * self.dt * math.sin(angle)
            diff = self.distance(x, y, x2, y2)
            distance += v * self.dt

            tList.append(t)
            xList.append(x)
            yList.append(y)
            vList.append(v)

            print(t, x, y, diff, v)

        return {'t': tList, 'x': xList, 'y': yList, 'v': vList}


if __name__ == '__main__':
    manipurator = Manipurator(150, 150)
    vmax = 308.7904831758910
    acc = vmax / 0.2
    path = PathPlanner(acc, vmax).planning(300, 0, 155, 200)
    # path = PathPlanner(acc, vmax).planning(100, 50, 300, 150)

    theta1List, theta2List = [], []
    for i in range(len(path['t'])):
        theta1, theta2 = manipurator.inverseKinematics(
            path['x'][i], path['y'][i])
        x, y = manipurator.forwardKinematics(theta1, theta2)
        print(x, y)
        theta1List.append(math.degrees(theta1))
        theta2List.append(math.degrees(theta2))

    theta1 = np.stack([path['t'], theta1List], axis=1)
    theta2 = np.stack([path['t'], theta2List], axis=1)
    np.savetxt('theta1.csv', theta1, delimiter=',', fmt='%04f')
    np.savetxt('theta2.csv', theta2, delimiter=',', fmt='%04f')

    plt.figure()
    # plt.plot([300, 155], [0, 200])
    plt.plot(path['x'], path['y'])
    plt.savefig('path.png')

    plt.figure()
    plt.plot(path['t'], path['v'])
    plt.savefig('vel.png')

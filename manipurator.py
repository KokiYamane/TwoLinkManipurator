import math
import numpy as np
import matplotlib.pyplot as plt
# import seaborn as sns

# sns.set()


class Manipurator():
    def __init__(self, l1, l2, d0=0):
        self.l1 = l1
        self.l2 = l2
        self.d0 = d0

    def forwardKinematics(self, theta1, theta2):
        x1 = self.l1 * math.cos(theta1)
        y1 = self.l1 * math.sin(theta1)
        x2 = x1 + self.l2 * math.cos(theta1 + theta2)
        y2 = y1 + self.l2 * math.sin(theta1 + theta2)
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

    def straight(self, x1, y1, x2, y2, t0=0):
        theta = self.angle(x1, y1, x2, y2)

        t, v = t0, 0
        x, y = x1, y1
        diff = self.distance(x1, y1, x2, y2)
        diff0 = diff
        distance = 0

        tList, xList, yList, vList = [t0], [x1], [y1], [0]
        while t < 5:
            t += self.dt

            if diff0 - distance < self.vmax ** 2 / self.acc / 2:
                v = self.acc * self.dt * (diff0 - distance)
            elif v < self.vmax:
                v += self.acc * self.dt
            v = np.clip(v, 0, self.vmax)

            x += v * self.dt * math.cos(theta)
            y += v * self.dt * math.sin(theta)
            diff = self.distance(x, y, x2, y2)
            if diff < 0.2:
                break
            distance += v * self.dt

            print(t, x, y, v, diff)

            tList.append(t)
            xList.append(x)
            yList.append(y)
            vList.append(v)

        tList.append(tList[-1] + self.dt)
        xList.append(xList[-1])
        yList.append(yList[-1])
        vList.append(0)

        return {'t': tList, 'x': xList, 'y': yList, 'v': vList}

    def planning_straight(self, waypoints):
        path = {'t': [], 'x': [], 'y': [], 'v': []}
        t0 = 0
        for i in range(len(waypoints) - 1):
            pathPart = self.straight(
                waypoints[i][0], waypoints[i][1], waypoints[i + 1][0], waypoints[i + 1][1], t0=t0)
            path['t'].extend(pathPart['t'])
            path['x'].extend(pathPart['x'])
            path['y'].extend(pathPart['y'])
            path['v'].extend(pathPart['v'])
            t0 = path['t'][-1]
        return path

    def planning(self, waypoints):
        path = {'t': [], 'x': [], 'y': [], 'v': []}
        t = 0
        x = waypoints[0][0]
        y = waypoints[0][1]
        v = 0
        i = 1
        theta = self.angle(x, y, waypoints[i][0], waypoints[i][1])
        theta2 = self.angle(waypoints[i][0], waypoints[i][1],
                            waypoints[i + 1][0], waypoints[i + 1][1])
        tList, xList, yList, vList = [t], [x], [y], [v]
        distance = 0
        diff = self.distance(waypoints[i - 1][0], waypoints[i - 1][1],
                             waypoints[i][0], waypoints[i][1])
        while t < 5:
            t += self.dt

            delta = 6
            if diff - distance < 1e-10:
                print('a')
                i += 1
                if i >= len(waypoints):
                    break
                distance = 0
                diff = self.distance(x, y, waypoints[i][0], waypoints[i][1])
                theta = self.angle(waypoints[i - 1][0], waypoints[i - 1][1],
                                   waypoints[i][0], waypoints[i][1])
                if i < len(waypoints) - 1:
                    theta2 = self.angle(waypoints[i][0], waypoints[i][1],
                                        waypoints[i + 1][0], waypoints[i + 1][1])
            elif diff - distance <= delta:
                print('b')
                a = (diff - distance) / delta
                v1 = self.vmax * a
                v2 = self.vmax * (1 - a)
                vx = v1 * math.cos(theta) + v2 * math.cos(theta2)
                vy = v1 * math.sin(theta) + v2 * math.sin(theta2)
                v = math.sqrt(vx**2 + vy ** 2)
                distance += v1 * self.dt
            elif v < self.vmax:
                print('c')
                v += self.acc * self.dt
                v = np.clip(v, 0, self.vmax)
                vx = v * math.cos(theta)
                vy = v * math.sin(theta)
                distance += v * self.dt
            else:
                distance += v * self.dt

            x += vx * self.dt
            y += vy * self.dt

            path['t'].append(t)
            path['x'].append(x)
            path['y'].append(y)
            path['v'].append(v)
            print(t, x, y, v, vx, vy, diff, distance)

        return path


if __name__ == '__main__':
    l1 = 140
    l2 = 160
    vmax = 0.3e3
    acc = vmax / 0.05
    offset = 130
    waypoints = np.array([
        [-160, 10 + offset],
        [8, 10 + offset],
        [8, 80 + offset],
        [160, 80 + offset],
    ])
    waypoints = np.array([
        [-160, 20 + offset],
        [0, 20 + offset],
        [0, 80 + offset],
        [160, 80 + offset],
    ])
    # waypoints = np.array([
    #     [-160, 140],
    #     # [-150, 140],
    #     [150, 140],
    #     [-170, 140],
    # ])
    manipurator = Manipurator(l1, l2)
    path = PathPlanner(acc, vmax).planning_straight(waypoints)
    # path = PathPlanner(acc, vmax).planning(waypoints)

    theta1List, theta2List = [], []
    for i in range(len(path['t'])):
        theta1, theta2 = manipurator.inverseKinematics(
            path['x'][i], path['y'][i])
        x, y = manipurator.forwardKinematics(theta1, theta2)
        theta1List.append(math.degrees(theta1))
        theta2List.append(math.degrees(theta2))

    # save to file
    theta1 = np.stack([path['t'], theta1List], axis=1)
    theta2 = np.stack([path['t'], theta2List], axis=1)
    trajectory = np.stack(
        [path['t'], -np.array(theta1List), theta2List], axis=1)
    np.savetxt('data/theta1.csv', theta1, delimiter=',', fmt='%04f')
    np.savetxt('data/theta2.csv', theta2, delimiter=',', fmt='%04f')
    np.savetxt('data/trajectory.csv', trajectory, delimiter=',', fmt='%04f')

    # path graph
    plt.figure()
    plt.plot(path['x'], path['y'], marker='.')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.gca().set_aspect('equal')
    plt.savefig('graph/path.png')
    plt.show()

    # velocity graph
    plt.figure()
    plt.plot(path['t'], path['v'])
    plt.xlabel('time [s]')
    plt.ylabel('velocity [mm/s]')
    plt.savefig('graph/vel.png')

    # anguler velocity graph
    plt.figure()
    omega1 = np.diff(theta1List) * 60**2 / 360
    omega2 = np.diff(theta2List) * 60**2 / 360
    omega1 = np.insert(omega1, 0, 0)
    omega2 = np.insert(omega2, 0, 0)
    plt.plot(path['t'], omega1, label='theta1')
    plt.plot(path['t'], omega2, label='theta2')
    plt.xlabel('time [s]')
    plt.ylabel('angular velocity [rpm]')
    plt.ylim(-100, 100)
    plt.legend()
    plt.savefig('graph/vel_theta.png')

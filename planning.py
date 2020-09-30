import math
import numpy as np
import matplotlib.pyplot as plt
import manipurator

class Curve():
  def __init__(self, x0, xf, v0, vf, tf, a0=0, af=0):
    self.a = np.zeros(6)
    self.a[0] = x0
    self.a[1] = v0
    self.a[2] = a0 / 2
    self.a[3] = (20 * (xf - x0) - (8 * vf + 12 * v0) * tf - (3 * a0 - af) * tf**2) / (2 * tf**3)
    self.a[4] = (30 * (x0 - xf) + (14 * vf + 16 * v0) * tf + (3 * a0 - 2 * af) * tf**2) / (2 * tf**4)
    self.a[5] = (12 * (xf - x0) - 6 * (vf + v0) * tf - (a0 - af) * tf**2) / (2 * tf**5)
    print(self.a)

  def calc(self, x):
    return self.a[0] + self.a[1] * x + self.a[2] * x**2 + self.a[3] * x**3 + self.a[4] * x**4 + self.a[5] * x**5

class Planning():
  def __init__(self, t, x, delta=0.05, dt=0.01):
    self.tList = t
    self.xList = x
    self.delta = delta
    self.dt = dt
    self.cource = []

    self._getAuxiliaryPoints()
    self.getCource()

  #  補助点を求める
  def _getAuxiliaryPoints(self):
    self.p = np.zeros((len(self.xList), 2))
    self.pt = np.zeros((len(self.xList), 2 ))
    for i in range(len(self.xList)):
      if i == 0:
        v = (self.xList[i + 1] - self.xList[i]) / (self.tList[i + 1] - self.tList[i])

        self.p[i][0] = self.xList[i]
        self.p[i][1] = self.xList[i] + v * self.delta
        self.pt[i][0] = self.tList[i] + self.delta
        self.pt[i][1] = self.tList[i] + 2 * self.delta

      elif i == len(self.xList) - 1:
        v = (self.xList[i] - self.xList[i - 1]) / (self.tList[i] - self.tList[i - 1])

        self.p[i][0] = self.xList[i] - v * self.delta
        self.p[i][1] = self.xList[i]
        self.pt[i][0] = self.tList[i] - 2 * self.delta
        self.pt[i][1] = self.tList[i] - self.delta

      else:
        v1 = (self.xList[i] - self.xList[i - 1]) / (self.tList[i] - self.tList[i - 1])
        v2 = (self.xList[i + 1] - self.xList[i]) / (self.tList[i + 1] - self.tList[i])

        self.p[i][0] = self.xList[i] - v1 * self.delta
        self.p[i][1] = self.xList[i] + v2 * self.delta
        self.pt[i][0] = self.tList[i] - self.delta
        self.pt[i][1] = self.tList[i] + self.delta

  # 軌跡を求める
  def getCource(self):
    for i in range(len(self.xList)):
      if i == 0:
        v0 = 0
        vf = (self.p[i + 1][0] - self.p[i][1]) / (self.pt[i + 1][0] - self.pt[i][1]) 
      else: v0 = (self.xList[i] - self.xList[i - 1]) / (self.tList[i] - self.tList[i - 1]) 

      if i == len(self.xList) - 1:
        vf = 0
        v0 = (self.p[i][0] - self.p[i - 1][1]) / (self.pt[i][0] - self.pt[i - 1][1]) 
      else: vf = (self.xList[i + 1] - self.xList[i]) / (self.tList[i + 1] - self.tList[i])

      curve = Curve(self.p[i][0], self.p[i][1], v0, vf, 2 * self.delta)
      t = np.arange(0, 2 * self.delta, self.dt)
      self.cource.extend(curve.calc(t))
      if i == len(self.xList) - 1: break

      if i == 0 or i == len(self.xList) - 2:
        v = (self.p[i + 1][0] - self.p[i][1]) / (self.pt[i + 1][0] - self.pt[i][1]) 
      else:
        v = (self.xList[i + 1] - self.xList[i]) / (self.tList[i + 1] - self.tList[i])
      tf = (self.p[i+1][0] - self.p[i][1]) / v
      t = np.arange(0, tf, self.dt)
      self.cource.extend(v * t + self.p[i][1])

  def plot(self):
    plt.plot(np.arange(len(self.cource)) * self.dt, self.cource, marker='.')
    plt.scatter(self.tList, self.xList)
    plt.scatter(self.pt.reshape(-1, 1), self.p.reshape(-1, 1))
    plt.show()

if __name__ == '__main__':
  offset = 130
  waypoints = np.array([
      [  0, -160, 10 + offset],
      [0.4,    8, 11 + offset],
      [0.8,    9, 79 + offset],
      [1.6,  160, 80 + offset],
  ])

  planning_x = Planning(waypoints[:, 0], waypoints[:, 1])
  planning_y = Planning(waypoints[:, 0], waypoints[:, 2])
  x = planning_x.cource
  y = planning_y.cource
  t = np.arange(len(x)) * planning_x.dt
  planning_x.plot()
  planning_y.plot()

  plt.plot(x, y, marker='.')
  plt.show()

  l1 = 140
  l2 = 160
  manipurator = manipurator.Manipurator(l1, l2)

  theta1List, theta2List = [], []
  for i in range(len(x)):
      theta1, theta2 = manipurator.inverseKinematics(
          x[i], y[i])
      # x, y = manipurator.forwardKinematics(theta1, theta2)
      theta1List.append(math.degrees(theta1))
      theta2List.append(math.degrees(theta2))

  # save to file
  theta1 = np.stack([t, theta1List], axis=1)
  theta2 = np.stack([t, theta2List], axis=1)
  trajectory = np.stack(
      [t, -np.array(theta1List), theta2List], axis=1)
  np.savetxt('data/theta1.csv', theta1, delimiter=',', fmt='%04f')
  np.savetxt('data/theta2.csv', theta2, delimiter=',', fmt='%04f')
  np.savetxt('data/trajectory.csv', trajectory, delimiter=',', fmt='%04f')

  print(trajectory)

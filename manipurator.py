import math

class Manipurator():
    def __init__(self, l1, l2, d0=0):
        self.l1 = l1
        self.l2 = l2
        self.d0 = d0

    def inverseKinematics(self, x, y):
        theta1 = math.atan2(y - self.d0, x + 1e-10) + \
            math.acos((self.l1**2 + x**2 + (y - self.d0)**2-self.l2**2) / \
            (2 * self.l1 * math.sqrt(x**2+(y-self.d0)**2) + 1e-10))
        theta2 = math.pi - math.acos(
            (self.l1**2 + self.l2**2 - x**2 - (y - self.d0)**2) /
            (2 * self.l1 * self.l2))
        return theta1, theta2

if __name__ == '__main__':
    manipurator = Manipurator(150, 150)
    xList = range(100)
    yList = range(100)

    for x, y in zip(xList, yList):
        theta1, theta2 = manipurator.inverseKinematics(x, y)
        print(x, y, theta1, theta2)

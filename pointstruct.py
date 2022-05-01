import math

class Point(object):
    x = 0
    y = 0

    distanceFromStart = 0
    curvature = 0
    globalHeading = 0
    maximumVelocity = 0
    targetVelocity = 0

    def setCoordinates(self, points):
        self.x = points[0]
        self.y = points[1]

    def setDistance(self, distance):
        self.distanceFromStart = distance

    def setCurvature(self, curv):
        self.curvature = curv

    def setMaxVelocity(self, maxVel):
        self.maximumVelocity = maxVel

    def setTargetVelocity(self, targVel):
        self.targetVelocity = targVel

    def __init__(self,  point):
        self.x = point[0]
        self.y = point[1]
    
    

def getMagnitude(point):
    return math.sqrt((point.x*point.x) + (point.y*point.y))

def getDistance(p0, p1):
    return getMagnitude(Point([p1.x - p0.x, p1.y - p0.y]))



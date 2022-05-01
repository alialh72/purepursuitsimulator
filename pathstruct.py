from pointstruct import Point, getMagnitude, getDistance
import math

class Vector :
  magnitude = 0
  thetaHeading = 0
  refAngle = 0
  quadrant = 0
  startPoint = Point([0,0])
  endPoint = Point([0,0])

  def Dot(self, b):
    return self.magnitude * b.magnitude * math.cos(abs(b.thetaHeading - self.thetaHeading))
  

  def __init__(self, p1, p2):
    #__Start & End__
    self.startPoint = p1
    self.endPoint = p2

    #__Vector magnitude
    self.magnitude = getDistance(p1, p2)

    #__Determine quadrant__

    #quadrant 1:  x2 > 1 & y2 > y1    
    if (p2.x > p1.x and p2.y > p1.y):
      insideAngle = math.asin(abs(p2.x - p1.x) / self.magnitude)
      self.quadrant = 1
      self.refAngle = math.pi/2 - insideAngle
      self.thetaHeading = insideAngle
    

    #quadrant 2:  x2 > x1 & y1 > y2    
    elif (p2.x > p1.x and p1.y > p2.y):
      insideAngle = math.asin(abs(p1.y - p2.y) / self.magnitude)
      self.quadrant = 2
      self.refAngle = insideAngle
      self.thetaHeading = insideAngle + (math.pi / 2)
    

    #quadrant 3:  x1 > x2 & y1 > y2    
    elif (p1.x > p2.x and p1.y > p2.y):
      insideAngle = math.asin(abs(p2.y - p1.y) / self.magnitude)
      self.quadrant = 3
      self.refAngle = insideAngle
      self.thetaHeading = (3 * math.pi / 2) - insideAngle
    

    #quadrant 4:  x1 > x2 & y2 > y1    
    elif (p1.x > p2.x and p2.y > p1.y):
      insideAngle = math.asin(abs(p2.x - p1.x) / self.magnitude)
      self.quadrant = 4
      self.refAngle = math.pi/2 - insideAngle
      self.thetaHeading = (2 * math.pi) - insideAngle
    

    #quadrant 5: straight up y2 > y1 and x2 == x1
    elif (p1.x == p2.x and p2.y > p1.y): 
      insideAngle = 0
      self.quadrant = 5
      self.refAngle = math.pi / 2
      self.thetaHeading = insideAngle
    
    
    #quadrant 6: straight right y2 == y1 && x2 > x1
    elif (p2.x > p1.x and p2.y == p1.y) :
      insideAngle = math.pi / 2
      self.quadrant = 6
      self.refAngle = 0
      self.thetaHeading = insideAngle
    

    #quadrant 7: straight down y1 > y2 && x2 = x1
    elif (p1.x == p2.x and p1.y > p2.y) :
      insideAngle = math.pi
      self.quadrant = 7
      self.refAngle =math.pi/2
      self.thetaHeading = insideAngle
    

    #quadrant 8: straight left x1 > x2 && y2 = y1
    elif (p1.x > p2.x and p2.y == p1.y) :
      insideAngle = 3 * math.pi / 2
      self.quadrant = 8
      self.refAngle = 0
      self.thetaHeading = insideAngle
    

    else:
      self.thetaHeading = 0
    


class Path :
  #deque is used here instead of vector as it makes it easy to push points to the front
  #point --> a vector of [x,y]
  points = []

  #returns a point
  def getPoint(self, t) :
    return self.points[t]
  
  
  #adds a point
  def addPoint(self, point):
    self.points.append(point)
  

  def addPointVector(self, vPoint):
    point = Point(vPoint)
    self.points.append(point)
 


import math
from pointstruct import Point, getDistance, getMagnitude
from pathstruct import Path, Vector

#gets the curvature of a point (p1)
def calcCurvature(p1, p2, p3):

  #find radius
  k1 = 0.5*( math.pow(p1.x,2) + math.pow(p1.y, 2) - math.pow(p2.x,2) - math.pow(p2.y,2)   ) /(p1.x - p2.x +0.0001)
  k2=(p1.y - p2.y)/(p1.x - p2.x)

  b= 0.5*( math.pow(p2.x,2) - 2*p2.x *k1 + math.pow(p2.y,2) - math.pow(p3.x,2) +2* p3.x *k1 - math.pow(p3.y,2))/(p3.x *k2 -p3.y +p2.y -p2.x *k2)
  a=k1 - k2 *b

  r = math.sqrt( math.pow((p1.x - a),2) +math.pow((p1.y - b),2) )

  #curvature = 1/radius
  curvature = 1/r

  return curvature

#checks if a line segment intersects with circle returns t value [0,1] 
def calcIntersection(startSegment, endSegment, robotCentre, radius):
  t1 = 0
  t2 = 0
  d = Vector(startSegment, endSegment)

  f = Vector(robotCentre, startSegment)


  a = d.Dot(d)
  b = 2 * f.Dot(d)
  c = f.Dot(f) - (radius * radius)
  discriminant = (b * b) - (4 * a * c)

  
  if (discriminant < 0):
    return -1; #no intersection
  
  else :
    discriminant = math.sqrt(discriminant)
    
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    print("t1: ",t1, "  t2: ", t2)

    if (t1 >= 0.0 and t1 <= 1.0):
        #return t1 intersection
        #print("t1: ",t1)

        return t1
    
    if (t2 >= 0.0 and t2 <= 1.0):
        #eturn t2
        #print("t2: ",t2)
        return t2
    
    return -1; #return no intersection
  



#Calculate tVal + index
def calcFractionalT(path, robotPos, lookahead, startingLargest) :
  print("robotPos: ", robotPos.x, " , ", robotPos.y)
  print("lookahead: ", lookahead)
  largestTVal = startingLargest
  
  #loop through path up until last point
  for i in range (0,len(path.points)-1, 1):
    #start and end points of vector
    startPoint = path.getPoint(i)
    endPoint = path.getPoint(i + 1)
    print("startpoint: ", startPoint.x, " , ", startPoint.y)
    print("endpoint: ", endPoint.x, " , ", endPoint.y)

    currentTVal = calcIntersection(startPoint, endPoint, robotPos, lookahead)
    print("currentTVal: ", currentTVal, "   index: ", i)
    if (currentTVal >= 0.000 and currentTVal <= 1.000):


      if (currentTVal + i >= largestTVal):
        
        largestTVal = currentTVal + i
      
    
    
  
  return largestTVal


#clamp function
def clamp(d, min, max):
  if (d<min):
    t = min
  else :
    t = d
  
  if (t>max):
    return max
  else:
    return t

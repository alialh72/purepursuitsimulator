from pathstruct import Path, Vector
from pointstruct import Point, getDistance, getMagnitude
from ratelimiter import limiter
import config
from calcfuncs import calcIntersection, calcCurvature, calcFractionalT, clamp
import math

def initPure() :
  config.prevClosestPointIndex = 0
  config.shortestDistance = getDistance(config.finalPosition, config.finalPath.getPoint(len(config.finalPath.points)-1))
  config.closestPointIndex = len(config.finalPath.points)
  config.l.reset()
  config.targetLW = 0
  config.targetRW = 0


#------------------------------------------
#fill valls
#------------------------------------------
def FillPointVals(cpath) :
  path = cpath #make copy of variable

  runningDistance = 0.0
  for i in range(0, len(path.points), 1):

    #______Set the point distances from start of path______
    pointDistance = 0.0

    #if point is not the first
    if (i != 0) :
      #find the difference between point i and point i-1
      pointDiff = getDistance(path.getPoint(i-1), path.getPoint(i)); 

      pointDistance = runningDistance + pointDiff
      runningDistance += pointDiff
     

    path.points[i].setDistance(pointDistance)


    #______Set curvature of points______
    if (i != 0 and i != len(path.points)-1) :
      curvature = calcCurvature(path.getPoint(i), path.getPoint(i-1), path.getPoint(i+1)); 
      path.points[i].setCurvature(curvature)
    else :
      path.points[i].setCurvature(1)
    

    #______Set max velocity of points______
    #Set max velocity of point i to a minimum of (Max Path Velocity , k/curavture at point i)
    path.points[0].setMaxVelocity(min(config.maxPathVelocity, config.kMaxVel/path.getPoint(i).curvature))
  

  #----------Calculate Target Velocities-----------
  #1. simulate a robot running the path with max acceleration, starting at the end
  #2. vf = √(vi^2 + 2*a*d).
  # --> new velocity at point i = min(old target velocity at point i, √(velocity at point (i + 1))2 + 2 * a * distance )

  #Set velocity of last point to 0
  path.points[len(path.points)-1].setTargetVelocity(0)

  #loop through points back to front
  for i in range(len(path.points)-2 , -1, -1):
    pointDiff = path.getPoint(i+1).distanceFromStart - path.getPoint(i).distanceFromStart

    newTargetVelocity = min(path.getPoint(i).maximumVelocity , math.sqrt(math.pow(path.getPoint(i+1).targetVelocity,2) + (2*config.maxAcceleration*pointDiff)) )

    path.points[i].setTargetVelocity(newTargetVelocity)

  return path


#-------------------------------------------
#Follow path functions
#-------------------------------------------

def findClosestPoint() :
  #______Find closest point______
  #Start at prev point index +1

  #by default, set shortest distance to distance from last point
  
  if(config.prevClosestPointIndex < len(config.finalPath.points)-1) :
    for i in range(config.prevClosestPointIndex+1, len(config.finalPath.points), 1):
      config.robotDistance = getDistance(config.finalPosition, config.finalPath.getPoint(i))

      if (config.robotDistance < config.shortestDistance) :
        config.shortestDistance = config.robotDistance
        config.closestPointIndex = i
      
    
  config.closestPoint = config.finalPath.getPoint(config.closestPointIndex)
  config.prevClosestPointIndex = config.closestPointIndex
  config.targetVel = config.closestPoint.targetVelocity


def findLookaheadPoint() :
  #TODO: add check if lookahead point is last point (case if startT + 1 is out of index range)
  #______Find lookahead point______
  print("closest: ",config.lookaheadPointIndex)
  tVal = calcFractionalT(config.finalPath, config.finalPosition, config.lookaheadDistance, config.lookaheadPointIndex)
  

  #round t val down to get the start tval
  startT = int(tVal); #get rids of decimals (converting double to int drops decimals)
  #add 1 to find end point  index
  
  config.lookaheadPointIndex = startT
  if (startT + 1 <= len(config.finalPath.points) - 1) :
    lookaheadSegment = Vector(config.finalPath.getPoint(startT), config.finalPath.getPoint(startT + 1))
    #similar triangles: 
    #Get only the decimal(the fractional distance along the line)
    nonIndexedVal = math.fmod(tVal, 1); # y = x % 1  = 0.45 

    #find new hypotenuse and x y difference
    similarHyp = nonIndexedVal * lookaheadSegment.magnitude
    xDiff = similarHyp * math.cos(lookaheadSegment.refAngle)
    yDiff = similarHyp * math.sin(lookaheadSegment.refAngle)

    #angle stays the same
    #find new x & y distance
    #depending on quadrant, either add/subtract the x and y to the initial point
    if(lookaheadSegment.quadrant == 1):
        config.lookaheadPoint.setCoordinates([lookaheadSegment.startPoint.x + xDiff, lookaheadSegment.startPoint.y + yDiff ])

    if(lookaheadSegment.quadrant == 2):
        config.lookaheadPoint.setCoordinates([lookaheadSegment.startPoint.x + xDiff, lookaheadSegment.startPoint.y - yDiff ])

    if(lookaheadSegment.quadrant == 3):
        config.lookaheadPoint.setCoordinates([ lookaheadSegment.startPoint.x - xDiff, lookaheadSegment.startPoint.y - yDiff ])

    if(lookaheadSegment.quadrant == 4):
        config.lookaheadPoint.setCoordinates([lookaheadSegment.startPoint.x - xDiff, lookaheadSegment.startPoint.y + yDiff ])

    if(lookaheadSegment.quadrant == 5):     #straight up
        config.lookaheadPoint.setCoordinates([ lookaheadSegment.startPoint.x, lookaheadSegment.startPoint.y + yDiff ])


    if(lookaheadSegment.quadrant == 6): #straight right
        config.lookaheadPoint.setCoordinates([lookaheadSegment.startPoint.x + xDiff, lookaheadSegment.startPoint.y ])


    if(lookaheadSegment.quadrant == 7):  #straight down
        config.lookaheadPoint.setCoordinates([lookaheadSegment.startPoint.x, lookaheadSegment.startPoint.y - yDiff ])


    if(lookaheadSegment.quadrant == 8):       #traight left
        config.lookaheadPoint.setCoordinates([lookaheadSegment.startPoint.x - xDiff, lookaheadSegment.startPoint.y ])

    #____________set vals of lookahead____________

    #______Distance from start______
    distanceDiff = getDistance(config.lookaheadPoint, config.finalPath.getPoint(startT))
    newDistance = config.finalPath.getPoint(startT).distanceFromStart + distanceDiff
    config.lookaheadPoint.setDistance(newDistance)

    #______Curvature______
    curvature = 1

    if (startT > 0 and startT < len(config.finalPath.points)-2):
      curvature = calcCurvature(config.lookaheadPoint, config.finalPath.getPoint(startT - 1), config.finalPath.getPoint(startT + 2))
    
    else :
      curvature = 1
    
      
    config.lookaheadPoint.setCurvature(curvature)

    #______Set max velocity of lookahead____
    #Set max velocity of point i to a minimum of (Max Path Velocity , k/curavture at point i)
    config.lookaheadPoint.setMaxVelocity(min(config.maxPathVelocity, config.kMaxVel / config.lookaheadPoint.curvature))

    #______Set target velocity of lookahead____
    # --> new velocity at point i = min(old target velocity at point i, √(velocity at point (i + 1))2 + 2 * a * distance )
  
    pointDiff = config.finalPath.getPoint(startT + 1).distanceFromStart - config.lookaheadPoint.distanceFromStart

    newTargetVelocity = min(config.lookaheadPoint.maximumVelocity, math.sqrt(math.pow(config.finalPath.getPoint(startT + 1).targetVelocity, 2) + (2 * config.maxAcceleration * pointDiff)))

    config.lookaheadPoint.setTargetVelocity(newTargetVelocity)

  else:
    config.lookaheadPoint = config.finalPath.getPoint(len(config.finalPath.points)-1)
  
    
def findCurvature() :
  #find curvature of current robot arc
  #curvature = 2x/L^2
  #where x is horizontal distance to the point and L is the lookahead

  config.aCurvatureSlope =  - (math.atan(config.absOrientation))
  config.bCurvatureSlope = 1
  config.cCurvatureSlope = (math.atan(config.absOrientation) * config.finalPosition.x) - config.finalPosition.y

  config.relativeX = abs(config.aCurvatureSlope*config.lookaheadPoint.x + config.bCurvatureSlope*config.lookaheadPoint.y + config.cCurvatureSlope) / math.sqrt(math.pow(config.aCurvatureSlope, 2) + math.pow(config.bCurvatureSlope, 2))

  #get signed curvature
  #side = signum(cross product) = signum((By − Ry) * (Lx − Rx) − (Bx − Rx) * (Ly − Ry))
  #Bx = Rx + cos(robot angle)
  #By = Ry + sin(robot angle)


  #sign = signum(cross product)
  config.crossProduct =  (math.cos(config.absOrientation) * (config.lookaheadPoint.x - config.finalPosition.x)) - (math.sin(config.absOrientation) * (config.lookaheadPoint.y - config.finalPosition.y))

  #signum ternary operator
  if (config.crossProduct > 0):
    config.side = 1
  elif (config.crossProduct < 0):
    config.side = -1
  else:
    config.side = 0

  config.signedCurvature = ((2*config.relativeX)/math.pow(config.lookaheadDistance, 2))*config.side

def calculateWheelVelocities() :
  #calculate wheel velocities
  #
  #V = target robot velocity
  #L = target left wheel’s speed
  #R = target right wheel’s speed
  #C = curvature of arc
  #W = angular velocity of robot
  #T = track width

  #V = (L + R)/2
  #W = (L − R)/T
  #V = W/C
  
  config.targetVel = config.l.rateLimiter(config.targetVel, config.maxAcceleration)
  #targetVel = closestPoint.targetVelocity;

  config.targetLW = config.targetVel * (2 + (config.signedCurvature * config.trackWidth) )/2
  config.targetRW = config.targetVel * (2 - (config.signedCurvature * config.trackWidth) )/2


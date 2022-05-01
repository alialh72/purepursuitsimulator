from pathstruct import Path, Vector
from pointstruct import Point, getDistance, getMagnitude
from ratelimiter import limiter

#------BOOLEANS-----------
enableOdom = True
enableFollowPath = True

#------VARS--------
desiredPath = Path()
finalPath = Path()
lookaheadPoint = Point([0,0])
closestPoint = Point([0,0])
curvature = 0.0
targetVelRight = 0.0
targetVelLeft = 0.0
targetVelRobot = 0.0
angularVel = 0.0
finalPosition = Point([0,0])

#------PARAMETERERS-------
maxPathVelocity = 100.0 #inches per second
kMaxVel = 3.0 #[1,5] higher k --> faster around turns 
lookaheadDistance = 13.3
maxAcceleration = 100.0 #incher per second^2


#------PURE PURSUIT LOOP VARIABLES-------
pathSize = 0.0
shortestDistance = 0.0
prevClosestPointIndex = 0.0
closestPointIndex = 0.0
lookaheadPointIndex = 0.0

aCurvatureSlope = 0.0
bCurvatureSlope = 0.0
cCurvatureSlope = 0.0
relativeX = 0.0
side = 0.0
crossProduct = 0.0
signedCurvature = 0.0

l = limiter()
#-----CALC VELOCITY--------
targetVel = 0.0
targetRW = 0.0
targetLW = 0.0 #target right/left wheel velocities

#-----Path Numbers---------

paths = Path()
import turtle
from calcfuncs import calcFractionalT, calcIntersection
from pointstruct import Point, getMagnitude, getDistance
from pathstruct import Path, Vector
from unicodedata import decimal
from purepursuit import FillPointVals, findClosestPoint, findLookaheadPoint , initPure, findCurvature, calculateWheelVelocities
import config

t = turtle.Turtle()
screen = t.getscreen()


delay = 20
scale = 5
points = []
with open("path.txt",'r') as file:
    for line in file:
        l = line.strip().split(',')
        pointx = float(l[0])
        pointy = float(l[1])

        point = Point([pointx, pointy])

        points.append(point)

config.finalPath = Path()
config.finalPath.points = points
config.finalPath = FillPointVals(config.finalPath)
t.pensize(3)
t.color('red')
for p in points:
    t.goto(p.x * scale, p.y * scale)


initPure()
findClosestPoint()
findLookaheadPoint()
#path = Path()
#path.points = [Point([26.767,29.012]), Point([27.357,33.673]), Point([27.89,38.398])]
#tval = calcIntersection(Point([2.5,2]), Point([6,3]), Point([3.9,2.2]), 1.5)
#print("_____finaltval: ", tval)
print("lookaheadPoint: ", config.lookaheadPoint.x, " , " , config.lookaheadPoint.y)

#move robot Pos
initPure()
while (getDistance(config.finalPosition, config.finalPath.points.at[len(config.finalPath.points-1)]) >= 5):
    findClosestPoint()
    findLookaheadPoint()
    findCurvature()
    calculateWheelVelocities()
    

screen.exitonclick()
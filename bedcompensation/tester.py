import pprint
from collections import namedtuple
import random
import math
import numpy as np

#A playpen for testing ideas related to automatic bed compensation for reprap firmware.

#determine if a line segment crosses the horizontal, virtical or diagonal line in a 2D mesh
#The mesh runs from 0,0 onwards with diagonal lines running with a positive gradient of 1.
def whichLine(x1,y1,x2,y2):
	goesRight = x2>x1
	goesUp = y2>y1
	positionInBoxX = x1%1
	positionInBoxY = y1%1

	inB = (positionInBoxX+positionInBoxY)>1
	inA = not inB

	if goesRight:
		distanceToX = 1-positionInBoxX
	else:
		distanceToX = -positionInBoxX

	if goesUp:
		distanceToY = 1-positionInBoxY
	else:
		distanceToY = -positionInBoxY


	overallGradient = (y2-y1)/(x2-x1)
	toCornerGradient = distanceToY/distanceToX

	steeper = abs(overallGradient)>abs(toCornerGradient)

	print "%f,%f -> %f,%f: (overallGradient:%f)(toCornerGradient:%f)(steeper:%r)"%(x1,y1,x2,y2,overallGradient,toCornerGradient, steeper)

    #1) figure out which box wall would be intersected
	#steeper is horiz
	# else virt

	#2) now refine for if we would have crossed the diagonal line on the way
	#if A == up then horiz becomes diag
	#if A == right then virt becomes diag

	if steeper:
		out = "h"
	else:
		out = "v"

	if inA==goesUp and out=="h":
		out="d"

	if inA==goesRight and out=="v":
		out = "d"

	print out

#for pX in [0.1, 0.9]:
#	for pY in [0.1, 0.9]:
#		for tX in range(-3,4):
#			for tY in range(-3,4):
#				whichLine(pX,pY,tX,tY) 


#Probing:
#probe a whole row of Z
#while we've still got more rows of z probes to do:
#   probe the next line building triangles as we go.
#     First point: do nothing
#     Each subsequent point until EOL (pX,pY):
#			Add triangle A: (pX-1,pY),(pX-1,pY-1),(pX,pY-1)
#			Add triangle B: (pX-1,pY),(pX,pY),(pX,pY-1)

##the X and Y probe counts
probeX = 4
probeY = probeX

triangleCount = (probeX-1)*(probeY-1)*2

#not strictly a triangle but the coefficients of the plane equation defined by a triangular probed region
Triangle = namedtuple("Triangle", "a b c d")

#This is where triangles will live:
triangles = range(triangleCount)

#Constructs a plane equation from a triangle of three 3D points
#(http://stackoverflow.com/questions/1985427/plane-equation-for-3d-vectors)
def mkTriangle(p1,p2,p3):
	x1, y1, z1 = p1
	x2, y2, z2 = p2
	x3, y3, z3 = p3
	v1 = [x3 - x1, y3 - y1, z3 - z1]
	v2 = [x2 - x1, y2 - y1, z2 - z1]
	cp = [v1[1] * v2[2] - v1[2] * v2[1],
      	  v1[2] * v2[0] - v1[0] * v2[2],
      	  v1[0] * v2[1] - v1[1] * v2[0]]

    #got the normal, now normalize it:
	magnitude = math.sqrt(cp[0]*cp[0]
    	                 +cp[1]*cp[1]
    	                 +cp[2]*cp[2]);

	cp[0] = cp[0]/magnitude;
	cp[1] = cp[1]/magnitude;
	cp[2] = cp[2]/magnitude;
	#the normal is now normalized.
	a, b, c = cp

	d = a * x1 + b * y1 + c * z1

	#pprint.pprint(locals())
	return Triangle(a,b,c,d);




def probePoint():
	return random.random()

def probeBed():
	onTriangle = 0

	#populate the first row:
	previousRow = range(probeX)
	for i in range(probeX):
		previousRow[i] = probePoint()
		print "%.3f"%previousRow[i],
	print

	#now probe all subsequent rows:
	for y in range(1,probeY):
		thisRow = range(probeX)
		#get the first point in the row
		thisRow[0] = probePoint()

		#for each subsequent point in the row construct triangles.
		for x in range(1,probeX):
			thisRow[x] = probePoint();
			#triangle A: (pX-1,pY),(pX-1,pY-1),(pX,pY-1)
			triangles[onTriangle] = mkTriangle([x-1,y,thisRow[x-1]],[x-1,y-1,previousRow[x-1]],[x,y-1,previousRow[x]])

			#Add triangle B: (pX-1,pY),(pX,pY),(pX,pY-1)
			triangles[onTriangle+1] = mkTriangle([x-1,y,thisRow[x-1]],[x,y,thisRow[x]],[x,y-1,previousRow[x]])
			onTriangle = onTriangle+2;

		previousRow = thisRow
		for i in range(len(previousRow)):
			print "%.3f"%previousRow[i],
		print

	#done
	pass

#so... probe our virtual bed!
print "Simulated bed height readings:"
probeBed()

#great! now let's try moving across our "bed"

currentPosition = [0,0,0]

#gets the z height at the x y provided by looking it up in the mesh
def getZatPoint(x,y):
	#1) determine which square this point is in
	inSX = x%1;
	inSY = y%1;
	sqX = x-inSX #do it this way so it'll work when squares are not 1 unit across
	sqY = y-inSY
	#2) is this the A or the B triangle?
	isB=0
	if (inSX+inSY)>1:
		isB=1
	#3) Calculate triangle index in array
	tIdx = int(sqX*2 + sqY*(probeX-1)*2 + isB)


	#4) Calculate Z from the plane equation of this triangle
	plane = triangles[tIdx];

	z = (plane.d - plane.a * x - plane.b * y) / plane.c

	return z

	pass

#print triangles
print
print "Estimated heights at (x,y) using the bed mesh:"
for y in np.arange(0,probeY-1,0.3):
	for x in np.arange(0,probeX-1,0.3):
		print "(%.1f,%.1f):%.3f 	" % (x,y,getZatPoint(x,y)),
	print





import pprint

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
	
for pX in [0.1, 0.9]:
	for pY in [0.1, 0.9]:
		for tX in range(-3,4):
			for tY in range(-3,4):
				whichLine(pX,pY,tX,tY) 




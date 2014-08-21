#!/usr/bin/env python

import argparse
import pprint
import copy
import math
from collections import namedtuple
import re
import sys

PARAMS_LINE = "Mesh parameters:"
PROBE_ROW = "Probe row:"
UNDEFINED_POINT = -99.0
DISABLED_ZHEIGHT = 10000
IGNORE_DISTANCE = 999999
#moves smaller than this will just go straight to their destination without cutting
FASTPATH_DISTANCE_SQR = 10 #(mm^2)

BASIC_GCODE_RE = re.compile("(([a-zA-Z])(-?[\d.]+))")

#not strictly a triangle but the coefficients of the plane equation defined by a triangular probed region
Triangle = namedtuple("Triangle", "a b c d")

bedoffsetX = 0
bedoffsetY = 0
probespacing = 0
probeX = 0

#Constructs a plane equation from a triangle of three 3D points
#(http://stackoverflow.com/questions/1985427/plane-equation-for-3d-vectors)
#...what can I say. School was a long time ago.
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

def loadProbeData():
	global args
	global bedoffsetX
	global bedoffsetY
	global probespacing
	global probeX

	rows = []

	#1) Read in probe data
	for inline in args.probedata.readlines():
		#find the params line:
		if (inline.find(PARAMS_LINE)>-1):
			params = [float(p) for p in inline.split(PARAMS_LINE)[1].split()]
			bedoffsetX = params[0]
			bedoffsetY = params[1]
			probespacing = params[4]


		#process a probe line:
		if inline.find(PROBE_ROW)>-1:
			row = inline.split(PROBE_ROW)[1].split()
			rows.append([float(f) for f in row])




	#2) fill in missing probe points
	#This is done by a very naiive algorithm n^2 on probe points! (or n^4 wrt probes count in X, assuming square probe zone)
	#Idea is for each undefined probe point find the closest probe point that is defined and use it's value.
	#No effort is put into efficiently performing this.
	#This seems like it should be fine considering a 50cm x 50cm bed with 1cm probing (overkill)
	#... gives ~~50^4 operations (6.25m) This is fine even in python. computers are faster than I can code better :D

	# print "Before:"
	# for r in rows:
	# 	print r

	fixedRows = copy.deepcopy(rows)
	ys = len(rows)
	probeX = xs = len(rows[0])

	for y in range(ys):
		for x in range(xs):
			if rows[y][x] == UNDEFINED_POINT:
				#undefined point! let's go hunting
				dist = 1000
				pval = 1000;

				for ty in range(ys):
					for tx in range(xs):
						if rows[ty][tx]>UNDEFINED_POINT:
							tDist = math.sqrt((tx-x)**2+(ty-y)**2)
							if tDist<dist:
								dist = tDist
								pval = rows[ty][tx]
				#print "found surrogate for (%d,%d) at (%d,%d) with val: %.2f"%(x,y,cx,cy,pval)
				fixedRows[y][x] = pval

			else:
				#defined point: put it in the new array
				fixedRows[y][x] = rows[y][x]

	print "Using heightmap:"
	for r in fixedRows:
		print r

	#3) Construct mesh from points and calculate plane equations for each triangle in the mesh
	onTriangle = 0
	global triangles

	triangleCount = (xs-1)*(ys-1)*2

	triangles = range(triangleCount)

	for y in range(1,ys):
		for x in range(1,xs):
			previousRow = fixedRows[y-1]
			thisRow = fixedRows[y]

			posX = x*probespacing+bedoffsetX
			posY = y*probespacing+bedoffsetY
			posXm1 = (x-1)*probespacing+bedoffsetX
			posYm1 = (y-1)*probespacing+bedoffsetY

			#triangle A: (pX-1,pY),(pX-1,pY-1),(pX,pY-1)
			triangles[onTriangle] = mkTriangle([posXm1,posY,thisRow[x-1]],[posXm1,posYm1,previousRow[x-1]],[posX,posYm1,previousRow[x]])

			#Add triangle B: (pX-1,pY),(pX,pY),(pX,pY-1)
			triangles[onTriangle+1] = mkTriangle([posXm1,posY,thisRow[x-1]],[posX,posY,thisRow[x]],[posX,posYm1,previousRow[x]])
			onTriangle = onTriangle+2;

	#done building map
	print "Mesh built and contains %d triangles."%triangleCount




currentPosition = [0.0,0.0,0.0,0.0]

def getTriangleIndex(x,y):
	global probespacing
	global bedoffsetX
	global bedoffsetY

	wX = x - bedoffsetX
	wY = y - bedoffsetY
	
	#1) determine which square this point is in
	inSX = (wX/probespacing)%1; #offset within a probe square, in the range 0-1
	inSY = (wY/probespacing)%1;
	sqX = int(wX/probespacing) #do it this way so it'll work when squares are not 1 unit across
	sqY = int(wY/probespacing)
	#2) is this the A or the B triangle?
	isB=0
	if (inSX+inSY)>1:
		isB=1
	#3) Calculate triangle index in array
	tIdx = int(sqX*2 + sqY*(probeX-1)*2 + isB)

	return tIdx

#gets the z height at the x y provided by looking it up in the mesh
def getZatPoint(x,y):
	
	tIdx = getTriangleIndex(x,y)

	#4) Calculate Z from the plane equation of this triangle
	plane = triangles[tIdx];

	z = (plane.d - plane.a * x - plane.b * y) / plane.c

	return z



#Gets the remapped Z height for a requested point in 3D space.
# if z >= correctZByHeight then returns z
#
# If the skew correction height is disabled then all z points will be adjusted by the printbed offset at the specified (x,y)
#
# otherwise maps z according to the linear scaling:
# 0-correctZByHeight -> getZatPoint(z)-correctZByHeight
def mappedZ(x,y,z):
	if correctZByHeight==DISABLED_ZHEIGHT:
		#never correct skew:
		return getZatPoint(x,y)+z;
	elif z >= correctZByHeight:
		return z
	else:
		return getZatPoint(x,y) * (1-z/correctZByHeight) + z

previousPointEMult = 1.0

#emits the given dictionary as a gcode line on args.output
def emit(codes):
	global args
	out = args.output

	codes = copy.copy(codes)

	#make sure command codes go out first
	firsts = ['G','M']
	doneFirst = False
	for l in firsts:
		if l in codes:
			out.write("%s%.5f "%(l,int(codes[l])))
			del codes[l]
			doneFirst = True

	if not doneFirst:
		print "Can't emit gcode without command!"
		pprint.pprint(locals())
		quit()

	for k in codes.keys():
		out.write("%s%.5f "%(k,float(codes[k])))
	out.write("; relevelled\n");


splitEPosition = 0

#issues a gcode move to the requested x,y,z,e
#
# absolute coords for all axis!
#
#if travel is True then no E code is emitted
#all extra codes in extras are emitted too.
#the extra codes should contain the G/M code command!
def commitMove(x,y,z,e,travel,extras):
	global currentPosition
	x1,y1,z1,e1 = currentPosition

	tZ = mappedZ(x,y,z)
	global previousPointEMult
	if correctZByHeight==DISABLED_ZHEIGHT:
		moveEMult = 1.0
	else:
		gotoEMult = (correctZByHeight-getZatPoint(x,y))/correctZByHeight
		moveEMult = (gotoEMult+previousPointEMult)/2
		previousPointEMult = gotoEMult

	#now calculate sane e value:
	#This is done by calculating the relative E for this move segment
	#then scaling this inversely to the layer height modification 
	#finally adding this new relative E to the last E position we emitted.
	eRel = e-e1
	eCompensated = eRel*moveEMult
	global splitEPosition
	tE = eCompensated+splitEPosition
	if eRel<0 and not travel:
		print "Non-travel move with negative E!"
		pprint.pprint(locals())
		quit()

	if moveEMult<0.5 or moveEMult>2:
		print "moveEMult probably wrong: %.2f%%, gotoEMult: %.2f%%"%(moveEMult*100, gotoEMult*100)
		pprint.pprint(locals())

	#print "Committing move from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) e: %.2f (e%%:%.2f)"%(x1,y1,z1,x,y,z,e,moveEMult*100)
	#print "Actual z used: %.2f"%tZ
	xdiff = x-x1
	ydiff = y-y1

	dst = math.sqrt(xdiff**2+ydiff**2)

	global moveDst
	moveDst = moveDst + dst

	# if xdiff!=0:
	# 	m=(y-y1)/(x-x1)
	# 	c = y1 - m*x1
	# 	print "Sanity check: y=%.2fx + %.2f		dist:%.2f"%(m,c,dst)
	# else:
	# 	print "Sanity check: virtical line"

	currentPosition[0] = x
	currentPosition[1] = y
	currentPosition[2] = z
	
	extras['X'] = x;
	extras['Y'] = y;
	extras['Z'] = tZ;
	if not travel:
		currentPosition[3] = e
		splitEPosition = tE
		extras['E'] = tE

	emit(extras)



#splits a requested move into many sub-moves
def moveTo(to, travel, extras):
	x2,y2,z2,e2 = to
	x,y,z,e = to
	x1,y1,z1,e1 = currentPosition

	global splitEPosition
	splitEPosition = e1

	#print "Move requested from current position (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f) e: %.2f"%(x1,y1,z1,x,y,z,e)

	totalDistance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
	
	
	global moveDst
	moveDst = 0

	#fastpath:
	if (totalDistance<FASTPATH_DISTANCE_SQR):
		commitMove(x,y,z,e,travel, extras)
		#print "moveTo done. (fastpath)"
		#print
		return

	if x2-x1==0:
		overallGradient = float("+inf")
		#print "Planned line eq: virtical line"
		moveC = 0 #line equations are nonsense for this line
	else:
		overallGradient = (y2-y1)/(x2-x1)
		moveC = y1-overallGradient*x1   # C value for line equation of this move.
		#print "Planned line eq: y=%.2fx + %.2f"%(overallGradient,moveC)

	goesRight = x2>x1
	goesUp = y2>y1

	#a counter of distance completed. (to enable running Z and E counters)
	totalComplete = 0

	zStart = z1
	eStart = e1
	totalRelativeE = e2-eStart
	totalRelativeZ = z2-zStart

	while True:

		x1,y1,z1,e1 = currentPosition
		
		positionInBoxX = x1%probespacing
		positionInBoxY = y1%probespacing
		inboxX = x1 - positionInBoxX
		inboxY = y1 - positionInBoxY

		#print "finding next point to goto",
		#print "from current position (%.2f,%.2f,%.2f)"%(x1,y1,z1)

		
		if goesRight:
			distanceToX = probespacing-positionInBoxX
			nextCrossX = inboxX+probespacing
		else:
			distanceToX = -positionInBoxX
			nextCrossX = inboxX

		if goesUp:
			distanceToY = probespacing-positionInBoxY
			nextCrossY = inboxY+probespacing
		else:
			distanceToY = -positionInBoxY
			nextCrossY = inboxY			

		#point it crosses a virtical edge
		if abs(overallGradient) < sys.float_info.max:
			nextCrossX_y = overallGradient * nextCrossX + moveC
			nextCrossX_y_dst = nextCrossX_y-y1
			nextCrossXdstS = distanceToX**2 + nextCrossX_y_dst**2
			#we dont care about the X crossing if we're already on this point.
			if (nextCrossXdstS<=0): nextCrossXdstS = IGNORE_DISTANCE
			#print "will cross X-line at (%.2f,%.2f) in %.2f"%(nextCrossX,nextCrossX_y,math.sqrt(nextCrossXdstS))
		else:
			#In this case we're moving parallel to the Y axis so will never cross an X line.
			nextCrossXdstS = IGNORE_DISTANCE

		
		#point it crosses a horizontal edge
		if overallGradient != 0:
			if abs(overallGradient) < sys.float_info.max:
				#normal non-infinite gradient
				nextCrossY_x = (nextCrossY-moveC)/overallGradient
			else:
				#infinite gradient means a move without chaning X value.
				#(the move is parallel to Y-axis)
				nextCrossY_x = x1
			nextCrossY_x_dst = nextCrossY_x-x1
			nextCrossYdstS = distanceToY**2 + nextCrossY_x_dst**2
			#we dont care about the Y crossing if we're already on this point.
			if (nextCrossYdstS<=0): nextCrossYdstS = IGNORE_DISTANCE
			#print "will cross Y-line at (%.2f,%.2f) in %.2f"%(nextCrossY_x,nextCrossY,math.sqrt(nextCrossYdstS))
		else:
			#In this case we're moving parallel to the X axis so will never cross a Y line.
			nextCrossYdstS = IGNORE_DISTANCE


		
		#point it crosses a diagonal edge
		#This is more complex as we dont't already know the X or the Y or even which diagonal line we'll cross next.
		#for any given point, we're bounded by two diagonal lines both with gradients of -1
		# So, to work out which diagonal line we cross: (and the lines are distiguished by their Y-intercept.
		# in this explanation c is the Y intercept of the diagonal line that goes through the box our current point is in.		

		#Four predicates are used: inA, steep, goesUp and goesRight, so to find the Y-intercept of interest:

		#if we're in A:
		#	steep, up: c
		#   steep, down: c-1
		#   !steep, right: c
		#	!steep, left: c-1
		
		#if we're in B:
		#   steep, up: c+1
		#	steep, down: c
		#   !steep, right: c+1
		#   !steep, left: 	c

		#to simplify this:
		# Let's create a predicate goingPositive that is goesUp if steep or goesRight if !steep.
		# We can see that the table for B is the same as A but we +1 to the result in all cases.
		# So combining these we get:

		# y-intercept = c-1
		# if goesPositive: y-intercept ++
		# if B: y-intercept ++

		# Here goes:

		if overallGradient!=-1:
			steep = abs(overallGradient)>1
			if steep:
				goesPositive = goesUp
			else:
				goesPositive = goesRight
			
			#The y-intercept for the currentbox is the (_x_,_y_)+1, so let's start with this value-1:

			diagYintercept = inboxY+inboxX
			inB = (positionInBoxX+positionInBoxY)>probespacing

			if goesPositive:
				diagYintercept = diagYintercept + probespacing

			if inB:
				diagYintercept = diagYintercept + probespacing

			nextCrossD_x = (diagYintercept-moveC) / (overallGradient+1)
			#use the D-line equation as it's guaranteed to be sensible but the moveTo line might have a stupid graident.
			nextCrossD_y = -1 * nextCrossD_x + diagYintercept
			ncd_dx = (nextCrossD_x-x1)
			ncd_dy = (nextCrossD_y-y1)
			#make sure we only accept line crossings in the direction we're going.
			#This test should be redundant now. (due to more rigorious line finding method)
			if ((ncd_dx<0) == goesRight) or ((ncd_dy<0) == goesUp):
				nextCrossDdstS = IGNORE_DISTANCE
			else:
				nextCrossDdstS = ncd_dx**2 + ncd_dy**2
			
			#we dont care about the D crossing if we're already on this point.
			if (nextCrossDdstS<=0): nextCrossDdstS = IGNORE_DISTANCE

			#print "will cross D-line at (%.2f,%.2f) in %.2f"%(nextCrossD_x,nextCrossD_y,math.sqrt(nextCrossDdstS))
		else:
			#parallel to this line so no distance to intercept.
			nextCrossDdstS = IGNORE_DISTANCE


		targetDstS = (x2-x1)**2 + (y2-y1)**2
		#print "target is %.2f away."%math.sqrt(targetDstS);

		closest = min(nextCrossXdstS,nextCrossYdstS,nextCrossDdstS,targetDstS);

		#print "closest crossing is %.2f far away."%math.sqrt(closest)

		done = False
		if closest==targetDstS:
			#print "moving to the target",
			moveToX = x2
			moveToY = y2
			done = True
		elif nextCrossXdstS==closest:
			#print "moving to the next X-crossing",
			moveToX = nextCrossX
			moveToY = nextCrossX_y
		elif nextCrossYdstS==closest:
			#print "moving to the next Y-crossing",
			moveToX = nextCrossY_x
			moveToY = nextCrossY
		elif nextCrossDdstS==closest:
			#print "moving to the next D-crossing",
			moveToX = nextCrossD_x
			moveToY = nextCrossD_y

		#do not set moveToX/moveToY by default to fail fast if none matched.


		#print "at (%.2f,%.2f)"%(moveToX,moveToY)

		#How much of the remaining line is being consumed?
		totalComplete += math.sqrt(closest)
		fractionComplete = totalComplete/totalDistance

		#calculate new E position (assuming absolute E is wanted by the consumer)
		eMove = eStart + totalRelativeE*fractionComplete

		#Calculate what Z should be at the end of this sub-move
		zMove = zStart + totalRelativeZ*fractionComplete


		commitMove(moveToX,moveToY,zMove,eMove,travel,extras)

		if done:
			#print "Total dist: %.2f		Segment Sum: %.2f"%(totalDistance, moveDst)
			if abs(totalDistance - moveDst)>10**-6:
				print "WARNING: broken maths detected! segments don't sum to whole length accurately."
				pprint.pprint(locals())
				quit()
			#print "moveTo done."
			#print
			return

	
def processGcode():
	print "Processing input..."
	global args

	out = args.output

	out.write("; WARNING: processed by relevel.py!\n; This should only be printed on the matching print bed!\n")

	#we start at 0,0,0,0

	global currentPosition

	nextReportZ = 0.0
	passedCorrection = False

	#this is to make sure we don't modify the first G0/1 as we don't know the position before this command.
	#(we don't know where home is and don't want to know ideally)
	firstMove = True

	lineno = 0
	#process the gcodes one by one
	for inline in args.input.readlines():
		lineno = lineno +1
		# if lineno>800+200+25+6:
		# 	quit()
		#figure out what this line means:
		#Strip comments
		pline = inline.upper().split(";")[0]
		#do a basic parse
		codes = {}
		for code in BASIC_GCODE_RE.findall(pline):
			codes[code[1]] = code[2]
		
		#determine if this is a code we're interested in, and if not then just emit thee original?
		if 'G' in codes:
			gval = int(codes['G'])
			if gval == 91:
				print "Uh Oh! This gcode uses relative coordinates which are unsupported."
				quit()
			elif gval == 2 or gval == 3:
				print "Input gcode uses arcs which are not supported."
				quit()
			elif gval == 92:
				#Reset the position
				if 'X' in codes:
					currentPosition[0] = float(codes['X'])
				if 'Y' in codes:
					currentPosition[1] = float(codes['Y'])
				if 'Z' in codes:
					currentPosition[2] = float(codes['Z'])
				if 'E' in codes:
					currentPosition[3] = float(codes['E'])
				out.write(inline)
			elif gval == 0 or gval == 1:

				dest = copy.copy(currentPosition)

				moving = False
				if 'X' in codes:
					dest[0] = float(codes['X'])
					moving = True
				if 'Y' in codes:
					dest[1] = float(codes['Y'])
					moving = True
				if 'Z' in codes:
					dest[2] = float(codes['Z'])
					moving = True
				if 'E' in codes:
					dest[3] = float(codes['E'])
					travel = False
				else:
					travel = True

				if not travel and not moving:
					#E-only move. It's important to emit these right away as they confuse the other logic. (They go negative, for example)
					currentPosition[3] = dest[3]
					out.write(inline)
					continue

				if dest[2]>nextReportZ:
					print "Now on Z: %d"%nextReportZ
					if nextReportZ>correctZByHeight and not passedCorrection:
						passedCorrection = True
						print "Passed correction point."
					nextReportZ = int(nextReportZ)+1.0

				#superfastpath:
				#Do we need to even think about this?
				#If our Z is above the geometry mangling threshold then we can just emit the code as-is
				if (dest[2]>args.height and currentPosition[2]>args.height) or firstMove:
					#everything's above the height so just write the line
					out.write(inline)

					#but we do need to update our idea of position though :)
					currentPosition = dest

					firstMove = False;
				else:
					#Make sure interesting bonus codes (and the command code) are still included:
					extras = copy.copy(codes)
					#ensure that coords don't sneak in...
					for tdel in ['X','Y','Z','E']:
						if tdel in extras:
							del extras[tdel]

					#slowpath... Must rewrite this move.
					moveTo(dest, travel, extras)
					#If the gcode requested an E-move and we modified the extrusion amount
					#then now is a good time to let the firmware know the 'expected' value
					if not travel:
						if splitEPosition!=dest[3]:
							#print "Realigning E position from %.3f to %.3f"%(currentPosition[3], dest[3])
							currentPosition[3] = dest[3]
							out.write("G92 E%.5f ; relevel.py \n"%(currentPosition[3]))

			else:
				#not interesting. dump:
				out.write(inline)
		else:
			#not interesting, just dump the original:
			out.write(inline)

	pass


parser = argparse.ArgumentParser(description='Transforms 3D printer gcode to account for unlevel and uneven print surfaces.')
parser.add_argument('-o','--output', type=argparse.FileType('w'), default="relevelled.gcode", help='output filename for transformed gcode')
parser.add_argument('-i','--input', type=argparse.FileType('r'), required=True, help='input gcode to transform')
parser.add_argument('-p','--probedata', type=argparse.FileType('r'), required=True, help='map print bed as returned by G35 from the firmware')
parser.add_argument('-z','--height', type=int, default=5, help='the z-height where tranformations are complete and the print is true. >10000 disables progressive correction so the whole model will be identically skewed at all z-heights.')

args = parser.parse_args()

correctZByHeight = args.height

#load and process probe data
loadProbeData()

processGcode()


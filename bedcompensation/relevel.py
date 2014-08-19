#!/usr/bin/env python

import argparse
import pprint
import copy
import math
from collections import namedtuple
import re

PARAMS_LINE = "Mesh parameters:"
PROBE_ROW = "Probe row:"
UNDEFINED_POINT = -99.0

BASIC_GCODE_RE = re.compile("(([a-zA-Z])(-?[\d.]+))")

#not strictly a triangle but the coefficients of the plane equation defined by a triangular probed region
Triangle = namedtuple("Triangle", "a b c d")

bedoffsetX = 0
bedoffsetY = 0
probespacing = 0

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
	xs = len(rows[0])

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

			posX = bedoffsetX+x*probespacing
			posY = bedoffsetY+y*probespacing
			posXm1 = bedoffsetX+(x-1)*probespacing
			posYm1 = bedoffsetY+(y-1)*probespacing

			#triangle A: (pX-1,pY),(pX-1,pY-1),(pX,pY-1)
			triangles[onTriangle] = mkTriangle([posXm1,posY,thisRow[x-1]],[posXm1,posYm1,previousRow[x-1]],[posX,posYm1,previousRow[x]])

			#Add triangle B: (pX-1,pY),(pX,pY),(pX,pY-1)
			triangles[onTriangle+1] = mkTriangle([posXm1,posY,thisRow[x-1]],[posX,posY,thisRow[x]],[posX,posYm1,previousRow[x]])
			onTriangle = onTriangle+2;

	#done building map
	print "Mesh built and contains %d triangles."%triangleCount

	
def processGcode():
	print "Processing input..."
	global args

	out = args.output

	out.write("; WARNING: processed by relevel.py!\n; This should only be printed on the matching print bed!\n")

	#process the gcodes one by one
	for inline in args.input.readlines():
		#figure out what this line means:
		#Strip comments
		pline = inline.upper().split(";")[0]
		#do a basic parse
		codes = {}
		for code in BASIC_GCODE_RE.findall(pline):
			codes[code[1]] = code[2]
		
		#determine if this is a code we're interested in, and if not then just emit thee original?

	pass


parser = argparse.ArgumentParser(description='Transforms 3D printer gcode to account for unlevel and uneven print surfaces.')
parser.add_argument('-o','--output', type=argparse.FileType('w'), default="relevelled.gcode", help='output filename for transformed gcode')
parser.add_argument('-i','--input', type=argparse.FileType('r'), required=True, help='input gcode to transform')
parser.add_argument('-p','--probedata', type=argparse.FileType('r'), required=True, help='map print bed as returned by G35 from the firmware')
parser.add_argument('-z','--height', type=int, default=5, help='the z-height where tranformations are complete and the print is true. -1 disables progressive correction so the whole model will be uniformly skewedz-height will be skewed.')

args = parser.parse_args()

#load and process probe data
loadProbeData()

processGcode()


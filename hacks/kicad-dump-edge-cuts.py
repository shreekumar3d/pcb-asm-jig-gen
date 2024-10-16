#!/usr/bin/env python3
#
# Script to dump all the shapes in the Edge Cuts layer.
# Exercises the python API to a a just enough degree
#
import re
import pcbnew
import sys
from pprint import pprint
board = pcbnew.LoadBoard(sys.argv[1])
shapes = [x for x in board.GetDrawings() if x.GetLayer()==pcbnew.Edge_Cuts]
for d in shapes:
    if d.GetShapeStr() == 'Circle':
        print('Circle')
        print('  Center = ', d.GetCenter())
        print('  Radius = ', d.GetRadius())
    elif d.GetShapeStr() == 'Arc':
        print('Arc')
        print('  Center = ', d.GetCenter())
        print('  Radius = ', d.GetRadius())
        print('  AngleStart = ', d.GetArcAngleStart().AsDegrees())
        print('  Angle = ', d.GetArcAngle().AsDegrees())
        print('  Mid = ', d.GetArcMid())
        print('  Start Pt = ', d.GetStart())
        print('  End Pt = ', d.GetEnd())
    elif d.GetShapeStr() == 'Line':
        print('Line')
        print('  Start = ', d.GetStart())
        print('  End = ', d.GetEnd())
    elif d.GetShapeStr() == 'Rect':
        print('Rectangle')
        print('  Width = ', d.GetWidth())
        print('  Length = ', d.GetLength()) # not implemented!!
        print('  Corners = ', d.GetRectCorners())
    elif d.GetShapeStr() == 'Polygon':
        print('Polygon')
        shape = d.GetPolyShape()
        # Grr.. how do I extract the freakin points.
        # No API for get polygon ? Looks like only set stuff works from Python,
        # but gets are only in C. Find it hard to believe, 
        # FIXME find out what you don't know!
        # print(shape.OutlineCount())
        #print(shape.TotalVertices())
        #print(shape.FullPointCount())
        #shape.Format(False) gives a C/C++ source code representation !!
        #---------
        #SHAPE_LINE_CHAIN poly; 
        #auto tmp = SHAPE_LINE_CHAIN( { VECTOR2I( 215000000, 105500000), VECTOR2I( 246000000, 88000000), VECTOR2I( 260500000, 118500000)}, true );;
        #poly.AddOutline(tmp); } 
        #---------
        # now that, I understand ! let me parse that :-)
        shapeText = shape.Format(False)
        verts = []
        for s_pt in re.findall('VECTOR2I\\( [0-9]+, [0-9]+\\)',shapeText):
            coord_parts = s_pt.split(' ')[1:]
            x = int(coord_parts[0][:-1])
            y = int(coord_parts[1][:-1])
            verts.append([x,y]) # YES, and yikes!
        pprint(verts)

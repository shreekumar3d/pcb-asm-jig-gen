# Dependent packages
import pcbnew

# Local imports
import tripy
from jigcommon import *

# Standard packages
import math
from pprint import pprint

# This file contains functionality related to processing
# the 'Edge Cuts' layer of a KiCAD PCB file.
#
# Drawings on this layer define the PCB edges.
#
# KiCAD supports filled primitives (Rectangle, Circle and Polygon)
# as well as 2d segment shapes - Line and Arc.  It automatically
# stitches together these primitives to build the PCB shape. PCBs
# may have arbitrary shapes, with holes.  So you may have a round
# PCB with say a pacman shaped hole. Or a pacman shaped PCB with a
# pacman shaped hole, or two if you want one for each eye!
#
# Filled primitives may not intersect each other - that
# is treated as an error (easy to verify in 3D viewer, or in a
# DRC check)
#
# Lines and Arcs need to connect with each other precisely
# at their end points to create a valid filled area. Each loop
# defines a shape - which may be a hole.
#
# We can use filled shapes directly as is.  However, we need
# to process the segments to build filled shapes out of them.
#
# We find the filled shape with the # largest area, and treat
# that as the board edge.
#
# KiCAD doesn't disallow multiple non-overlapping board edges.
# (is this a bug or caught in DRC?)
# However, I don't know if people things this way at all - even
# panelized PCBs are still a single piece !
#
# Thus, we don't need to deal with this potential complexity :)
#
# Coordinate system note: In this file, we'll do all the ops on the
# edges in kicad coordinate system.
#

def load(board, pcb_segments, pcb_filled_shapes):
    shapes = [x for x in board.GetDrawings() if x.GetLayer()==pcbnew.Edge_Cuts]
    for d in shapes:
        if d.GetShapeStr() == 'Circle':
            ts = {
                'type' : d.GetShapeStr(),
                'center' : kcpt2pt(d.GetCenter()),
                'radius' : units_to_mm(d.GetRadius()),
            }
            pcb_filled_shapes.append(ts)
        elif d.GetShapeStr() == 'Rect':
            rect_points = []
            for corner in d.GetRectCorners():
                rect_points.append(kcpt2pt(corner))
                # a rectangle is a polygon!
            ts = {
                'type' : d.GetShapeStr(),
                'vertices' :  rect_points,
            }
            pcb_filled_shapes.append(ts)
        elif d.GetShapeStr() == 'Polygon':
            # FIXME: Find the proper KiCAD way to get the points
            # I didn't find anything, and deduced this rather
            # ugly hack by looking at the kicad C++ source code!
            shape = d.GetPolyShape()
            shapeText = shape.Format(False)
            poly_points = []
            for s_pt in re.findall('VECTOR2I\\( [0-9]+, [0-9]+\\)',shapeText):
                coord_parts = s_pt.split(' ')[1:]
                x = int(coord_parts[0][:-1])
                y = int(coord_parts[1][:-1])
                poly_points.append(kcpt2pt((x,y)))
            ts = {
                'type' : d.GetShapeStr(),
                'vertices' :  rect_points
            }
            pcb_filled_shapes.append(ts)
        elif d.GetShapeStr() == 'Arc':
            ts = {
                'type' : d.GetShapeStr(),
                'start' : kcpt2pt(d.GetStart()),
                'end' : kcpt2pt(d.GetEnd()),
                'mid' : kcpt2pt(d.GetArcMid()),
                'center' : kcpt2pt(d.GetCenter()),
                'radius' : units_to_mm(d.GetRadius()),
                'angle' : d.GetArcAngle().AsDegrees(),
                'angle_start' : d.GetArcAngleStart().AsDegrees(),
            }
            pcb_segments.append(ts)
        elif d.GetShapeStr() == 'Line':
            ts = {
                'type' : d.GetShapeStr(),
                'start' : kcpt2pt(d.GetStart()),
                'end' : kcpt2pt(d.GetEnd())
            }
            pcb_segments.append(ts)

def tess_iters(arc_resolution, r, degrees):
    return int(abs(((2*math.pi*r)/arc_resolution)/(360/degrees)))

def tesellate_circle(arc_resolution, seg):
    circle_angle = 2*math.pi # 2pi radians = 180 degree
    verts = []
    cx, cy = seg['center']
    r = seg['radius']
    iters = tess_iters(arc_resolution, r, 360)
    for i in range(iters):
        angle = (i/iters)*circle_angle
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        verts.append([x,y])
    return verts

def tesellate_arc(arc_resolution, seg):
    cx, cy = seg['center']
    r = seg['radius']
    sweep_angle = seg['angle']
    angle_start = seg['angle_start']
    iters = tess_iters(arc_resolution, r, sweep_angle)
    verts = []
    verts.append(seg['start'])
    for i in range(1, iters): # skip first
        angle = angle_start+((i/iters)*sweep_angle)
        angle = (angle*math.pi)/180.0
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        verts.append([x,y])
    verts.append(seg['end'])
    return verts

def is_close(pt1, pt2):
    dist = math.sqrt(math.pow((pt1[0]-pt2[0]),2)+pow((pt1[1]-pt2[1]),2))
    return (dist<0.01) # 0.01mm should be close enough?

def filled_shape(candidate_shape, pcb_segments):
    while True:
        extended = False
        for segment in pcb_segments:
            if is_close(segment['end'], candidate_shape['start']):
                # segment comes before, so prepend
                candidate_shape['segments'].insert(0, segment)
                # other end is the new starting point
                candidate_shape['start'] = segment['start']
            elif is_close(segment['start'], candidate_shape['end']):
                # segment comes after, so append
                candidate_shape['segments'].append(segment)
                # other end is the new ending point
                candidate_shape['end'] = segment['end']
            # segments can be in any order. While the following two
            # may look unusual, they must be considered!
            elif is_close(segment['start'], candidate_shape['start']):
                # other end is the new starting point
                candidate_shape['start'] = segment['end']
                # segment needs to be reversed
                reverse_segment(segment)
                # segment comes before, so prepend
                candidate_shape['segments'].insert(0, segment)
            elif is_close(segment['end'], candidate_shape['end']):
                # other end is the new ending point
                candidate_shape['end'] = segment['start']
                # segment needs to be reversed
                reverse_segment(segment)
                # segment comes after, so append
                candidate_shape['segments'].append(segment)
            else:
                continue
            pcb_segments.remove(segment) # remove this one
            extended = True
            #print('--- New candidate shape ---')
            #pprint(candidate_shape)
            break # Get out of the loop
        if len(pcb_segments)==0:
            #print('No more to check')
            break
        # Get out of the loop if the entire loop yields
        # no extensions!
        if not extended:
            break
    # do we have a closed loop !? That's success
    return is_close(candidate_shape['start'], candidate_shape['end'])

def reverse_segment(segment):
    # Reverse start and end
    segment['start'], segment['end'] = segment['end'], segment['start']
    if segment['type'] == 'Arc':
        # Arcs need to change angle and angle_start too
        angle = segment['angle']
        angle_start = segment['angle_start']
        segment['angle_start'] = angle + angle_start
        segment['angle'] = -angle

def coalesce_segments(pcb_segments, seg_shapes):
    # Coalesce segments
    while len(pcb_segments)>1:
        # Start with the first one
        candidate_shape = {
            'start' : pcb_segments[0]['start'],
            'end' : pcb_segments[0]['end'],
            'segments' : [ pcb_segments[0] ],
        }
        pcb_segments.pop(0)
        if filled_shape(candidate_shape, pcb_segments):
            seg_shapes.append(candidate_shape)
        else:
            return False

    # If we have unprocessed segments, that means something
    # is not connected anywhere => FAIL
    return not len(pcb_segments)>0

def tesellate(arc_resolution, seg_shapes, pcb_filled_shapes):
    # Tesllate filled shapes - circles
    for fs in pcb_filled_shapes:
        if fs['type'] == 'Circle':
            fs['vertices'] = tesellate_circle(arc_resolution, fs)

    # Convert combined shapes into filled shape(polygon)
    for shape in seg_shapes:
        poly_vertices = []
        for segment in shape['segments']:
            if segment['type'] == 'Arc':
                verts = tesellate_arc(arc_resolution, segment)
            else:
                verts = [segment['start'], segment['end']]
            # skip last one of current, it will be same as first
            # of next
            poly_vertices = poly_vertices[:-1] + verts
        fs = {
            'type' : 'Composite',
            'vertices' :  poly_vertices[:-1], # skip last one, it's same as first
            'shape': shape
        }
        pcb_filled_shapes.append(fs)

def compute_areas(pcb_filled_shapes):
    # Compute area of each of the filled shape
    for fs in pcb_filled_shapes:
        if fs['type'] == 'Circle':
            fs['area'] = math.pi * math.pow(fs['radius'],2)
        else:
            # It's a polygon, rectangle or composite
            tris = tripy.earclip(fs['vertices'])
            fs['area'] = tripy.calculate_total_area(tris)

def append_lip_lines_for_line(lip_lines, lip_size, min_gap, a,b):
    # classic distance formula
    len_ab = math.sqrt(math.pow(a[0]-b[0],2)+math.pow(a[1]-b[1],2))
    if lip_size  >= len_ab-min_gap:
        lip_lines.append([a,b])
    else:
        # compute a segment on either end
        frac = (lip_size*0.5)/len_ab
        dx = (b[0]-a[0])*frac
        dy = (b[1]-a[1])*frac

        # half lip length on either side
        ax1 = a[0]+dx
        ay1 = a[1]+dy
        lip_lines.append([a,[ax1,ay1]])
        bx1 = b[0]-dx
        by1 = b[1]-dy
        lip_lines.append([[bx1,by1],b])

def append_lip_lines_for_arc(
        arc_resolution, lip_lines, center, radius,
        start_angle, angle, start_pt, end_pt):
    temp_fs = {
        'type' : 'Arc',
        'center' : center,
        'radius' : radius,
        'start' : start_pt,
        'end'   : end_pt,
        'angle_start' : start_angle,
        'angle' : angle
    }
    points = tesellate_arc(arc_resolution, temp_fs)
    points = points + [points[0]]
    for i in range(len(points)-1):
        lip_lines.append([points[i],points[i+1]])

# compute_lips works this way -
# it basically iterates over every segment
# (line, arc) in the PCB shape.
#
# for lines, lips are created near the start and end
# points. lip_size is split in half towards each
# extremity. If the line is not much longer than
# lip_size, then the entire line is used as a lip
#
# for arcs, things get a bit more interesting.
# again, the general rule is that every end point must
# create half lip - so that is the start and end points.
# But the arc also has a mid point. Arcs can get pretty
# big, and we'd need to support that place as well.
# So, we consider 2*lip_size as the total lip that will
# be generated for an arc. If the arc is not much longer
# than that, we output the entire arc as a lip.
# Else, lip_size around the center, half on the start
# and end points.
#
# Finally, circle is 4 times lip, as we consider it as
# having 4 edges.
def compute_lips(arc_resolution, filled_shape, lip_size):
    min_gap = 5
    fs = filled_shape
    lip_lines = []
    if fs['type'] == 'Circle':
        peri_circle = 2*math.pi*fs['radius']
        # if the lips will be very small (or none)
        # return the full circle
        if (lip_size*4)  >= peri_circle-(4*min_gap):
            points = tesellate_circle(arc_resolution, fs)
            points = points + [points[0]]
            for i in range(len(points)-1):
                lip_lines.append([points[i],points[i+1]])
        else:
            # what fraction in angle terms is a single lip
            angle = ((lip_size)/peri_circle)*360
            start_angle = -angle/2
            end_angle = angle/2
            cx, cy = fs['center']
            r = fs['radius']
            angle_inc = 90
            for i in range(4):
                arc_start_x = cx + r*math.cos(start_angle*math.pi/180)
                arc_start_y = cx + r*math.cos(start_angle*math.pi/180)
                arc_end_x = cx + r*math.cos(end_angle*math.pi/180)
                arc_end_y = cx + r*math.cos(end_angle*math.pi/180)
                append_lip_lines_for_arc(
                    arc_resolution, lip_lines, fs['center'], r,
                    start_angle, angle,
                    [arc_start_x, arc_start_y],
                    [arc_end_x, arc_end_y])
                start_angle = start_angle+angle_inc
                end_angle = end_angle+angle_inc
    elif fs['type'] in ['Rect', 'Polygon']:
        # add first one to end to make loop easier
        verts = fs['vertices'] + [fs['vertices'][0]]
        pprint(verts)
        for i in range(len(verts)-1):
            a = verts[i]
            b = verts[i+1]
            append_lip_lines_for_line(lip_lines, lip_size, min_gap, a,b)
            # classic distance formula
            len_ab = math.sqrt(math.pow(a[0]-b[0],2)+math.pow(a[1]-b[1],2))
            if lip_size  >= len_ab-min_gap:
                lip_lines.append([a,b])
            else:
                # compute a segment on either end
                frac = (lip_size*0.5)/len_ab
                dx = (b[0]-a[0])*frac
                dy = (b[1]-a[1])*frac

                # half lip length on either side
                ax1 = a[0]+dx
                ay1 = a[1]+dy
                lip_lines.append([a,[ax1,ay1]])
                bx1 = b[0]-dx
                by1 = b[1]-dy
                lip_lines.append([[bx1,by1],b])
    elif fs['type'] == 'Composite':
        for seg in fs['shape']['segments']:
            if seg['type'] == 'Line':
                append_lip_lines_for_line(lip_lines, lip_size, min_gap, seg['start'],seg['end'])
            else:
                # Arc
                # arc length
                peri_arc = 2*math.pi*seg['radius']*seg['angle']/360
                #print(peri_arc)
                # again, if only small gaps shall result, then just push out the entire arc
                if lip_size  >= abs(2*peri_arc)-2*min_gap:
                    append_lip_lines_for_arc(
                        arc_resolution, lip_lines, seg['center'], seg['radius'],
                        seg['angle_start'], seg['angle'],
                        seg['start'], seg['end'])
                else:
                    #print('arc start_angle=', seg['angle_start'], ' angle=', seg['angle'])
                    # start and end 50-50
                    theta = (lip_size/(2*abs(peri_arc)))*seg['angle']
                    seg_end_angle = seg['angle_start']+theta
                    #print('sub seg start:',theta, ' start=', seg['angle_start'], ' angle =', theta)
                    pt_x2 = seg['center'][0] + seg['radius']*math.cos(seg_end_angle*math.pi/180)
                    pt_y2 = seg['center'][1] + seg['radius']*math.sin(seg_end_angle*math.pi/180)
                    append_lip_lines_for_arc(
                        arc_resolution, lip_lines, seg['center'], seg['radius'],
                        seg['angle_start'], theta,
                        seg['start'], [pt_x2, pt_y2])
                    seg_start_angle = seg['angle_start']+seg['angle']-theta
                    #print('sub seg end :',theta, ' start=', seg_start_angle, ' angle = ', theta)
                    pt_x1 = seg['center'][0] + seg['radius']*math.cos(seg_start_angle*math.pi/180)
                    pt_y1 = seg['center'][1] + seg['radius']*math.sin(seg_start_angle*math.pi/180)
                    append_lip_lines_for_arc(
                        arc_resolution, lip_lines, seg['center'], seg['radius'],
                        seg_start_angle, theta,
                        [pt_x1, pt_y1], seg['end'])
                    seg_start_angle = seg['angle_start']+(seg['angle']/2)-theta
                    seg_end_angle = seg_start_angle + 2*theta
                    #print('sub seg mid :', ' start=', seg_start_angle, ' angle = ', theta)
                    pt_x1 = seg['center'][0] + seg['radius']*math.cos(seg_start_angle*math.pi/180)
                    pt_y1 = seg['center'][1] + seg['radius']*math.sin(seg_start_angle*math.pi/180)
                    pt_x2 = seg['center'][0] + seg['radius']*math.cos(seg_end_angle*math.pi/180)
                    pt_y2 = seg['center'][1] + seg['radius']*math.sin(seg_end_angle*math.pi/180)
                    append_lip_lines_for_arc(
                        arc_resolution, lip_lines, seg['center'], seg['radius'],
                        seg_start_angle, 2*theta,
                        [pt_x1, pt_y1], [pt_x2, pt_y2])
        #raise RuntimeError('Composite Unsupported')
    return lip_lines

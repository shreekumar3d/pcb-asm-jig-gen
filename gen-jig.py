#!/usr/bin/env python3

# FIXME: right now coded to work with thejas32-testbed.
# YMMV for other boards !

#
# Usage notes:
#
# Before running the script,
# 1. Export environment variables that are used by KiCAD
#    e.g. export KICAD8_3DMODEL_DIR=/usr/share/kicad/3dmodels
# 2. Ensure runtime dependencies are met.  Meshlab and
#    KiCAD are used for file format conversion at this time.
#    Without them, the script will fail
#

# These are all modules you need to have
import tinyobjloader
import pcbnew
import scipy.spatial
from scipy.spatial.transform import Rotation

# Standard imports
import tempfile
import argparse
import subprocess
import os
import functools
from pprint import pprint
import numpy as np

mesh_cache = {}

pcb_thickness = 1.6 # FIXME: get this from KiCAD
# shells are generated to hold components. These don't
# stretch all the way till the board. They fall short
# by this 'clearance' value.  This serves two purposes:
#   1. You can see from the sides that the components
#      are sitting flush with the PCB before you solder
#      them
#   2. You may have SMD components on the boards placed
#      very close to the through-hole stuff. This is an
#      indirect strategy to avoid them.
#      FIXME: implement a direct solution to ensure this
#      never happens. On AtiVEGA, the USB power connector
#      also intersects the shells. So this is not just an
#      SMD thing. Potential idea - perhaps add all the
#      "perimeter solids" first, then subtract the
#      "outline solids", then subtract an extended
#      bounding box of each other individual component not
#      considered for the shells.
pcb_clearance = 1 # a 1206 SMD resistor is 0.55mm. Tune further if required

base_thickness = 1

protrude = 1 # shells will come above PCB by this much, so user can enable and see

pcb_perimeter_height = 2 # PCB perimeter height, beyond the base

# AtiVEGA has no mounting hole on one corner.
# Support enforcers ensure that a board outline is
# included close to these places
forced_pcb_supports = [
  [ 121.1, -152.0] # non-existent mounting hole location near to Pico
]

# Selectively process these component references
ref_filter_list = [] # ['U1', 'J2']

def units_to_mm(x):
    return x/1000000

# returns flat array of xyz coordinates
def load_obj_mesh_verts(filename, scale=1.0):
    reader = tinyobjloader.ObjReader()
    ret = reader.ParseFromFile(filename)
    if ret == False:
        raise RuntimeError("Unable to load OBJ file %s"%(filename))
    attrib = reader.GetAttrib()
    nverts = len(attrib.vertices)//3 # attrib.vertices is contiguous x,y,z
    mesh = np.array(attrib.vertices)
    mesh *= scale
    mesh.resize((nverts,3)) # in place change in dims
    return mesh

def load_mesh(filename):
    global mesh_cache
    retval = None
    if filename in mesh_cache:
        print('Returning %s from cache'%(filename))
        return mesh_cache[filename]

    if filename.endswith('.obj'):
        retval = load_obj_mesh_verts(filename)
    elif filename.endswith('.step') or filename.endswith('.stp'):
        with tempfile.NamedTemporaryFile(suffix='.obj') as fp:
            print('Converting STEP file %s to OBJ file %s'%(filename, fp.name))
            retcode = subprocess.call([
                'freecad.cmd', 'stp2obj.py', filename, fp.name ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            if retcode != 0:
                raise RuntimeError("Unable to convert STEP file %s to obj"%(filename))
            retval = load_obj_mesh_verts(fp.name)
    elif filename.endswith('.wrl') or filename.endswith('.vrml'):
        with tempfile.NamedTemporaryFile(suffix='.obj') as fp:
            print('Converting mesh file %s to OBJ file %s'%(filename, fp.name))
            retcode = subprocess.call([
                'meshlabserver', '-i', filename, '-o', fp.name ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            if retcode != 0:
                raise RuntimeError("Unable to convert file %s to obj"%(filename))
            retval = load_obj_mesh_verts(fp.name, scale=2.54) # VRML files are in 1/2.54 mm units
    else:
        raise RuntimeError("No converter to load %s"%(filename))

    mesh_cache[filename] = retval
    return retval

mounting_holes = forced_pcb_supports
def get_th_info(board):
    global mounting_holes
    fp_list = board.Footprints()
    th_info = []
    for fp in fp_list:
        fp_x = units_to_mm(fp.GetX())
        fp_y = -units_to_mm(fp.GetY())
        print(fp.GetReference())
        print('  Position(mm):', fp_x, fp_y)
        print('  On side     :', fp.GetSide())
        print('  Orientation :', fp.GetOrientation().AsDegrees())
        print('  DNP ?       :', fp.IsDNP())
        print('  TH ?        :', fp.HasThroughHolePads())
        if fp.HasThroughHolePads():
            thc_models = []
            for mod3d in fp.Models():
                # NOTE XXX don't hold references to internal vectors - they can get
                # messed up! Make copies!
                thc_models.append({
                        'model'    : mod3d.m_Filename,
                        'offset'   : [mod3d.m_Offset[0], mod3d.m_Offset[1], mod3d.m_Offset[2]],
                        'scale'    : [mod3d.m_Scale[0], mod3d.m_Scale[1], mod3d.m_Scale[2]],
                        'rotation' : [mod3d.m_Rotation[0], mod3d.m_Rotation[1], mod3d.m_Rotation[2]]
                })
            if len(thc_models)>0:
                th_info.append({
                    'ref': fp.GetReference(),
                    'x' : fp_x,
                    'y' : fp_y,
                    'orientation' : fp.GetOrientation().AsDegrees(),
                    'side' : fp.GetSide(),
                    'position' : fp.GetPosition(),
                    'models' : thc_models,
                    'kfp' : fp}) # Reference if we need more info later
            for fn in fp.Fields():
                #print(fn)
                #print(dir(fn))
                #print(fn.GetName(), fn.GetText(), fn.GetShownText(True), fn.GetHyperlink(), fn.GetCanonicalName(), fn.GetFriendlyName(), fn.GetParentFootprint(), fn.GetParentAsString())
                if fn.GetText().startswith('MountingHole'):
                    print('  --> Is a Mounting Hole')
                    mounting_holes.append(
                        [fp_x, fp_y]
                    )
                    break
        #print(fp.Footprint().GetName())
        #pprint(dir(fp.Footprint()))
    return th_info

parser = argparse.ArgumentParser()
parser.add_argument("kicad_pcb")
args = parser.parse_args()
board = pcbnew.LoadBoard(args.kicad_pcb)
th_info = get_th_info(board)

# test if you can load all models
os.environ["KIPRJMOD"] = os.path.split(args.kicad_pcb)[0]
for comp in th_info:
    # We're guaranteed to have at-least one 3d model
    for modinfo in comp['models']:
        model_filename = os.path.expandvars(modinfo['model'])
        modinfo['mesh'] = load_mesh(model_filename)
#pprint(th_info)

# Footprint useful things
# fp.GetCenter()
# fp.GetX(), fp.GetY() - same as center?
# fp.GetSide() - 0 for top, 31 for bottom
# fp.GetOrientation() - 0, 90, ...

def ref2outline(ref):
    return 'ref_%s'%(ref)
def ref2shell(ref):
    return 'shell_%s'%(ref)
def ref2pocket(ref):
    return 'pocket_%s'%(ref)
def ref2peri(ref):
    return 'peri_%s'%(ref)

mod_lines = []
geom_lines = []
def gen_shell_shape(ref, x, y, rot, min_z, max_z, verts):
    global mod_lines, geom_lines

    # first define the polygon so that we can do offset on it
    mod_name = ref2outline(ref)
    mod_lines.append('// Outline for %s\n'%(ref))
    mod_lines.append('module %s() {\n'%(mod_name))
    mod_lines.append('  polygon(\n')
    mod_lines.append('    points=[\n')

    for v in verts:
        mod_lines.append('      [%f,%f],\n'%(v[0],v[1]))
    mod_lines.append('    ]\n')
    mod_lines.append('  );\n')
    mod_lines.append('}\n')
    #
    # note: max_z is positive - so this lifts
    #if rot==90 or rot==-90: # Invert 90 and -90
    #    rot=-rot
    shell_name = ref2shell(ref)
    geom_lines.append('// Shell for %s\n'%(ref))
    geom_lines.append('module %s() {\n'%(shell_name))
    geom_lines.append('  translate([%f,%f,pcb_clearance])\n'%(x,y))
    #geom_lines.append('    rotate([0,0,%f])\n'%(rot))
    # note linear extrude is only "max_z". Assumes top
    # side of the board. "min_z" is the extent from the top
    # side of the board all the way to the end of the pin
    # on the other side of the bottom.
    geom_lines.append('      linear_extrude(%f-pcb_clearance)\n'%(max_z))
    geom_lines.append('        difference() {\n')
    geom_lines.append('          offset(r=clearance+wall_thickness)\n')
    geom_lines.append('            %s();\n'%(mod_name))
    geom_lines.append('          offset(clearance)\n')
    geom_lines.append('            %s();\n'%(mod_name))
    geom_lines.append('        }\n')
    geom_lines.append('}\n')

    shell_name = ref2pocket(ref)
    geom_lines.append('// Pocket for %s\n'%(ref))
    geom_lines.append('module %s() {\n'%(shell_name))
    geom_lines.append('  translate([%f,%f,-protrude])\n'%(x,y))
    #geom_lines.append('    rotate([0,0,%f])\n'%(rot))
    geom_lines.append('      linear_extrude(protrude+pcb_thickness+%s)\n'%(max_z))
    geom_lines.append('        offset(clearance)\n')
    geom_lines.append('          %s();\n'%(mod_name))
    geom_lines.append('}\n')

    peri_name = ref2peri(ref)
    mod_lines.append('// Perimeter for %s\n'%(ref))
    mod_lines.append('module %s() {\n'%(peri_name))
    mod_lines.append('  translate([%f,%f,pcb_thickness+%f])\n'%(x,y, max_z))
    #mod_lines.append('    rotate([0,0,%f])\n'%(rot))
    mod_lines.append('      offset(r=clearance+wall_thickness)\n')
    mod_lines.append('        %s();\n'%(mod_name))
    mod_lines.append('}\n')

output_fname = 'test.scad'
print('Creating output in %s...\n')
fp_scad = open(output_fname, 'w')
# We use OpenSCAD to do the grunt work of building the
# actual model for us. Tesellation level must be set
# to match or exceed the limits of 3D printing.
# 3D printing generally works on the order of 0.1mm
# so we choose a value half of that, 0.05 mm
# This should give a decent balance of smooth shapes
# and file sizes, and processing needs.
fp_scad.write('$fs = 0.05;\n')
fp_scad.write('clearance = 0.1;\n');
fp_scad.write('wall_thickness = 1.2;\n');
fp_scad.write('pcb_thickness=%s;\n'%(pcb_thickness))
fp_scad.write('pcb_clearance=%s;\n'%(pcb_clearance))
fp_scad.write('protrude=%s;\n'%(protrude))
fp_scad.write('base_thickness = %s;\n'%(base_thickness))

# Process the PCB edge
pcb_edge_points = []
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
    elif d.GetShapeStr() == 'Line':
        pt = d.GetStart()
        pcb_edge_points.append([units_to_mm(pt[0]), -units_to_mm(pt[1])])
        pt = d.GetEnd()
        pcb_edge_points.append([units_to_mm(pt[0]), -units_to_mm(pt[1])])
    elif d.GetShapeStr() == 'Rect':
        for corner in d.GetRectCorners():
            pcb_edge_points.append([units_to_mm(corner[0]), -units_to_mm(corner[1])])
    elif d.GetShapeStr() == 'Polygon':
        shape = d.GetPolyShape()
        shapeText = shape.Format(False)
        for s_pt in re.findall('VECTOR2I\\( [0-9]+, [0-9]+\\)',shapeText):
            coord_parts = s_pt.split(' ')[1:]
            x = units_to_mm(int(coord_parts[0][:-1]))
            y = -units_to_mm(int(coord_parts[1][:-1]))
            pcb_edge_points.append([x,y])

# Write out the edge
pcb_edge_points = np.array(pcb_edge_points)
hull = scipy.spatial.ConvexHull(pcb_edge_points)
pcb_edge_hull = pcb_edge_points[hull.vertices]
fp_scad.write('module pcb_edge() {\n')
fp_scad.write('  polygon(\n')
fp_scad.write('    points=[\n')
for v in pcb_edge_hull:
    fp_scad.write('      [%f,%f],\n'%(v[0],v[1]))
fp_scad.write('    ]\n')
fp_scad.write('  );\n')
fp_scad.write('}\n')

all_shells = []
fp_centers = []
topmost_z = 0
# For each TH component on the board
for th in th_info:
    #if th['ref'] in ['U1','U4','U5']:
    #    print('Skipping known component %s to work around issues'%(th['ref']))
    #    continue
    if len(ref_filter_list)>0 and th['ref'] not in ref_filter_list:
        #print('Skipping ', th['ref'])
        continue
    print('Processing ', th['ref'])
    # each footprint can have multiple models.
    # each model that is "in contact" with the board will generate
    # a shell
    for idx, modinfo in enumerate(th['models']):
        mesh = modinfo['mesh']
        if mesh.shape[0]==0:
            print('  WARNING: Mesh %s is empty on import. Watch out for intended side effects. SKIPPING!'
                  %(modinfo['model']))
            continue
        #print(modinfo['model'])
        #print(modinfo['offset'])
        #print(modinfo['scale'])
        #print(modinfo['rotation'])
        mesh_scale = mesh * modinfo['scale']
        # FIXME: Hrrmph! why should I need to reverse these
        # angles !?
        rx = Rotation.from_euler('x', -modinfo['rotation'][0], degrees=True)
        ry = Rotation.from_euler('y', -modinfo['rotation'][1], degrees=True)
        rz = Rotation.from_euler('z', -modinfo['rotation'][2], degrees=True)
        rot_angle=th['orientation'];
        rz2 = Rotation.from_euler('z', rot_angle, degrees=True)
        r_xyz = rx*ry*rz
        mesh_rotated = r_xyz.apply(mesh_scale)
        mesh2 = mesh_rotated + [modinfo['offset'][0], modinfo['offset'][1], modinfo['offset'][2]]
        mesh = rz2.apply(mesh2)
        min_z = min(mesh[:,2])
        max_z = max(mesh[:,2])
        #print('min_z = ', min_z, ' max_z = ', max_z)
        if min_z>0:
            print('  Mesh %s is NOT mounted on board. Skipping.'%(modinfo['model']))
        else:
            min_x = min(mesh[:,0])
            max_x = max(mesh[:,0])
            min_y = min(mesh[:,1])
            max_y = max(mesh[:,1])
            center_x = ((min_x + max_x)/2)
            center_y = ((min_y + max_y)/2)
            fp_centers.append([center_x+th['x'],center_y+th['y']])
            #for v in combined_xy:
            #    print(v)
            # FIXME: how is ==0 possible?
            mesh_xy = mesh[:,0:2] # get rid of Z coordinates for hull calc
            hull = scipy.spatial.ConvexHull(mesh_xy)
            hull_verts = mesh_xy[hull.vertices]
            #print('Hull size = ', len(hull.vertices), ' min Z=', min_z, ' max Z=', max_z)
            #print(hull_verts)
            shell_ident = '%s_%d'%(th['ref'],idx)
            gen_shell_shape(shell_ident,
                          th['x'], th['y'], th['orientation'],
                          min_z, max_z, hull_verts)
            all_shells.append({
                'name':shell_ident,
                'min_z':min_z,
                'max_z':max_z})
            print('  Generating shell %s for mesh %s'%(shell_ident, modinfo['model']))
            topmost_z = max(topmost_z, max_z)

fp_scad.write(''.join(mod_lines))
fp_scad.write(''.join(geom_lines))
# This module will include all shells
fp_scad.write('''
module mounted_component_shells() {
  translate([0,0,pcb_thickness])
    union() {
''')
for shell_ident in all_shells:
    fp_scad.write('      %s();\n'%(ref2shell(shell_ident['name'])))
fp_scad.write('''
    }
}
''')
# This module will include all pockets
fp_scad.write('module mounted_component_pockets() {\n')
fp_scad.write('  union() { \n')
for shell_ident in all_shells:
    fp_scad.write('    %s();\n'%(ref2pocket(shell_ident['name'])))
fp_scad.write('  }\n')
fp_scad.write('}\n')
fp_scad.write('\n')

fp_scad.write('''
module wide_line(start, end) {
    hull() {
        translate(start) cylinder(base_thickness);
        translate(end) cylinder(base_thickness);
    }
}\n\n''')

# Compute bounding box of PCB
# FIXME: make this min, max code less verbose
pcb_min_x = pcb_max_x = pcb_edge_points[0][0]
pcb_min_y = pcb_max_y = pcb_edge_points[0][1]
for pt in pcb_edge_points:
    pcb_min_x = min(pcb_min_x, pt[0])
    pcb_max_x = max(pcb_max_x, pt[0])
    pcb_min_y = min(pcb_min_y, pt[1])
    pcb_max_y = max(pcb_max_y, pt[1])

# Delaunay triangulation will be done on the following points
# 1. centers of all TH footprints
# 2. mounting holes
# 3. bounding box corners of PCB edge. mounting holes are
#    inside the PCB and don't extend all the way to the edge.
#    If we don't include them, we may end up having a separate
#    "delaunay island", depending on the exact PCB.
dt_centers = fp_centers + mounting_holes + [
    [pcb_min_x , pcb_min_y],
    [pcb_min_x , pcb_max_y],
    [pcb_max_x , pcb_min_y],
    [pcb_max_x , pcb_max_y]
]

fp_scad.write('''
pcb_gap=0.3;
pcb_overlap=0.3;
pcb_perimeter=1.6;
topmost_z=%s;
pcb_perimeter_height = %s; // from base
'''%(topmost_z, pcb_perimeter_height))

fp_scad.write('module base_structure() {\n')
fp_scad.write('  translate([0,0,pcb_thickness+topmost_z]) {\n')
fp_scad.write('    union() {\n')
fp_scad.write('      intersection() { \n')
fp_scad.write('        union(){ \n')
if len(dt_centers)>=4:
    d_verts = np.array(dt_centers)
    d_tris = scipy.spatial.Delaunay(d_verts)
    for tri in d_tris.simplices:
        # tri is a,b,c
        av = d_verts[tri[0]]
        a = '[%s,%s]'%(av[0],av[1])
        bv = d_verts[tri[1]]
        b = '[%s,%s]'%(bv[0],bv[1])
        cv = d_verts[tri[2]]
        c = '[%s,%s]'%(cv[0],cv[1])
        fp_scad.write('          wide_line(%s,%s);\n'%(a,b))
        fp_scad.write('          wide_line(%s,%s);\n'%(b,c))
        fp_scad.write('          wide_line(%s,%s);\n'%(c,a))
else:
    pprint(dt_centers)
    for vert in dt_centers:
        pt = '[%s,%s]'%(vert[0],vert[1])
        fp_scad.write('        translate(%s) sphere(base_thickness);\n'%(pt))
fp_scad.write('        }\n')
fp_scad.write('''
        // ensure no peri lines go out of frame
        // tall shell supports must be included though
        linear_extrude(base_thickness)
          offset(r=pcb_perimeter+pcb_gap)
            pcb_edge();
      }
''')

fp_scad.write('      union() { \n')
for shell_ident in all_shells:
    # if this component is less taller than the tallest component
    # then the extra must be filled up
    extra_extrude = topmost_z - shell_ident['max_z']
    if extra_extrude>0:
        fp_scad.write('        translate([0,0,-%f])\n'%(extra_extrude))
        fp_scad.write('          linear_extrude(base_thickness+%f)\n'%(extra_extrude))
        fp_scad.write('            %s();\n'% (ref2peri(shell_ident['name'])))
    else:
        fp_scad.write('        linear_extrude(base_thickness)\n')
        fp_scad.write('          %s();\n'% (ref2peri(shell_ident['name'])))
fp_scad.write('      }\n')
fp_scad.write('    }\n')
fp_scad.write('  }\n')
fp_scad.write('}\n')

fp_scad.write('''
module pcb_holder() {
  // solid between perimeter & outline
  // full height
  linear_extrude(topmost_z+pcb_thickness+base_thickness) {
    difference() {
      offset(r=pcb_perimeter+pcb_gap) pcb_edge();
      offset(r=pcb_gap) pcb_edge();
    }
  }

  // solid between outline & overlap
  // PCB sits here, so that much lower height than
  // the full height
  translate([0,0,pcb_thickness]) {
    linear_extrude(topmost_z+base_thickness) {
      difference() {
        offset(r=pcb_gap) pcb_edge();
        offset(r=-pcb_overlap) pcb_edge();
      }
    }
  }

}

module pcb_perimeter_short() {
  translate([0,0,pcb_thickness+topmost_z-pcb_perimeter_height]) {
    linear_extrude(pcb_perimeter_height+base_thickness) {
      difference() {
        offset(r=pcb_perimeter+pcb_gap) pcb_edge();
        offset(r=-pcb_overlap) pcb_edge();
      }
    }
  }
}
''')

fp_scad.write('corner_support_size=15;\n')
fp_scad.write('corner_support_height=60;\n') # FIXME: this is laziness! compute it!
fp_scad.write('module pcb_corner_support() {\n')
fp_scad.write('  translate([0,0,0])\n')
fp_scad.write('  union() {\n')
for pt in mounting_holes:
    fp_scad.write('    translate([%s,%s,0])\n'%(pt[0],pt[1]))
    fp_scad.write('      cube([corner_support_size, corner_support_size,  corner_support_height],center = true);\n')
fp_scad.write('  }\n')
fp_scad.write('}\n')

fp_scad.write('''
difference() {
  union() {
    intersection() {
      pcb_corner_support();
      pcb_holder();
    };
    pcb_perimeter_short();
    base_structure();
    mounted_component_shells();
  }
  mounted_component_pockets();
}
''')

fp_scad.write('''
module stackup() {
  pcb_min_x = %s;
  pcb_max_x = %s;
  sq_size = 5;
  sq_gap = sq_size*1.2;
  sq_x = pcb_min_x - sq_gap;

  color("green")
    translate([sq_x-sq_gap,0,0])
      linear_extrude(pcb_thickness)
        square(sq_size);

  color("blue")
    translate([sq_x-sq_gap,0,pcb_thickness])
      linear_extrude(pcb_clearance)
        square(sq_size);

  color("red")
    translate([sq_x-sq_gap,0,pcb_thickness+pcb_clearance])
      linear_extrude(topmost_z-pcb_clearance)
        square(sq_size);

  color("white")
    translate([sq_x-sq_gap,0,pcb_thickness+topmost_z])
      linear_extrude(base_thickness)
        square(sq_size);
}

// Include the next line to visualize the "stack-up"
//stackup();
'''%(
    pcb_min_x,
    pcb_max_x)
)
#
#
# Coordinate system notes:
#
# We adhere to the normal 3D coordinate system in this program,
# with Z pointing up.
#
# KiCAD uses a coordinate system where X increases to the right,
# and Y increases down - like a regular framebuffer. Therefore,
# Y coordinates from KiCAD needs to be negated to map to the
# regular 3D system. Note that 3D meshes in KiCAD use the regular
# 3D coordinate system, so those coordinates don't need to be
# transformed.
#
# This script also uses OpenSCAD. The extrude operation extrudes
# along the positive Z axis - i.e. upwards.
#
# Z = 0 corresponds to the bottom of the PCB. At the top of the
# PCB, Z = PCB thickness
#
# This setup is so that we can exactly match KiCAD's step file
# export of the board. Overlaying the mesh generated on this
# program on top of the step file is useful both for debugging
# as well as understanding any issues.
#
# Here is the Z coordinate stackup, for top side assembly.
# Soldering is on the bottom side.
#
# Z = topmost + 1 | "base". Start of 1 mm thick layer, delaunay triangles
#     +thickness  |
#
# Z = topmost     | topmost point of tallest component, when mounted on
#     +thickness | the PCB. Typically the long end of a berg header
#
# Z = in-between  | Highest point of intermediate height component
#
# Z = thickness+  | Start of typical shell
#     clearance   | Clearance allows user to visually verify component
#                 | placement and fit from the sides
#
# Z = thickness   | PCB top
#
# Z = 0           | PCB botto
#
# This program uses this terminology
#
# edge      => closed polygon matching exact border.
#              e.g. the outer edge of the PCB. In the case of a component
#              you can think of a projection of the 3d model of
#              the component onto the Z plane, and the resulting outline.
#              outlines can be concave.
#
# hull      => convex hull of the edge. Typically used as these are
#              easier to compute and have useful properties. Using
#              concave surfaces needs more care, else things may break.
#
# overlap   => small inset of the edge/hull.
#              offset value here is called "overlap" as well.
#
# outline    => small offset of the edge/hull (meaning outwards)
#               offset value here is called "gap"
#
# perimeter => large offset of the edge/hull. Typically used to
#              build "walls" or shells
#              offset value here is termed "thickness"
#
# component/board can slide inside outline, but will abut above an
# overlap. The term "clearance" is used to a gap in the
# Z direction.
#
# "pocket" is a negative shape - e.g. the hollow cavity required
# to push in a component
#
# "shell" is a solid shape with a cavity. the cavity is a "pocket" so
# that the component can be held in the pocket.
#
#

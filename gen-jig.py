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
import argparse
import os
import functools
from pprint import pprint
import numpy as np
import sys
import tomllib
import json

# Local imports
from jigcommon import *
import edge_cuts
import mesh_ops

shell_protrude = 1 # shells will come above PCB by this much, so user can enable and see

def default_cfg():
    return '''
[pcb]
thickness = 1.6
tesellate_edge_cuts_curve = 0.1

[pcb_holder]
gap = 0.3
overlap = 0.3
perimeter = 1.6
base_perimeter_height = 2
forced_lips = [ ]
lip_dimensions = 15

[component_holder]
shell_clearance = 1
shell_gap = 0.1
shell_thickness = 1.2

[refs]
do_not_process = []
process_only_these = []

[jig_options]
style = "soldering_helper"
base_thickness = 1
base_style = "mesh"
'''

def get_th_info(board, mounting_holes, ref_process_only_these, ref_do_not_process):
    fp_list = board.Footprints()
    th_info = []
    for fp in fp_list:
        ref = fp.GetReference()
        if len(ref_process_only_these)>0:
            # process_only_these takes precedence
            if ref not in ref_process_only_these:
                continue
        elif len(ref_do_not_process)>0 and ref not in ref_do_not_process:
            # exclusion is enforced if process_only_these isn't specified
            continue
        fp_x = units_to_mm(fp.GetX())
        fp_y = -units_to_mm(fp.GetY())
        #print(fp.GetReference())
        #print('  Position(mm):', fp_x, fp_y)
        #print('  On side     :', fp.GetSide())
        #print('  Orientation :', fp.GetOrientation().AsDegrees())
        #print('  DNP ?       :', fp.IsDNP())
        #print('  TH ?        :', fp.HasThroughHolePads())
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
                    #print('  --> Is a Mounting Hole')
                    mounting_holes.append(
                        [fp_x, fp_y]
                    )
                    break
        #print(fp.Footprint().GetName())
        #pprint(dir(fp.Footprint()))
    return th_info

# generate module names used in oscad
def ref2outline(ref):
    return 'ref_%s'%(ref)
def ref2shell(ref):
    return 'shell_%s'%(ref)
def ref2pocket(ref):
    return 'pocket_%s'%(ref)
def ref2peri(ref):
    return 'peri_%s'%(ref)

def gen_shell_shape(ref, x, y, rot, min_z, max_z, verts, mod_lines, geom_lines):
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
    geom_lines.append('  translate([%f,%f,shell_clearance])\n'%(x,y))
    #geom_lines.append('    rotate([0,0,%f])\n'%(rot))
    # note linear extrude is only "max_z". Assumes top
    # side of the board. "min_z" is the extent from the top
    # side of the board all the way to the end of the pin
    # on the other side of the bottom.
    geom_lines.append('      linear_extrude(max_z_%s-shell_clearance)\n'%(ref))
    geom_lines.append('        difference() {\n')
    geom_lines.append('          offset(r=shell_gap+shell_thickness)\n')
    geom_lines.append('            %s();\n'%(mod_name))
    geom_lines.append('          offset(shell_gap)\n')
    geom_lines.append('            %s();\n'%(mod_name))
    geom_lines.append('        }\n')
    geom_lines.append('}\n')

    shell_name = ref2pocket(ref)
    geom_lines.append('// Pocket for %s\n'%(ref))
    geom_lines.append('module %s() {\n'%(shell_name))
    geom_lines.append('  translate([%f,%f,-shell_protrude])\n'%(x,y))
    #geom_lines.append('    rotate([0,0,%f])\n'%(rot))
    geom_lines.append('      linear_extrude(shell_protrude+pcb_thickness+max_z_%s)\n'%(ref))
    geom_lines.append('        offset(shell_gap)\n')
    geom_lines.append('          %s();\n'%(mod_name))
    geom_lines.append('}\n')

    peri_name = ref2peri(ref)
    mod_lines.append('// Perimeter for %s\n'%(ref))
    mod_lines.append('module %s() {\n'%(peri_name))
    mod_lines.append('  translate([%f,%f,pcb_thickness+max_z_%s])\n'%(x,y, ref))
    #mod_lines.append('    rotate([0,0,%f])\n'%(rot))
    mod_lines.append('      offset(r=shell_gap+shell_thickness)\n')
    mod_lines.append('        %s();\n'%(mod_name))
    mod_lines.append('}\n')

#
# Execution starts here
#

parser = argparse.ArgumentParser()
parser.add_argument("--config", help='Use specified configuration options file')
parser.add_argument("kicad_pcb", help='KiCAD PCB file (.kicad_pcb) to process')
args = parser.parse_args()

if args.config:
    config_text = open(args.config, 'r').read()
    cfg = tomllib.load(open(args.config,'rb'))
    #print(json.dumps(cfg, indent=2))
else:
    config_text = 'Internal Defaults'
    cfg = tomllib.loads(default_cfg())

pcb_thickness = cfg['pcb']['thickness']
shell_clearance = cfg['component_holder']['shell_clearance']
shell_gap = cfg['component_holder']['shell_gap']
shell_thickness = cfg['component_holder']['shell_thickness']
base_is_solid = 0 if cfg['jig_options']['base_style']=="mesh" else 1

base_thickness = cfg['jig_options']['base_thickness']
pcb_perimeter_height = cfg['pcb_holder']['base_perimeter_height']
pcb_holder_gap = cfg['pcb_holder']['gap']
pcb_holder_overlap = cfg['pcb_holder']['overlap']
pcb_holder_perimeter = cfg['pcb_holder']['perimeter']
forced_pcb_supports = cfg['pcb_holder']['forced_lips']
ref_do_not_process = cfg['refs']['do_not_process']
ref_process_only_these = cfg['refs']['process_only_these']
mounting_hole_support_size = cfg['pcb_holder']['lip_dimensions']
arc_resolution = cfg['pcb']['tesellate_edge_cuts_curve']
jig_style = cfg['jig_options']['style']
jig_style_range = ['soldering_helper', 'component_shell']
if jig_style not in jig_style_range:
    print('BAD value "%s" for jig_options/style. Valid values are : ' %
        (jig_style,','.join(jig_style_range)))
    sys.exit(-1)
jig_style_soldering_helper = (jig_style == 'soldering_helper')
jig_style_component_shell = (jig_style == 'component_shell')

if jig_style_component_shell:
    if shell_clearance>0:
        print('INFO: Generating component shells, shell_clearance=%s will be ignored.'
            %(shell_clearance))
        shell_clearance = 0

board = pcbnew.LoadBoard(args.kicad_pcb)
mounting_holes = forced_pcb_supports
th_info = get_th_info(board, mounting_holes, ref_process_only_these, ref_do_not_process)

# Setup environment for file name expansion
os.environ["KIPRJMOD"] = os.path.split(args.kicad_pcb)[0]
path_sys_3dmodels = '/usr/share/kicad/3dmodels'
for ver in [6,7,8]: # Hmm - would we need more ?
    env_var_name = 'KICAD%d_3DMODEL_DIR'%(ver)
    if env_var_name not in os.environ:
        os.environ[env_var_name] = path_sys_3dmodels

# test if you can load all models
for comp in th_info:
    # We're guaranteed to have at-least one 3d model
    for modinfo in comp['models']:
        model_filename = os.path.expandvars(modinfo['model'])
        modinfo['mesh'] = mesh_ops.load_mesh(model_filename)
#pprint(th_info)

# Footprint useful things
# fp.GetCenter()
# fp.GetX(), fp.GetY() - same as center?
# fp.GetSide() - 0 for top, 31 for bottom
# fp.GetOrientation() - 0, 90, ...

mod_lines = []
geom_lines = []

output_fname = 'test.scad'
print('Creating output in %s...\n'%(output_fname))
fp_scad = open(output_fname, 'w')
# We use OpenSCAD to do the grunt work of building the
# actual model for us. Tesellation level must be set
# to match or exceed the limits of 3D printing.
# 3D printing generally works on the order of 0.1mm
# so we choose a value half of that, 0.05 mm
# This should give a decent balance of smooth shapes
# and file sizes, and processing needs.
fp_scad.write('''
// { These are configurable parameters
// you can tweak these here and count on
// openscad magic to do show you the result
// right away!

// Tesellate finer than 3D printer can print
$fs = 0.05;

// Gap for components to slide into their shell,
// on all sides in the horizontal plane (in mm)
shell_gap = %s;
                
// Thickness of shells (in mm)
shell_thickness = %s;

// Thickness of PCB (in mm)
pcb_thickness=%s;

// Clearance between typical shells and PCB
// Allows you to check and confirm component fit
shell_clearance=%s;

// Shells jet out of the PCB by this much
// (Mostly a visualization/debug aid)
shell_protrude=%s;

// Thickness of base (in mm)
// base provides a holding structure for the shells
// and connects it to the frame (on which the board
// sits)
base_thickness = %s;

// By default, we generate a base with a delaunay
// frame holding the shells together.
//
// Sometimes you just need a solid base - e.g. to
// put some text - like PCB info, version
// and even stuff like qrcodes! For these you
// mostly need a solid base. Set this is non-zero
// if that's what you need.
base_is_solid = %s;

// Board will sit on a lip, close to mounting holes
// Lips that lie in a square of this size (in mm)
// will be part of the model.
mounting_hole_support_size=%s;
'''%(shell_gap, shell_thickness, pcb_thickness, shell_clearance,
     shell_protrude, base_thickness, base_is_solid,
     mounting_hole_support_size))

pcb_segments = []
pcb_filled_shapes = []
seg_shapes = []

edge_cuts.load(board, pcb_segments, pcb_filled_shapes)

seg_shapes = []
if not edge_cuts.coalesce_segments(pcb_segments, seg_shapes):
    print('ERROR: Please check the edge cuts layer in KiCAD.')
    print('ERROR: There are incomplete outlines. DRC or 3D View should help')
    print('ERROR: diagnose the issue')
    sys.exit(-1)

edge_cuts.tesellate(arc_resolution, seg_shapes, pcb_filled_shapes)
edge_cuts.compute_areas(pcb_filled_shapes)

if len(pcb_filled_shapes) == 0:
    print('ERROR: At-least one filled shape is needed in Edge.Cuts layer')
    print('Please check and validate board file.')
    sys.exit(-1)

# Find the largest filled area. This is assumed to be the actual
# PCB outline
pcb_filled_shapes.sort(key=lambda x:x['area'], reverse=True)
# And hence these are the vertices
pcb_edge_points = pcb_filled_shapes[0]['vertices']

all_shells = []
fp_centers = []
topmost_z = 0
# For each TH component on the board
for th in th_info:
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
                          min_z, max_z, hull_verts,
                          mod_lines, geom_lines)
            all_shells.append({
                'name':shell_ident,
                'min_z':min_z,
                'max_z':max_z,
                'model':modinfo['model']})
            print('  Generating shell %s for mesh %s'%(shell_ident, modinfo['model']))
            topmost_z = max(topmost_z, max_z)

fp_scad.write('''
// Gap (in mm) between board edge and slot on which the board sits
pcb_gap=%s;

// Width of the lip (in mm) on which the board rests
pcb_overlap=%s;

// PCB holder's wall thickness (in mm)
// This will be the thickness after the gap
pcb_holder_perimeter=%s;

// The board can have a (lower) perimeter for added
// strength/aesthetics.  Perimeter height is above
// the base, and in mm
pcb_perimeter_height = %s;

// } End of configurable parameters

// { START : Computed Values

// Height of the tallest component on the top side
topmost_z=%s;
'''%(pcb_holder_gap, pcb_holder_overlap, pcb_holder_perimeter, pcb_perimeter_height, topmost_z))

fp_scad.write('// Height of the individual components\n')
for shell_info in all_shells:
    fp_scad.write('max_z_%s=%s; //3D Model: %s\n'%(shell_info['name'],shell_info['max_z'], shell_info['model']))
fp_scad.write('// } END : Computed Values\n')
fp_scad.write('\n')

# Write out the PCB edge
pcb_edge_points = np.array(pcb_edge_points)
pcb_edge_points[:,1] *= -1.0 # Negate all Ys to fix coordinate system
#hull = scipy.spatial.ConvexHull(pcb_edge_points)
#pcb_edge_hull = pcb_edge_points[hull.vertices]
fp_scad.write('module pcb_edge() {\n')
fp_scad.write('  polygon(\n')
fp_scad.write('    points=[\n')
for v in pcb_edge_points:
    fp_scad.write('      [%f,%f],\n'%(v[0],v[1]))
fp_scad.write('    ]\n')
fp_scad.write('  );\n')
fp_scad.write('}\n')

# Uncomment to see PCB edge as a 2D diagram :)
#from matplotlib import pyplot as plt
#x, y = pcb_edge_points.T
#plt.scatter(x1,y)
#plt.plot(x1,y1)
#plt.show()

fp_scad.write(''.join(mod_lines))
fp_scad.write(''.join(geom_lines))
# This module will include all shells
fp_scad.write('''
module mounted_component_shells() {
  translate([0,0,pcb_thickness])
    union() {
''')
for shell_info in all_shells:
    fp_scad.write('      %s();\n'%(ref2shell(shell_info['name'])))
fp_scad.write('''
    }
}
''')
# This module will include all pockets
fp_scad.write('module mounted_component_pockets() {\n')
fp_scad.write('  union() { \n')
for shell_info in all_shells:
    fp_scad.write('    %s();\n'%(ref2pocket(shell_info['name'])))
fp_scad.write('  }\n')
fp_scad.write('}\n')
fp_scad.write('\n')

# Compute bounding box of PCB
# FIXME: make this min, max code less verbose
pcb_min_x = pcb_max_x = pcb_edge_points[0][0]
pcb_min_y = pcb_max_y = pcb_edge_points[0][1]
for pt in pcb_edge_points:
    pcb_min_x = min(pcb_min_x, pt[0])
    pcb_max_x = max(pcb_max_x, pt[0])
    pcb_min_y = min(pcb_min_y, pt[1])
    pcb_max_y = max(pcb_max_y, pt[1])

pcb_bb_corners = [
    [pcb_min_x , pcb_min_y],
    [pcb_min_x , pcb_max_y],
    [pcb_max_x , pcb_min_y],
    [pcb_max_x , pcb_max_y]
]

fp_scad.write('''
module wide_line(start, end) {
    hull() {
        translate(start) cylinder(base_thickness);
        translate(end) cylinder(base_thickness);
    }
}\n\n''')

# Delaunay triangulation will be done on the following points
if jig_style_soldering_helper:
    # 1. centers of all considered footprints
    # 2. mounting holes
    # 3. bounding box corners of PCB edge. mounting holes are
    #    inside the PCB and don't extend all the way to the edge.
    #    If we don't include them, we may end up having a separate
    #    "delaunay island", depending on the exact PCB.
    dt_centers = fp_centers + mounting_holes + pcb_bb_corners
else:
    # it's enough to link footprint centers
    dt_centers = fp_centers

fp_scad.write('''
module base_solid() {
  translate([0,0,pcb_thickness+topmost_z]) {
    linear_extrude(base_thickness) {
      offset(r=pcb_holder_perimeter+pcb_gap) pcb_edge();
    }
  }
};
''')

fp_scad.write('module base_mesh() {\n')
fp_scad.write('  translate([0,0,pcb_thickness+topmost_z])\n')
fp_scad.write('  intersection() { \n')
fp_scad.write('    union(){ \n')
if len(dt_centers)>=4:
    fp_scad.write('        // delunay triangulated mesh\n')
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
        fp_scad.write('      wide_line(%s,%s);\n'%(a,b))
        fp_scad.write('      wide_line(%s,%s);\n'%(b,c))
        fp_scad.write('      wide_line(%s,%s);\n'%(c,a))
else:
    if len(dt_centers)>1:
        av = dt_centers[0]
        a = '[%s,%s]'%(av[0],av[1])
        bv = dt_centers[1]
        b = '[%s,%s]'%(bv[0],bv[1])
        fp_scad.write('      wide_line(%s,%s);\n'%(a,b))
    if len(dt_centers)==3:
        cv = dt_centers[2]
        c = '[%s,%s]'%(cv[0],cv[1])
        fp_scad.write('      wide_line(%s,%s);\n'%(b,c))
        fp_scad.write('      wide_line(%s,%s);\n'%(c,a))

fp_scad.write('    }\n')
fp_scad.write('''
    // ensure no peri lines go out of frame
    // tall shell supports must be included though
    linear_extrude(base_thickness)
      offset(r=pcb_holder_perimeter+pcb_gap)
        pcb_edge();
    }
}
''')

fp_scad.write('module component_shell_support() {\n')
fp_scad.write('  translate([0,0,pcb_thickness+topmost_z]) {\n')
fp_scad.write('    union() {\n')
for shell_info in all_shells:
    # if this component is less taller than the tallest component
    # then the extra must be filled up
    extra_extrude = topmost_z - shell_info['max_z']
    if extra_extrude>0:
        fp_scad.write('      translate([0,0,max_z_%s-topmost_z])\n'%(shell_info['name']))
        fp_scad.write('        linear_extrude(base_thickness+topmost_z-max_z_%s)\n'%(shell_info['name']))
        fp_scad.write('          %s();\n'% (ref2peri(shell_info['name'])))
    else:
        fp_scad.write('      linear_extrude(base_thickness)\n')
        fp_scad.write('        %s();\n'% (ref2peri(shell_info['name'])))
fp_scad.write('    }\n')
fp_scad.write('  }\n')
fp_scad.write('}\n')

if jig_style_soldering_helper:
    fp_scad.write('''
module pcb_holder() {
  // solid between perimeter & outline
  // full height
  linear_extrude(topmost_z+pcb_thickness+base_thickness) {
    difference() {
      offset(r=pcb_holder_perimeter+pcb_gap) pcb_edge();
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
        offset(r=pcb_holder_perimeter+pcb_gap) pcb_edge();
        offset(r=-pcb_overlap) pcb_edge();
      }
    }
  }
}
''')

if jig_style_soldering_helper:
    fp_scad.write('corner_support_height=60;\n') # FIXME: this is laziness! compute it!
    fp_scad.write('module pcb_corner_support() {\n')
    fp_scad.write('  translate([0,0,0])\n')
    fp_scad.write('  union() {\n')
    # FIXME: Policy: include PCB bounding box corners in support enforcers
    for pt in mounting_holes+pcb_bb_corners:
        fp_scad.write('    translate([%s,%s,0])\n'%(pt[0],pt[1]))
        fp_scad.write('      cube([mounting_hole_support_size, mounting_hole_support_size,  corner_support_height],center = true);\n')
    fp_scad.write('  }\n')
    fp_scad.write('}\n')

    fp_scad.write('''
// This _is_ the entire jig model. Structured such that
// you can individually check the parts
module complete_model() {
  difference() {
    union() {
      intersection() {
        pcb_corner_support();
        pcb_holder();
      };
      pcb_perimeter_short();
      if(base_is_solid==0) {
        base_mesh();
        } else {
        base_solid();
      }
      component_shell_support();
      mounted_component_shells();
    }
    mounted_component_pockets();
  }
}

complete_model();
''')
    fp_scad.write('''
// Stackup of the mesh
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
      linear_extrude(shell_clearance)
        square(sq_size);

  color("red")
    translate([sq_x-sq_gap,0,pcb_thickness+shell_clearance])
      linear_extrude(topmost_z-shell_clearance)
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
else:
    fp_scad.write('''
module complete_model() {
    union() {
        if(base_is_solid==0) {
            base_mesh();
        } else {
            base_solid();
        }
        component_shell_support();
        mounted_component_shells();
    }
}
complete_model();
''')

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

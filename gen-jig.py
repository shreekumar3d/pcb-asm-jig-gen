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
from solid2 import * # SolidPython
import solid2

# Standard imports
import argparse
import os
import functools
from pprint import pprint
import numpy as np
import sys
import tomllib
import json
import subprocess
import tempfile

# Local imports
from jigcommon import *
import edge_cuts
import mesh_ops
from solid2_module import module, exportReturnValueAsModule
import default_config

shell_protrude = 1 # shells will come above PCB by this much, so user can enable and see

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

mod_map = {}
shell_map = {}
pocket_map = {}
perimeter_map = {}

# sv => scad value
sv_max_z = {}
sv_shell_clearance = ScadValue('shell_clearance');
sv_shell_thickness = ScadValue('shell_thickness');
sv_shell_gap = ScadValue('shell_gap');
sv_pcb_thickness = ScadValue('pcb_thickness');
sv_shell_protrude = ScadValue('shell_protrude');

def gen_shell_shape(ref, x, y, rot, min_z, max_z, verts, mod_lines, geom_lines):
    sv_max_z[ref] = ScadValue('max_z_%s'%(ref))
    # first define the polygon so that we can do offset on it
    mod_name = ref2outline(ref)
    mod_map[ref] = module(mod_name, polygon(verts))
    #
    # note: max_z is positive - so this lifts
    #if rot==90 or rot==-90: # Invert 90 and -90
    #    rot=-rot
    shell_name = ref2shell(ref)
    # note linear extrude is only "max_z". Assumes top
    # side of the board. "min_z" is the extent from the top
    # side of the board all the way to the end of the pin
    # on the other side of the bottom.
    shell = translate([x,y,sv_shell_clearance]) (
                linear_extrude(sv_max_z[ref]-sv_shell_clearance) (
                    offset(sv_shell_gap+sv_shell_thickness) (
                        mod_map[ref]()
                    ) - offset(sv_shell_gap) (
                        mod_map[ref]()
                    )
                )
            )
    shell_map[ref] = module(shell_name, shell)

    pocket_name = ref2pocket(ref)
    pocket = translate([x,y,-sv_shell_protrude])(
                linear_extrude(sv_shell_protrude+sv_pcb_thickness+sv_max_z[ref]) (
                    offset(sv_shell_gap) (
                        mod_map[ref]()
                    )
                )
             )
    pocket_map[ref] = module(pocket_name, pocket)

    perimeter_name = ref2peri(ref)
    perimeter_solid = translate([x,y,sv_pcb_thickness+sv_max_z[ref]]) (
                     offset(sv_shell_gap+sv_shell_thickness) (
                         mod_map[ref]()
                     )
                 )
    perimeter_map[ref] = module(perimeter_name, perimeter_solid,
                           comment=f"Perimeter for {ref}")

#
# Execution starts here
#

parser = argparse.ArgumentParser()
parser.add_argument("--config", help='Use specified configuration options file')
parser.add_argument("--keep-orientation", action='store_true',
                    default=False,
                    help='''Match orientation of the output to KiCAD 3D view.
The default orients the output for direct printing (i.e. rotated by 180 degrees
in along the X axis.''')
parser.add_argument("--output-format", default='stl',
                    choices=['stl','oscad'],
                    help='Output file format')
parser.add_argument("kicad_pcb", help='KiCAD PCB file (.kicad_pcb) to process')
parser.add_argument("output", help='Output file to generate.')
args = parser.parse_args()

if args.config:
    config_text = open(args.config, 'r').read()
    cfg = tomllib.load(open(args.config,'rb'))
    #print(json.dumps(cfg, indent=2))
else:
    config_text = default_config.get()
    cfg = tomllib.loads(config_text)

pcb_thickness = cfg['pcb']['thickness']
shell_clearance = cfg['component_shell']['clearance_from_pcb']
shell_gap = cfg['component_shell']['gap']
shell_thickness = cfg['component_shell']['thickness']
base_is_solid = 0 if cfg['holder']['base']['type']=="mesh" else 1

base_thickness = cfg['holder']['base']['thickness']
arc_resolution = cfg['pcb']['tesellate_edge_cuts_curve']

mesh_line_width = cfg['holder']['base']['mesh']['line_width']
mesh_line_height = cfg['holder']['base']['mesh']['line_height']
pcb_perimeter_height = cfg['holder']['base']['perimeter_height']
pcb_holder_gap = cfg['holder']['pcb_gap']
pcb_holder_overlap = cfg['holder']['pcb_overlap']
pcb_holder_perimeter = cfg['holder']['perimeter']
forced_pcb_supports = cfg['holder']['forced_lips']
lip_size = cfg['holder']['lip_size']
ref_do_not_process = cfg['refs']['do_not_process']
ref_process_only_these = cfg['refs']['process_only_these']
jig_style = cfg['jig']['type']
jig_style_range = ['th_soldering', 'component_fitting']
if jig_style not in jig_style_range:
    print('BAD value "%s" for jig_options/style. Valid values are : %s' %
        (jig_style,','.join(jig_style_range)))
    sys.exit(-1)
jig_style_th_soldering = (jig_style == 'th_soldering')
jig_type_component_fitting = (jig_style == 'component_fitting')

if jig_type_component_fitting:
    if shell_clearance>0:
        print('INFO: Generating component shells, note shell_clearance=%s will cut into shell.'
            %(shell_clearance))

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

if args.output_format == 'stl':
    fp_scad = tempfile.NamedTemporaryFile(mode='w', suffix='.scad', delete_on_close=False)
    oscad_filename = fp_scad.name
    stl_filename = args.output
else:
    oscad_filename = args.output
    fp_scad = open(oscad_filename, 'w')

fp_scad.write('''
// Auto generated file by jig-gen, the awesome automatic
// jig generator for your PCB designs.
//
// Input board file   : %s
// Configuration file : %s
//
// Complete configuration file is embedded at the end of this
// file.
'''%(args.kicad_pcb,
    '(Tool Default Internal Configuration)' if not args.config else args.config))

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

// Line width in the mesh
mesh_line_width = %s;

// Line height in the mesh
mesh_line_height = %s;

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
lip_size=%s;

'''%(shell_gap, shell_thickness, pcb_thickness, shell_clearance,
     shell_protrude, base_thickness, mesh_line_width, mesh_line_height,
     base_is_solid, lip_size))

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

lip_width = max(pcb_gap+pcb_holder_perimeter, pcb_overlap)*1.2;
tiny_dimension = 0.001;
base_z =  pcb_thickness+topmost_z+base_thickness+2*tiny_dimension;

mesh_start_z = pcb_thickness+topmost_z+base_thickness-mesh_line_height;
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
sm_pcb_edge = module('pcb_edge', polygon(pcb_edge_points))

# Uncomment to see PCB edge as a 2D diagram :)
#from matplotlib import pyplot as plt
#x, y = pcb_edge_points.T
#plt.scatter(x1,y)
#plt.plot(x1,y1)
#plt.show()

fp_scad.write(''.join(mod_lines))
fp_scad.write(''.join(geom_lines))
# This module will include all shells
combined_shell = union()
for shell_info in all_shells:
    combined_shell += shell_map[shell_info['name']]()

sm_mounted_component_shells = module('mounted_component_shells',
        translate([0,0,sv_pcb_thickness]) (
            combined_shell
        )
    )

# This module will include all pockets
combined_pockets = union()
for shell_info in all_shells:
    combined_pockets += pocket_map[shell_info['name']]()

sm_mounted_component_pockets = module('mounted_component_pockets',
        combined_pockets
    )

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

sv_base_thickness = ScadValue('base_thickness')
sv_mesh_line_width = ScadValue('mesh_line_width')
sv_mesh_line_height = ScadValue('mesh_line_height')
sv_mesh_start_z = ScadValue('mesh_start_z')
@exportReturnValueAsModule
def wide_line(start, end):
    return solid2.hull()(
            translate(start)(cylinder(h=sv_mesh_line_height, d=sv_mesh_line_width))+
            translate(end)(cylinder(h=sv_mesh_line_height, d=sv_mesh_line_width))
            )
# Delaunay triangulation will be done on the following points
if jig_style_th_soldering:
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

sv_topmost_z = ScadValue('topmost_z')
sv_pcb_holder_perimeter = ScadValue('pcb_holder_perimeter')
sv_pcb_gap = ScadValue('pcb_gap')
sv_pcb_overlap = ScadValue('pcb_overlap')
sv_pcb_perimeter_height = ScadValue('pcb_perimeter_height')
base_solid = translate([0,0,sv_pcb_thickness+sv_topmost_z])(
                linear_extrude(sv_base_thickness) (
                    offset(sv_pcb_holder_perimeter+sv_pcb_gap)(
                        sm_pcb_edge()
                    )
                )
             )
sm_base_solid = module('base_solid', base_solid)

mesh_lines = union()
if len(dt_centers)>=4:
    mesh_comment = 'delaunay triangulated mesh'
    d_verts = np.array(dt_centers)
    d_tris = scipy.spatial.Delaunay(d_verts)
    for tri in d_tris.simplices:
        # tri is a,b,c
        av = d_verts[tri[0]]
        a = [av[0], av[1]]
        bv = d_verts[tri[1]]
        b = [bv[0], bv[1]]
        cv = d_verts[tri[2]]
        c = [cv[0], cv[1]]
        mesh_lines += wide_line(a,b)
        mesh_lines += wide_line(b,c)
        mesh_lines += wide_line(c,a)
else:
    if len(dt_centers)>1:
        av = dt_centers[0]
        a = [av[0],av[1]]
        bv = dt_centers[1]
        b = [bv[0],bv[1]]
        mesh_lines += wide_line(a,b)
    if len(dt_centers)==3:
        cv = dt_centers[2]
        c = [cv[0],cv[1]]
        mesh_lines += wide_line(b,c)
        mesh_lines += wide_line(c,a)

base_mesh_volume = linear_extrude(sv_mesh_line_height) (
                       offset(sv_pcb_holder_perimeter+sv_pcb_gap) (
                           sm_pcb_edge()
                       )
                   )
base_mesh = translate([0,0,sv_mesh_start_z]) (
                intersection() (
                    mesh_lines,
                    base_mesh_volume
                )
            )

sm_base_mesh = module('base_mesh', base_mesh())

support_component_shells = union()
for shell_info in all_shells:
    this_ref = shell_info['name']
    # if this component is less taller than the tallest component
    # then the extra must be filled up
    extra_extrude = topmost_z - shell_info['max_z']
    if extra_extrude>0:
        support_component_shells += translate([0,0,sv_max_z[this_ref]-sv_topmost_z]) (
                linear_extrude(sv_base_thickness+sv_topmost_z-sv_max_z[this_ref]) (
                    perimeter_map[this_ref]()
                )
            )
    else:
        # we only need a solid as thick as the base to hold the
        # component from falling down :)
        support_component_shells += linear_extrude(sv_base_thickness) (
                perimeter_map[this_ref]()
            )

component_shell_support = translate([0,0,sv_pcb_thickness+sv_topmost_z]) (
        support_component_shells
    )

sm_component_shell_support = module('component_shell_support', component_shell_support)

pcb_holder = linear_extrude(sv_topmost_z+sv_pcb_thickness+sv_base_thickness) (
        difference()(
            offset(sv_pcb_holder_perimeter+sv_pcb_gap)(
                sm_pcb_edge()
            ),
            offset(sv_pcb_gap) (
                sm_pcb_edge()
            )
        )
    ) + translate([0,0,sv_pcb_thickness]) (
        linear_extrude(sv_topmost_z+sv_base_thickness) (
            difference() (
                offset(sv_pcb_gap)(
                    sm_pcb_edge()
                ),
                offset(-sv_pcb_overlap)(
                    sm_pcb_edge()
                )
            )
        )
    )
sm_pcb_holder = module('pcb_holder', pcb_holder,
        comment = 'solid between perimeter and outline, full height')

pcb_perimeter_short = translate([0,0,sv_pcb_thickness+sv_topmost_z-sv_pcb_perimeter_height]) (
        linear_extrude(sv_pcb_perimeter_height+sv_base_thickness) (
            difference() (
                offset(sv_pcb_holder_perimeter+sv_pcb_gap) (
                    sm_pcb_edge()
                ),
                offset(-sv_pcb_overlap)(
                    sm_pcb_edge()
                )
            )
        )
    )

sm_pcb_perimeter_short = module('pcb_perimeter_short', pcb_perimeter_short)

if jig_style_th_soldering:
    lip_lines = edge_cuts.compute_lips(arc_resolution, pcb_filled_shapes[0], lip_size)
    #print('lip lines no = ',len(lip_lines))
    #pprint(lip_lines)
    @exportReturnValueAsModule
    def peri_line(start, end, line_width):
        return solid2.hull() (
            circle(d=line_width).translate(start),
            circle(d=line_width).translate(end)
        )

    #fp_scad.write('  lip_width = max(pcb_gap+pcb_holder_perimeter, pcb_overlap)*1.2;\n')
    #fp_scad.write('  tiny_dimension = 0.001;\n')
    #fp_scad.write('  base_z =  pcb_thickness+topmost_z+base_thickness+2*tiny_dimension;\n')
    sv_lip_width = ScadValue('lip_width')
    sv_tiny_dimension = ScadValue('tiny_dimension')
    sv_base_z = ScadValue('base_z')
    s_lip_lines = union()
    for line in lip_lines:
        # FIXME: see the -y below? This is ugliness. Aim for consistency
        s_lip_lines += peri_line(
                        [line[0][0],-line[0][1]],
                        [line[1][0],-line[1][1]],
                        sv_lip_width)
    sm_pcb_support_lips = module('pcb_support_lips',
            translate([0,0,-sv_tiny_dimension]) (
                linear_extrude(sv_base_z) (
                    s_lip_lines
                )
            )
    )

    fp_scad.write(scad_render(ScadValue('//')))
    fp_scad.write('''
// This _is_ the entire jig model. Structured such that
// you can individually check the parts
module complete_model() {
  difference() {
    union() {
      intersection() {
        pcb_support_lips();
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
''')

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
''')

fp_scad.write('''
orient_to_print=%d;
if(orient_to_print == 1) {
  rotate([180,0,0])
    complete_model();
} else {
    complete_model();
}
'''%(not args.keep_orientation))


# Help understanding
if jig_style_th_soldering:
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

fp_scad.write('''
/*
%s
*/
'''%(config_text))

fp_scad.close()

if args.output_format == 'stl':
    cmd = [ 'openscad', '--hardwarnings', '-o', stl_filename, oscad_filename]
    print('Generating output using : %s'%(' '.join(cmd)))
    print('-----------------------------------------')
    retcode = subprocess.call(cmd)
    print('-----------------------------------------')
    if retcode !=0 :
        print('ERROR: OpenSCAD Failed, exit code %d'%(retcode))
    else:
        print('Done, output : %s'%(stl_filename))
else:
    print('Done, output : %s'%(oscad_filename))
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

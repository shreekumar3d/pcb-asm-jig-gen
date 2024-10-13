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

# Standard imports
import tempfile
import argparse
import subprocess
import os
import functools
from pprint import pprint
import numpy as np

mesh_cache = {}

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

def get_th_info(board):
    fp_list = board.Footprints()
    th_info = []
    for fp in fp_list:
        if fp.HasThroughHolePads():
            thc_models = []
            for mod3d in fp.Models():
                thc_models.append({
                        'model'    : mod3d.m_Filename,
                        'offset'   : mod3d.m_Offset,
                        'scale'    : mod3d.m_Scale,
                        'rotation' : mod3d.m_Rotation
                })
            if len(thc_models)>0:
                th_info.append({
                    'ref': fp.GetReference(),
                    'x' : units_to_mm(fp.GetX()),
                    'y' : units_to_mm(fp.GetY()),
                    'orientation' : fp.GetOrientation().AsDegrees(),
                    'side' : fp.GetSide(),
                    'position' : fp.GetPosition(),
                    'models' : thc_models,
                    'kfp' : fp}) # Reference if we need more info later
        #print(fp.GetReference())
        #print('  Position(mm):', units_to_mm(fp.GetX()), units_to_mm(fp.GetY()))
        #print('  On side     :', fp.GetSide())
        #print('  Orientation :', fp.GetOrientation().AsDegrees())
        #print('  DNP ?       :', fp.IsDNP())
        #print('  TH ?        :', fp.HasThroughHolePads())
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

def write_shell_shape(ref, x, y, rot, min_z, max_z, fp, verts):
    #
    # here we do some coordinate hackery
    # some fun things about KiCAD
    # PCB has this coordinate system
    # X ----->
    # Y
    # |
    # |
    # V
    #
    # But 3D models are in the regular coordinate system!
    #
    # ^
    # |
    # Y
    # X ------->
    #
    # Z increases upwards in the 3D model space
    # 
    # So, we hack things around a bit for sanity
    # But this "sanity" also means that the top view
    # becomes the "bottom" view in openscad
    # So, Z decreases "upwards"


    # first define the polygon so that we can do offset on it
    mod_name = 'ref_%s'%(ref)
    fp.write('module %s() {\n'%(mod_name))
    fp.write('  polygon(\n')
    fp.write('    points=[\n')

    for v in verts:
        fp.write('      [%f,%f],\n'%(v[0],-v[1])) # invert Y to match coordinate system
    fp.write('    ]\n')
    fp.write('  );\n')
    fp.write('}\n')
    #
    # note: max_z is positive - so this lifts
    fp.write('translate([%f,%f,%f])\n'%(x,y, -max_z))
    if rot==90 or rot==-90: # Invert 90 and -90
        rot=-rot
    fp.write('  rotate([0,0,%f])\n'%(rot))
    # note linear extrude is only "max_z". Assumes top
    # side of the board. "min_z" is the extent from the top
    # side of the board all the way to the end of the pin
    # on the other side of the bottom.
    fp.write('    linear_extrude(%f)\n'%(max_z))
    fp.write('      difference() {\n')
    fp.write('        offset(r=clearance+wall_thickness)\n')
    fp.write('          %s();\n'%(mod_name))
    fp.write('        offset(clearance)\n')
    fp.write('          %s();\n'%(mod_name))
    fp.write('      }\n')

output_fname = 'test.scad'
print('Creating output in %s...\n')
fp_scad = open(output_fname, 'w')
fp_scad.write('clearance = 0.1;\n');
fp_scad.write('wall_thickness = 1.2;\n');
# This module will include all shells
fp_scad.write('module holders() {\n')
# For each TH component on the board
for th in th_info:
    print('Processing ', th['ref'])
    if th['ref'] in ['U1','U4','U5']:
        print('Skipping known component %s to work around issues'%(th['ref']))
        continue
    #if th['ref'] not in [ 'J12', 'J10']:
    #    continue
    # transform the mesh to its final position on the board
    result_verts = 0
    mesh_verts = map(lambda x: x['mesh'].shape[0], th['models'])
    result_verts = functools.reduce(lambda a,b: a+b, mesh_verts)
    #print('  Result verts: ', result_verts)
    combined_xy = np.zeros((result_verts,2))
    st_idx = 0
    min_z = 0
    max_z = 0
    for modinfo in th['models']:
        this_len = modinfo['mesh'].shape[0]
        #print('     ', st_idx, this_len)
        if this_len==0:
            continue # FIXME: how is this possible ??
        #for v in modinfo['mesh']:
        #    print(v)
        combined_xy[st_idx:st_idx+this_len] = modinfo['mesh'][:,0:2] # get rid of the Z coordinate
        min_z = min(min_z, min(modinfo['mesh'][:,2]))
        max_z = max(max_z, max(modinfo['mesh'][:,2]))
        st_idx += this_len
        #combined_xy[:,0] += th['x']
        #combined_xy[:,1] += th['y']
    if result_verts>0:
        #for v in combined_xy:
        #    print(v)
        # FIXME: how is ==0 possible?
        hull = scipy.spatial.ConvexHull(combined_xy)
        hull_verts = combined_xy[hull.vertices]
        print('Hull size = ', len(hull.vertices), ' min Z=', min_z, ' max Z=', max_z)
        print(hull_verts)
        fp_scad.write('//%s\n'%(th['ref']))
        write_shell_shape(th['ref'],
                      th['x'], th['y'], th['orientation'],
                      min_z, max_z,
                      fp_scad, hull_verts)
fp_scad.write('}\n')
fp_scad.write('\n')
fp_scad.write('// convert to regular 3D coordinate system\n')
fp_scad.write('// this is more friendly with step files\n')
fp_scad.write('// but ensure your export from KiCAD with origin as grid origin\n')
fp_scad.write('// without this, there will be an offset\n')
fp_scad.write('rotate([180,0,0])\n')
fp_scad.write('  holders();')

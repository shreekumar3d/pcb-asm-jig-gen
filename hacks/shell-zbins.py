#
# create a test "hugging" shell around a component (represented by
# a mesh)
#
import tinyobjloader
import numpy as np
from matplotlib import pyplot as plt
from pprint import pprint
import scipy.spatial
from itertools import cycle
import os
import sys

# This is why this directory is called 'hacks' !!
import sys
sys.path.insert(0, '..')
import tripy

reader = tinyobjloader.ObjReader()
if len(sys.argv)==2:
	ret = reader.ParseFromFile(sys.argv[1])
else:
	ret = reader.ParseFromFile('PinHeader_1x01_P2.54mm_Vertical.obj')
attrib = reader.GetAttrib()
nverts = len(attrib.vertices)//3 # attrib.vertices is contiguous x,y,z
mesh = np.array(attrib.vertices)
mesh.resize((nverts,3)) # in place change in dims
print('Vertices = %d'%(nverts))


# Find all the unique Zs
all_z = np.unique(mesh[:,2])
all_z.sort()
reverse_z = all_z[::-1]

z_bins = []
z_bins.append({
    "start_z" : reverse_z[0],
    "z_list" : [],
    "verts" : None
})

z_bin_size = 0.5
# Walk the mesh from top (high Z) to bottom(0 or -ve Z)
# cluster vertices in Z bins of bin_size
for z_val in reverse_z:
	#print(z_val)
	# find all the verts at this Z
	mask = np.isin(element = mesh[:,2], test_elements=z_val)
	result = mesh[mask]

	# append to existing bin, if we belong to the same bin,
	# else start a new bin
	last_z_bin = z_bins[-1]["start_z"]
	if last_z_bin-z_val < z_bin_size:
		if z_bins[-1]["verts"] is None:
			z_bins[-1]["verts"] = result
		else:
			z_bins[-1]["verts"] = np.concatenate([z_bins[-1]["verts"], result])
		z_bins[-1]["z_list"].append(z_val)
	else:
		z_bins.append({
			"start_z" : z_val,
			"z_list" : [z_val],
			"verts" : result
		})

#pprint(z_bins)

# Each bin basically represents an area that casts a shadow looking down at Z=0
# We can take the convex hull of the area.
# Subsequent bins will keep adding to convex hulls from the previous bins
# Thus, cumulatively we'll cover every bin, and the largest covex hull will be
# at the bottom, allowing the part to slide in.

prev_bin = None
hull_bins = []
for this_bin in z_bins:
	points_xy = this_bin['verts'][:,0:2] # chuck Z
	if len(hull_bins)>0:
		points_xy = np.concatenate([points_xy, hull_bins[-1]['hull']])
	hull = scipy.spatial.ConvexHull(points_xy)
	hull_verts = points_xy[hull.vertices]
	tris = tripy.earclip(hull_verts)
	area = tripy.calculate_total_area(tris)
	if len(hull_bins)>0:
		if area > hull_bins[-1]['area']+1: # heh - looking for some meaningful change :)
			# start new bin
			hull_bins.append({
				'hull' : hull_verts,
				'area' : area,
				'z_list' : this_bin['z_list']
			})
		else:
			hull_bins[-1]['z_list'] = hull_bins[-1]['z_list']+this_bin['z_list']
	else:
		hull_bins.append({
			'hull' : hull_verts,
			'area' : area,
			"z_list" : this_bin["z_list"],
		})

# reverse the hull bins, as it represents the right order of
# cutting out!
hull_bins.reverse()
#pprint(hull_bins)

for idx, h_bin in enumerate(hull_bins):
	h_bin['z_list'].sort()
	if idx==0:
		h_bin['start_z'] = h_bin['z_list'][0]
	else:
		h_bin['start_z'] = hull_bins[idx-1]['z_list'][-1]
	pprint(h_bin)

fp_scad = open('test.scad', 'w')

# each hull becomes a module in oscad
for idx, h_bin in enumerate(hull_bins):
	s_pts = ''
	for pt in h_bin['hull']:
		s_pts += '[%f,%f],\n'%(pt[0],pt[1])
	fp_scad.write('''
module hull_%d() {
	polygon(points=[
%s
	]);
}
'''%(idx, s_pts))

# offset of the first hull is extruded all the way from the bottom to thea top
# this is the external solid
fp_scad.write('''
difference() {
	translate([0,0,%f])
		linear_extrude(%f)
			offset(r=1.6+0.1)
				hull_0();
'''%(
	hull_bins[0]['z_list'][0],
	hull_bins[-1]['z_list'][-1] - hull_bins[0]['z_list'][0]
))

# the hulls are now subtracted, overlapping each by a small amount
tiny_overlap = 0.001
fp_scad.write('''
tiny_overlap = %s;
'''%(tiny_overlap))
for idx, h_bin in enumerate(hull_bins):
	fp_scad.write('''
	translate([0,0,%f-tiny_overlap])
	    linear_extrude(%f+2*tiny_overlap)
	        offset(r=0.1)
			hull_%d();
'''%(
	hull_bins[idx]['start_z'],
	hull_bins[idx]['z_list'][-1]-hull_bins[idx]['start_z'],
	idx
))

fp_scad.write('''
}
''')

# Plot and show results
cycol = cycle('bgrcmk')
for h_bin in hull_bins:
	x, y = h_bin['hull'].T
	plt.fill(x,y, fill=False, c=next(cycol))
plt.show()

#pprint(z_bins)
#mesh_xy = mesh[:,0:2]
#x, y = mesh_xy.T
#plt.scatter(x,y)

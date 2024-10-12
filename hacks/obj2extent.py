#!/usr/bin/env python3

import sys
import tinyobjloader
import subprocess
import time
from pprint import pprint
import pyclipper

reader = tinyobjloader.ObjReader()
ret = reader.ParseFromFile(sys.argv[1])
if ret == False:
    sys.exit(-1)

attrib = reader.GetAttrib()
print("attrib.vertices = ", len(attrib.vertices))
print("attrib.normals = ", len(attrib.normals))
print("attrib.texcoords = ", len(attrib.texcoords))

# vertices are a flat array
# v[0] is x, v[1] is y, v[2] is z...

nverts = len(attrib.vertices)//3
print('verts = ',nverts)
min_z = attrib.vertices[2]
max_z = min_z
for i in range(nverts):
    val = attrib.vertices[i*3+2]
    min_z = min(min_z, val)
    max_z = max(max_z, val)

print('Min Z = ', min_z, ' Max Z = ', max_z)

def get_hull(nverts, vertices):
    p = subprocess.Popen(['qhull', 'Qt', 'o'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, universal_newlines=True)

    p.stdin.write('2 rbox D2\n')
    p.stdin.write('%d\n'%(nverts))
    for i in range(nverts):
        x = vertices[i*3]
        y = vertices[i*3+1]
        p.stdin.write('%f %f\n'%(x, y))
    p.stdin.close()
    # Read the first line - ignore.
    p.stdout.readline()
    l = p.stdout.readline().strip()
    hull_nverts, hull_faces, hull_edges = [ int(x) for x in l.split() ]
    hull_verts = []
    for i in range(hull_nverts):
        x, y = [ float(x) for x in p.stdout.readline().strip().split() ]
        hull_verts.append((x,y))
    edges = []
    for i in range(hull_edges):
        n, idx1, idx2 = [ int(x) for x in p.stdout.readline().strip().split() ]
        edges.append((idx1, idx2))

    print(p.stdout.read()) # this should be empty!

    # qhull doesn't seem to return edges in order.
    # convert it to a loop
    loop_edges = [edges[0]]
    edge_start = edges[0][0]
    edge_end = edges[0][1]
    print(hull_verts[edge_start])
    for j in range(hull_edges-1):
        for i in range(hull_edges):
            if edges[i][0] == edge_end:
                print(hull_verts[edge_end])
                loop_edges.append(edges[i])
                edge_end = edges[i][1]
                break
    #    edges.append([hull_verts[idx1], hull_verts[idx2]])
    #print('ngon is:')
    #fp_ngon = open(ngon_fname,'w')
    #pprint(loop_edges)
    #for pt in loop_edges:
    #    fp_ngon.write('%s %s\n'%(hull_verts[pt[0]][0], hull_verts[pt[0]][1]))
    #print('ngon coordinates:')
    #for e_start, e_end in loop_edges:
    #    print(hull_verts[e_start][0], hull_verts[e_end])
    #for pt in loop_edges:
    #fp_ngon.close()
    ngon = []
    for pt in loop_edges:
        ngon.append((hull_verts[pt[0]][0], hull_verts[pt[0]][1]))
    pprint(ngon)
    return ngon

def offset_one_poly(poly, scale, value):
    int_poly = pyclipper.scale_to_clipper(poly, scale)
    pco = pyclipper.PyclipperOffset()
    pco.AddPath(int_poly, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    solution = pco.Execute(value)[0]
    return pyclipper.scale_from_clipper(solution, scale)

def write_ngon(ngon_fname, poly):
    fp_ngon = open(ngon_fname,'w')
    for pt in poly:
        fp_ngon.write('%s %s\n'%(pt[0], pt[1]))
    fp_ngon.close()

hull_poly = get_hull(nverts, attrib.vertices)
poly_inner = offset_one_poly(hull_poly, 10000, 1000) # .1 mm
poly_outer = offset_one_poly(hull_poly, 10000, 17000) # 1.6 mm
print('inner has  verts ', len(poly_inner))
print('outer has  verts ', len(poly_outer))
write_ngon('ngon.txt', poly_inner)
write_ngon('ngon2.txt', poly_outer)
print('Now run lender --background --python proc-ngon.py')
#write_hull('ngon.txt', nverts, attrib.vertices, True, 0.1, 0.1)
#write_hull('ngon2.txt', nverts, attrib.vertices, True, 1.3, 1.3)

#print(dir(p))
#print(attrib.numpy_vertices)
#print("numpy_v = {}".format(attrib.numpy_vertices()))

#for i, v in enumerate(attrib.vertices):
#    print("v[{}] = {}".format(i, v))
#    #print(type(v))

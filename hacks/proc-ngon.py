# offset-poly at
# https://pypi.org/project/offset-poly/0.3.3/

from mathutils import Vector
import bpy
from bpy.types import Operator
from bpy.props import FloatVectorProperty
from bpy_extras.object_utils import AddObjectHelper, object_data_add
from mathutils import Vector

# Delete all objects
# Get rid of the cube or anything else! We have no control over the default
# scene of the user
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

def create_ngon_mesh(ngon_fname, mesh_name, ob_name, collection_name, loc=Vector((0,0,0))):
    fp_ngon = open(ngon_fname, 'r')
    verts = []

    for line in fp_ngon:
        x, y = [float(x) for x in line.split()]
        verts.append(Vector((x,y,0)))

    edges = []
    faces = [range(len(verts))]

    mesh = bpy.data.meshes.new(name=mesh_name)
    mesh.from_pydata(verts, edges, faces)

    ob = bpy.data.objects.new(ob_name, mesh)
    ob.location = loc #by.context.scene.cursor_location   # position object at 3d-cursor

    # Dunno, but why should we create a new collection? We can insert to default tree ?
    # https://b3d.interplanety.org/en/how-to-create-mesh-through-the-blender-python-api/
    new_collection = bpy.data.collections.new(collection_name)
    bpy.context.scene.collection.children.link(new_collection)
    new_collection.objects.link(ob)

create_ngon_mesh('ngon.txt', 'mesh1', 'mesh_inner', 'collection1',Vector((0,0,1)))
create_ngon_mesh('ngon2.txt', 'mesh2', 'mesh_outer', 'collection2')


# useful for development when the mesh may be invalid.
# mesh.validate(verbose=True)


# Must be in object mode for the following to work
bpy.ops.object.select_all(action='DESELECT')
# see https://docs.blender.org/api/current/bpy.ops.object.html

# select the mesh object we created
#bpy.context.scene.objects['mesh_inner'].select_set(True)
#print(bpy.context.scene.objects[ob_name].select_get()) #will return true

#print(bpy.context.selected_objects)

bpy.context.view_layer.objects.active = bpy.context.scene.objects['mesh_inner']
bpy.ops.object.editmode_toggle() # enter edit mode
bpy.ops.mesh.select_all(action='SELECT') # select all vertices
bpy.ops.mesh.extrude_region_move(TRANSFORM_OT_translate={"value":(0, 0, 10) }) # extrude 10 units in Z
bpy.ops.object.editmode_toggle() # exit edit mode

bpy.context.view_layer.objects.active = bpy.context.scene.objects['mesh_outer']
bpy.ops.object.editmode_toggle() # enter edit mode
bpy.ops.mesh.select_all(action='SELECT') # select all vertices
bpy.ops.mesh.extrude_region_move(TRANSFORM_OT_translate={"value":(0, 0, 10) }) # extrude 10 units in Z
bpy.ops.object.editmode_toggle() # exit edit mode

bpy.ops.object.modifier_add(type='BOOLEAN')
bpy.context.object.modifiers["Boolean"].operation = 'DIFFERENCE'
bpy.context.object.modifiers["Boolean"].solver = 'FAST'
bpy.context.object.modifiers["Boolean"].object = bpy.context.scene.objects['mesh_inner']

ob = bpy.context.scene.objects['mesh_outer']
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
bpy.ops.wm.save_as_mainfile(filepath="blah.blend")
bpy.ops.wm.stl_export(filepath='blah.stl',export_selected_objects=True)

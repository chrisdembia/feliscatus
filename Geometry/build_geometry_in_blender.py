from math import *
import os
import bpy.ops

candidate_list = [item.name for item in bpy.data.objects if item.type == "MESH"]
 
# Select them only.
for object_name in candidate_list:
    bpy.data.objects[object_name].select = True
 
# Remove all selected.
bpy.ops.object.delete()
 
# Remove the meshes, they have no users anymore.
for item in bpy.data.meshes:
    bpy.data.meshes.remove(item)
    
# Re-build the meshes.
torso_diameter = 1
torso_length = 1
nub_diameter = 0.2
nub_length = .4
nub_offset = .3
torso = bpy.ops.mesh.primitive_cylinder_add(vertices=100, 
        radius=0.5*torso_diameter, depth=torso_length,
        location=(0.5*torso_length, 0, 0),
        rotation=(0, 0.5*pi, 0))
        
offset_sign = [1, -1]
nub = list()
for i in range(len(offset_sign)):
    nub.append(bpy.ops.mesh.primitive_cylinder_add(vertices=50,
            radius=0.5*nub_diameter, depth=nub_length,
            location=(0.75*torso_length, offset_sign[i]*nub_offset, -0.35*torso_diameter),
            rotation=(0, 0, 0)))

bpy.ops.export_scene.obj(filepath=os.path.join(os.path.split(bpy.data.filepath)[0], 
        "feliscatus_cylinder_with_two_offset_feet_nubs.obj"))
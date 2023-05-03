import bpy
import os.path
import glob
meshdir = "./"

for dae_file in glob.glob(os.path.join(meshdir, "*.dae")):
    bpy.ops.wm.collada_import(filepath=dae_file)
    name = os.path.split(dae_file)[1]
    name = name.split(".")[0] + ".stl"
    bpy.ops.export_mesh.stl(filepath=os.path.join(meshdir, name), axis_forward='-Z', axis_up='Y')

    for item in bpy.context.scene.objects:
        bpy.context.scene.objects.unlink(item)
    for item in bpy.data.objects:
        bpy.data.objects.remove(item)
    for item in bpy.data.meshes:
        bpy.data.meshes.remove(item)
    for item in bpy.data.materials:
        bpy.data.materials.remove(item)

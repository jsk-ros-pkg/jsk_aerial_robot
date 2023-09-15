import bpy
import os.path
import glob
meshdir = "./"

# blender -b -P convert.py

for dae_file in glob.glob(os.path.join(meshdir, "*.dae")):
    for col in bpy.data.collections:
        for item in col.objects:
            col.objects.unlink(item)
            bpy.data.objects.remove(item)

    for item in bpy.context.scene.collection.objects:
        bpy.context.scene.collection.objects.unlink(item)
        bpy.data.objects.remove(item)

    for item in bpy.data.meshes:
        bpy.data.meshes.remove(item)

    # for item in bpy.data.materials:
    #     bpy.data.materials.remove(item)

    bpy.ops.wm.collada_import(filepath=dae_file)
    name = os.path.split(dae_file)[1]
    name = name.split(".")[0] + ".stl"
    bpy.ops.export_mesh.stl(filepath=os.path.join(meshdir, name))


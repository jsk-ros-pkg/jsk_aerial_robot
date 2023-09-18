# $ blender -b -P convert.py -- meshdir

import bpy
import os.path
import sys
import glob

meshdir = sys.argv
if len(meshdir) != 6:
    print("$ blender -b -P convert.py -- meshdir")
    sys.exit()

meshdir = meshdir[5]

def delete_all():
    if bpy.app.version[1] > 80:
        for col in bpy.data.collections:
            for item in col.objects:
                col.objects.unlink(item)
                bpy.data.objects.remove(item, do_unlink=True)

        for item in bpy.context.scene.collection.objects:
            bpy.context.scene.collection.objects.unlink(item)
            bpy.data.objects.remove(item, do_unlink=True)

        for item in bpy.data.meshes:
            bpy.data.meshes.remove(item, do_unlink=True)

        # for item in bpy.data.materials:
        #     bpy.data.materials.remove(item)

    if bpy.app.version[1] <= 79:
        for item in bpy.context.scene.objects:
            bpy.context.scene.objects.unlink(item)
        for item in bpy.data.objects:
            bpy.data.objects.remove(item)
        for item in bpy.data.meshes:
            bpy.data.meshes.remove(item)
        for item in bpy.data.materials:
            bpy.data.materials.remove(item)

def process_subdirectories(root):
    for foldername, subfolders, filenames in os.walk(root):
        for filename in filenames:
            if filename.endswith(".dae"):
                input_file = os.path.join(foldername, filename)
                output_file = os.path.join(foldername, os.path.splitext(filename)[0] + ".stl")

                delete_all()

                bpy.ops.wm.collada_import(filepath=input_file)
                bpy.ops.export_mesh.stl(filepath=output_file)

process_subdirectories(meshdir)

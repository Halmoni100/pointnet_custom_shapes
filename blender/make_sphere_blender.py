import bpy
import bmesh

scene = bpy.data.scenes['Scene']

mesh = bpy.data.meshes.new('Basic_Sphere')

bm = bmesh.new()
bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, diameter=1)
bm.to_mesh(mesh)
bm.free()

basic_sphere = bpy.data.objects.new('Basic_Sphere', mesh)
scene.collection.objects.link(basic_sphere)

bpy.ops.export_scene.obj(filepath="data/basic_sphere.obj",
                         check_existing=False)

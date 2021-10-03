import math

import bpy
import bmesh

scene = bpy.data.scenes['Scene']
mesh = bpy.data.meshes.new('Basic_Cube')
bm = bmesh.new()

vertex_coords = [(math.sqrt(8./9.), 0., -1./3.),
                 (-math.sqrt(2./9.), math.sqrt(2./3.), -1./3.),
                 (-math.sqrt(2./9.), -math.sqrt(2./3.),-1./3.),
                 (0.,0.,1.)]
V = []
for coord in vertex_coords:
    V.append(bm.verts.new(coord))

F = []
face_indices = [(2,3,0), (0,3,1), (1,3,2), (0,1,2)]
for indices in face_indices:
    F.append(bm.faces.new((V[indices[0]], V[indices[1]], V[indices[2]])))

bm.to_mesh(mesh)
bm.free()

tetrahedron = bpy.data.objects.new('Tetrahedron', mesh)
scene.collection.objects.link(tetrahedron)

bpy.ops.export_scene.obj(filepath="data/tetrahedron.obj", 
                         check_existing=False)

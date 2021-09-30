import bpy
import bmesh

def get_cube_vertices(bm):
    coord_vals = (-0.5,0.5)
    vertices = []
    for x in coord_vals:
        for y in coord_vals:
            for z in coord_vals:
                new_vertex = bm.verts.new((x,y,z))
                vertices.append(new_vertex)
    return vertices

def get_cube_faces(vertices, bm):
    faces = []
    i_bit = (0,4)
    j_bit = (0,2)
    k_bit = (0,1)
    completed_edge_pairs = set()
    index_intervals_combinations = ((i_bit,j_bit,k_bit), 
                                    (i_bit,k_bit,j_bit), 
                                    (j_bit,k_bit,i_bit))
    for index_intervals in index_intervals_combinations:
        for a in index_intervals[2]:
            v1 = vertices[a+index_intervals[1][0]+index_intervals[0][0]]
            v2 = vertices[a+index_intervals[1][0]+index_intervals[0][1]]
            v3 = vertices[a+index_intervals[1][1]+index_intervals[0][0]]
            v4 = vertices[a+index_intervals[1][1]+index_intervals[0][1]]
            new_face = bm.faces.new((v1,v2,v4,v3))
            faces.append(new_face)
    return faces

def create_bmesh_cube():
    bm = bmesh.new()
    cube_vertices = get_cube_vertices(bm)
    faces = get_cube_faces(cube_vertices, bm)
    return bm

scene = bpy.data.scenes['Scene']

mesh = bpy.data.meshes.new('Basic_Cube')

bm = create_bmesh_cube()
print("Num faces: ", len(bm.faces))
bm.to_mesh(mesh)
bm.free()

basic_cube = bpy.data.objects.new('Basic_Cube', mesh)
scene.collection.objects.link(basic_cube)

bpy.ops.export_scene.obj(filepath="/home/mrincredible/temp/test_cube.obj", 
                         check_existing=False)

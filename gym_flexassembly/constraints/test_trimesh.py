import trimesh

mesh = trimesh.load_mesh('/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/3d/bend_wood_y_z.obj')

# mesh.is_watertight

# mesh.moment_inertia

# mesh.split()
# print(mesh.faces)

# mesh.show()

# unmerge so viewer doesn't smooth
mesh.unmerge_vertices()
# make mesh white- ish
for i in range(len(mesh.visual.face_colors)):
    mesh.visual.face_colors = [255,255,255,255]
    mesh.visual.face_colors[i] = [255, 0, 0, 255]

mesh.show()
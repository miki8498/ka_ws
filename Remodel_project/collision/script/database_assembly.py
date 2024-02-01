#!/usr/bin/env python

import pyvista as pv

gearbox_mesh = pv.read("/home/lar/ros/final_settings_ws/src/collision/collision_object/database_gearbox.stl")

#canalina1 parte dai componenti più a sinistra
canalina_1   = pv.read("/home/lar/ros/final_settings_ws/src/collision/collision_object/canaline/canalina1.stl")
canalina_2   = pv.read("/home/lar/ros/final_settings_ws/src/collision/collision_object/canaline/canalina2.stl")
canalina_3   = pv.read("/home/lar/ros/final_settings_ws/src/collision/collision_object/canaline/canalina3.stl")
canalina_4   = pv.read("/home/lar/ros/final_settings_ws/src/collision/collision_object/canaline/canalina4.stl")
canalina_5   = pv.read("/home/lar/ros/final_settings_ws/src/collision/collision_object/canaline/canalina_bucata.stl")


assembly_mesh = gearbox_mesh

canalina_1 = canalina_1.rotate_z(90)
canalina_1 = canalina_1.rotate_y(90)
canalina_1 = canalina_1.translate([-0.187, 0.170, 0.585 ])

assembly_mesh += canalina_1

canalina_2 = canalina_2.rotate_x(90)
canalina_2 = canalina_2.translate([-0.174, 0.170, 0.403 ])

assembly_mesh += canalina_2

canalina_3 = canalina_3.rotate_x(90)
canalina_3 = canalina_3.translate([-0.174, 0.170, 0.211])

assembly_mesh += canalina_3

canalina_4 = canalina_4.rotate_x(90)
canalina_4 = canalina_4.translate([-0.175, 0.170, 0.066])

assembly_mesh += canalina_4

canalina_5 = canalina_5.rotate_x(90)
canalina_5 = canalina_5.rotate_y(-90)
canalina_5 = canalina_5.translate([-0.20, 0.170, 0.573])

assembly_mesh += canalina_5

assembly_mesh.plot()


assembly_mesh.save("//home/lar/ros/final_settings_ws/src/collision/collision_object/database_assembly.stl", assembly_mesh)
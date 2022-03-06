# RBE-598-Haptics_Robotic_Interact

# Project Documentation

Project: Collision Detection System using Open3D

Purpose: To check for collision detection between two complex 3D object meshes by creating Axis Aligned Bounding Boxes around those objects and breaking them into smaller boxes by traversing through the octree

Description: The program consists of four function declarations and one class declaration:

 ## def is_colliding():
 To check for overlap collision between the AABB created
 ## def check_collision():
 To check for collision between the 3D objects
 ## def get_AABB_List():
 To fetch the Axis Aligned Bounding Box of each leaf node
 ## def get_leaf_node_list():
 To traverse through the octree and return a list of leaf nodes
 ## Class Traversal():
 Brute-force algorithm for traversing
FILES: Coll_Detect_BVH - laughing_buddha_right.h - contains the main program
Stanford_Bunny.ply – contains the ply version of Stanford Bunny file
happyStandRight_240.ply - contains the ply (stand right) version of Happy Buddha fil

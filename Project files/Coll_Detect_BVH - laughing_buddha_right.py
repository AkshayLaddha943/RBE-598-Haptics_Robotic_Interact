#simulation library
import open3d as o3d

#library for mathematical operation
import numpy as np

#To log start & end time of a certain process
import time

Leaf_node_list = []

#To check for overlap between both the bounding boxes by comparing their minimum and maximum values
def is_colliding(aabb1, aabb2):

    #retrieving min and max value of the bounding box1
    aabb1_min = aabb1.get_min_bound()
    aabb1_max = aabb1.get_max_bound()

    #retrieving min and max value of the bounding box2
    aabb2_min = aabb2.get_min_bound()
    aabb2_max = aabb2.get_max_bound()

    return ((aabb1_min[0] <= aabb2_max[0] and aabb1_max[0] >= aabb2_min[0]) and
            (aabb1_min[1] <= aabb2_max[1] and aabb1_max[1] >= aabb2_min[1]) and
            (aabb1_min[2] <= aabb2_max[2] and aabb1_max[2] >= aabb2_min[2]))


#checking collision between the 3D object meshes by extracting the leaf node list of the octree and creating an AABB around them
def check_collision(mesh1_pcd, mesh2_pcd):

    #declaring the leaf_node_list globally to use it outside the function
    global Leaf_node_list

    leaf_nodes_mesh1 = get_leaf_node_list(mesh1_pcd)
    Leaf_node_list = []
    leaf_nodes_mesh2 = get_leaf_node_list(mesh2_pcd)
    Leaf_node_list = []


    aabb_obj1 = get_AABB_List(leaf_nodes_mesh1, mesh1_pcd)
    aabb_obj2 = get_AABB_List(leaf_nodes_mesh2, mesh2_pcd)

    #to check if any of the points of the 3D objects lie within each other, if yes, return TRUE, else return FALSE
    for aabb1 in aabb_obj1:
        for aabb2 in aabb_obj2:
            state = is_colliding(aabb1, aabb2)
            if state:
                return state

    return False

#AABB is created for each leaf node and a list containing the points of the generated AABB is created
def get_AABB_List(leaf_node_list, lead_mesh_pcd):
    aabb_list = []

    #for every point in the leaf node, a 3D vector is generated which further creates an AABB and the respective object points
    for leaf in leaf_node_list:
        points = np.asarray(lead_mesh_pcd.points)[np.array(leaf)]
        vector_points = o3d.utility.Vector3dVector(points)
        aabb_vector = o3d.geometry.AxisAlignedBoundingBox().create_from_points(vector_points)
        aabb_list.append(aabb_vector)

    return aabb_list

#traversing through the octree and retrieving the leaf node list from every level
def get_leaf_node_list(mesh_pcd):

    #an Octree (with max_depth 3) is created and a variable is declared to traverse through the octree using the Brute Force Algorithm

    octree = o3d.geometry.Octree(max_depth=3)
    octree.convert_from_point_cloud(mesh_pcd, size_expand=0.01)

    octree_traverse = Traversal()

    octree.traverse(octree_traverse.f_traverse)
    len(Leaf_node_list)
    return Leaf_node_list

#Implementation of the Brute-Force Algorithm
class Traversal:
    @staticmethod
    def f_traverse(node, node_info):
        early_stop = False

        if isinstance(node, o3d.geometry.OctreeLeafNode):
            if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
                Leaf_node_list.append(node.indices)
        #                 print(node.indices)

        return early_stop


def main():

    #read and draw the Point cloud version of the respected 3D objecct models
    textured_mesh_pcd = o3d.io.read_point_cloud("Stanford_Bunny.ply")

    textured_mesh2_pcd = o3d.io.read_point_cloud("happyStandRight_240.ply")
    # textured_mesh2_pcd.scale(0.001, center=textured_mesh2_pcd.get_center())
    textured_mesh2_pcd.translate(np.array([0.01, 0, 0]), False)

    o3d.visualization.draw_geometries([textured_mesh_pcd])
    o3d.visualization.draw_geometries([textured_mesh2_pcd])

    #An octree is generated from the point clouds with max_depth = 5 and visualized

    octree = o3d.geometry.Octree(max_depth=5)
    octree.convert_from_point_cloud(textured_mesh_pcd, size_expand=0.01)
    o3d.visualization.draw_geometries([octree])

    octree2 = o3d.geometry.Octree(max_depth=5)
    octree2.convert_from_point_cloud(textured_mesh2_pcd, size_expand=0.01)
    o3d.visualization.draw_geometries([octree2])

    #For the main simulation,
     # - an instance of a visualizer is initiated
     # - A window is created with a width and height of 1024 and 768 px resp.
     # - Both the 3D objects are added to the simulation window

    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1024, height=768)
    vis.add_geometry(textured_mesh_pcd)
    vis.add_geometry(textured_mesh2_pcd)

    #A while loop is created which:
     # - start time is initiated
     # - check_collision fucntion is called
     # - total time for the fucntion to perform is calculated
     # - Translational motions of the objects are performed which is further updated in the visualizer
     # - rendering is updated and simulation exits after the stop button is manually pressed
    while True:
        start_time = time.time()
        print(check_collision(textured_mesh_pcd, textured_mesh2_pcd))
        print("Time taken: ", (time.time() - start_time))
        textured_mesh_pcd.translate(np.array([0, 0, 0]))
        textured_mesh2_pcd.translate(np.array([0, 0, 0.001]))
        vis.update_geometry(textured_mesh_pcd)
        vis.update_geometry(textured_mesh2_pcd)
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()



main()


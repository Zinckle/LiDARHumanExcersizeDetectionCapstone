
import velodyne_decoder as vd
import open3d as o3d
import pandas as pd
import numpy as np
from PIL import Image
import cv2 as cv
import math as math
import time


def map_to_int_range(float_list, target_min, target_max):
    # Find the minimum and maximum values in the input list
    min_val = min(float_list)
    max_val = max(float_list)

    # Map each float value to the target integer range and cast to int
    mapped_values = [int((value - min_val) / (max_val - min_val) * (target_max - target_min) + target_min)
                     for value in float_list]

    return mapped_values


def printPointcloudList(fullCloudArray):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(fullCloudArray[0][["x", "y", "z"]].values)
    # o3d.visualization.draw_geometries([geom])

    #R = geom.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
    #geom.rotate(R, center=(0, 0, 0))
    vis.add_geometry(geom)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
    # red = x
    # green = y
    # blue = z

    vis.get_view_control().set_front([-0.5, 0.4, 0])
    vis.get_view_control().set_up([0, 0, 1])
    vis.get_view_control().set_zoom(0.5)
    depthColours = False

    vis.add_geometry(mesh_frame)

    for item in fullCloudArray:

        if depthColours:
            # for generating colour based on distance
            distances = np.linalg.norm(item[["x", "y", "z"]].values, axis=1)
            min_distance = min(distances)
            max_distance = max(distances)
            colors = [
                [(d - min_distance) / (max_distance - min_distance), 0,
                 1 - (d - min_distance) / (max_distance - min_distance)]
                for d in distances]
            geom.colors = o3d.utility.Vector3dVector(colors)

        # set point values
        geom.points = o3d.utility.Vector3dVector(item[["x", "y", "z"]].values)

        degree = 20
        rotMat = [[np.cos(np.radians(degree)), -1 * np.sin(np.radians(degree)), 0],
                  [np.sin(np.radians(degree)), np.cos(np.radians(degree)), 0],
                  [0, 0, 1]]
        geom.rotate(rotMat, center=(0, 0, 0))

        # updated geometry and re render
        vis.update_geometry(geom)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.05)
        # input("click to continue")


def saveAsImages(fullCloudArray, width, height):
    j = 0
    for items in fullCloudArray:
        xs = map_to_int_range(items[["x"]].values, 0, width - 1)
        ys = map_to_int_range(items[["z"]].values, 0, height - 1)
        ps = map_to_int_range(items[["y"]].values, 0, 255)
        print(j)

        im = Image.new(mode="RGB", size=(width, height))
        for i in range(len(ps)):
            if ps[i] > 50:
                new_pixel_value = (0, ps[i], 0)  # Replace with your desired RGB color value
                im.putpixel((width - 1 - xs[i], height - 1 - ys[i]), new_pixel_value)
        im.save(str(j) + ".png")
        j += 1

def rotationForX(x, y):
    return x * np.cos(np.radians(45)) - y * np.sin(np.radians(45))

def rotationForY(x, y):
    return x * np.sin(np.radians(45)) + y * np.cos(np.radians(45))


# Iinitialization stuff
config = vd.Config(model='VLP-16', rpm=600, min_angle=0, max_angle=60)
pcap_file = 'Sitting-standing-sitting-Velodyne-VLP-16-Data.pcap'
cloud_arrays = []

# get the data out of the pcap
for stamp, points in vd.read_pcap(pcap_file, config):
    cloud_arrays.append(points)

# This block combines 2 packets to make a full view remove if full frame can fit in one packet
fullCloudArray = []
i = 0
# for i in range(0, len(cloud_arrays) - 1, 2):
# fullCloudArray.append(np.concatenate((cloud_arrays[i],cloud_arrays[i+1]), axis=0))
# fullCloudArray.append(pd.DataFrame(np.concatenate((cloud_arrays[i], cloud_arrays[i + 1]), axis=0),
# columns=["x", "y", "z", "Intensity", "Ring", "Time"]))
# for 60deg input
for i in range(0, len(cloud_arrays)):
    # fullCloudArray.append(np.concatenate((cloud_arrays[i],cloud_arrays[i+1]), axis=0))
    fullCloudArray.append(pd.DataFrame(cloud_arrays[i][0:int(len(cloud_arrays[i]) / 3)],
                                       columns=["x", "y", "z", "Intensity", "Ring", "Time"]))

    fullCloudArray.append(
        pd.DataFrame(cloud_arrays[i][int(len(cloud_arrays[i]) / 3) + 1:int(len(cloud_arrays[i]) / 3) * 2],
                     columns=["x", "y", "z", "Intensity", "Ring", "Time"]))

    fullCloudArray.append(pd.DataFrame(cloud_arrays[i][int(len(cloud_arrays[i]) / 3) * 2 + 1:int(len(cloud_arrays[i]))],
                                       columns=["x", "y", "z", "Intensity", "Ring", "Time"]))

print("number of frames = ", i + 1 * 3)

#saveAsImages(fullCloudArray[0:1], 400, 200)
#printPointcloudList(fullCloudArray)

#for j in range(len(fullCloudArray)):#
#
#    fullCloudArray[j]['x'] = fullCloudArray[j].apply(lambda row: rotationForX(row['x'], row['y']), axis=1)
#    fullCloudArray[j]['y'] = fullCloudArray[j].apply(lambda row: rotationForY(row['x'], row['y']), axis=1)
#    print(j)

printPointcloudList(fullCloudArray)


#saveAsImages(fullCloudArray, 500, 200)

print("hi")


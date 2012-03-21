
import math
import sys
import pickle
import os
import struct 
import ctypes
import time 
import random 
import bpy
from mathutils import Vector, Euler, Matrix

from blensor import evd
import blensor


parameters = { "el_start": 60.0,
               "el_end": 140.0,
               "el_resolution": 0.1,
               "az_start": 0.0,
               "az_end": 360.0,
               "az_resolution": 0.1,
               "max_dist": 1500.0,
               "reflectivity_distance":50,
               "reflectivity_limit":0.1,
               "reflectivity_slope":0.01,
             }
              

def addProperties(cType):
    global parameters
    
    cType.rs_el_resolution = bpy.props.FloatProperty(name = "Resolution", default=parameters["el_resolution"], min=0.01, max=360, description="How far two scan lines are apart in elevation angle (degrees)")
    cType.rs_el_start = bpy.props.FloatProperty(name = "Start", default=parameters["el_start"], description="Angle to start at in elevation (degrees)")
    cType.rs_el_end = bpy.props.FloatProperty(name = "End", default=parameters["el_end"], description="Angle to end at in elevation (degrees)")
    
    cType.rs_az_resolution = bpy.props.FloatProperty(name = "Resolution", default=parameters["az_resolution"], min=0.01, max=360, description="How far two scan lines are apart in azimuth angle (degrees)")
    cType.rs_az_start = bpy.props.FloatProperty(name = "Start", default=parameters["az_start"], description="Angle to start at in azimuth (degrees)")
    cType.rs_az_end = bpy.props.FloatProperty(name = "End", default=parameters["az_end"], description="Angle to end at in azimuth (degrees)")
    
    cType.rs_max_dist = bpy.props.FloatProperty( name = "Scan distance", default = parameters["max_dist"], min=0, max=2000, description = "How far the laser can see" )
 
    cType.velodyne_ref_dist = bpy.props.FloatProperty( name = "Reflectivity Distance", default = parameters["reflectivity_distance"], description = "Objects closer than reflectivity distance are independent of their reflectivity" )
    cType.velodyne_ref_limit = bpy.props.FloatProperty( name = "Reflectivity Limit", default = parameters["reflectivity_limit"], description = "Minimum reflectivity for objects at the reflectivity distance" )
    cType.velodyne_ref_slope = bpy.props.FloatProperty( name = "Reflectivity Slope", default = parameters["reflectivity_slope"], description = "Slope of the reflectivity limit curve" ) 
 

def deg2rad(deg):
    return deg*math.pi/180.0

def rad2deg(rad):
    return rad*180.0/math.pi

def tuples_to_list(tuples):
    l = []
    for t in tuples:
        l.extend(t)
    return l



def scan_advanced(sensor, evd_file=None):
    start_time = time.time()
    
    if sensor.local_coordinates:
        world_transformation = Matrix()
    else:
        world_transformation = sensor.matrix_world
   
    evd_storage = evd.evd_file(evd_file)

    rays = []
    ray_info = []

    n_az = (sensor.rs_az_end - sensor.rs_az_start) / sensor.rs_az_resolution
    n_el = (sensor.rs_el_end - sensor.rs_el_start) / sensor.rs_el_resolution
    
    print('n_az', 'n_el', n_az, n_el)    
    
    for i_az in range(int(n_az)):
        az_angle = (1e-6 + sensor.rs_az_start + float(i_az) * sensor.rs_az_resolution + 180.0) % 360.0
        
        for i_el in range(int(n_el)):
            el_angle = (1e-6 + sensor.rs_el_start + float(i_el) * sensor.rs_el_resolution + 180.0) % 360.0

            ray = Vector([0, 0, sensor.rs_max_dist])            
            timestamp = 0.0
            ray_info.append([deg2rad(az_angle), deg2rad(el_angle), timestamp])
            
            rotator = Euler([deg2rad(el_angle), deg2rad(az_angle), 0.0])
            ray.rotate(rotator)
            rays.extend([ray[0],ray[1],ray[2]])
            

    returns = blensor.scan_interface.scan_rays(rays, sensor.rs_max_dist)
    verts = []
    verts_noise = []

    for i in range(len(returns)):
        idx = returns[i][-1]
        vt = (world_transformation * Vector((returns[i][1], returns[i][2], returns[i][3],1.0))).xyz
        v = [returns[i][1], returns[i][2], returns[i][3]]
        verts.append(vt)

        distance_noise = 0
        vector_length = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        norm_vector = [v[0]/vector_length, v[1]/vector_length, v[2]/vector_length]
        vector_length_noise = vector_length + distance_noise
        v_noise = (world_transformation * Vector((norm_vector[0]*vector_length_noise, norm_vector[1]*vector_length_noise, norm_vector[2]*vector_length_noise,1.0))).xyz
        verts_noise.append(v_noise)

        evd_storage.addEntry(timestamp = ray_info[idx][2], yaw =(ray_info[idx][0]+math.pi)%(2*math.pi), pitch=ray_info[idx][1], distance=vector_length, distance_noise=vector_length_noise, x=vt[0], y=vt[1], z=vt[2], x_noise=v_noise[0], y_noise=v_noise[1], z_noise=v_noise[2], object_id=returns[i][4], color=returns[i][5])
        
    if evd_file:
        evd_storage.appendEvdFile()

    if sensor.add_scan_mesh:
        scan_mesh = bpy.data.meshes.new("scan_mesh")
        scan_mesh.vertices.add(len(verts))
        scan_mesh.vertices.foreach_set("co", tuples_to_list(verts))
        scan_mesh.update()
        scan_mesh_object = bpy.data.objects.new("Scan.{0}".format(bpy.context.scene.frame_current), scan_mesh)
        bpy.context.scene.objects.link(scan_mesh_object)
        blensor.show_in_frame(scan_mesh_object, bpy.context.scene.frame_current)

        if world_transformation == Matrix():
            scan_mesh_object.matrix_world = bpy.context.object.matrix_world

    if sensor.add_noise_scan_mesh:
        noise_scan_mesh = bpy.data.meshes.new("noisy_scan_mesh")
        noise_scan_mesh.vertices.add(len(verts_noise))
        noise_scan_mesh.vertices.foreach_set("co", tuples_to_list(verts_noise))
        noise_scan_mesh.update()
        noise_scan_mesh_object = bpy.data.objects.new("NoisyScan.{0}".format(bpy.context.scene.frame_current), noise_scan_mesh)
        bpy.context.scene.objects.link(noise_scan_mesh_object)
        blensor.show_in_frame(noise_scan_mesh_object, bpy.context.scene.frame_current)
    
        if world_transformation == Matrix():
            noise_scan_mesh_object.matrix_world = bpy.context.object.matrix_world

    bpy.context.scene.update()
    
    end_time = time.time()
    scan_time = end_time - start_time
    print ("Elapsed time: %.3f" % (scan_time))

    return True, 0, scan_time



## This Function creates scans over a range of frames
#
#def scan_range(frame_start, frame_end, filename="/tmp/landscape.evd", frame_time = (1.0/24.0), rotation_speed = 10.0, add_blender_mesh=False, add_noisy_blender_mesh=False, angle_resolution = 0.1728, max_distance = 120.0, noise_mu = 0.0, noise_sigma= 0.02, last_frame = True, world_transformation=Matrix()):
#    start_time = time.time()
#
#    angle_per_second = 360.0 * rotation_speed
#    angle_per_frame = angle_per_second * frame_time
#
#    try:
#        for i in range(frame_start,frame_end):
#                bpy.context.scene.frame_current = i
#
#                ok,start_radians,scan_time = scan_advanced(rotation_speed=rotation_speed, angle_resolution = angle_resolution, start_angle = float(i)*angle_per_frame, end_angle=float(i+1)*angle_per_frame, evd_file = filename, evd_last_scan=False,add_blender_mesh=add_blender_mesh, add_noisy_blender_mesh=add_noisy_blender_mesh, frame_time=frame_time, simulation_time = float(i)*frame_time, max_distance=max_distance, noise_mu = noise_mu, noise_sigma=noise_sigma, world_transformation=world_transformation)
#
#                if not ok:
#                    break
#    except:
#        print ("Scan aborted")
#
#
#    if last_frame:
#        #TODO: move this into the evd module
#        evd_file = open(filename,"a")
#        evd_file.buffer.write(struct.pack("i",-1))
#        evd_file.close()
#
#
#    end_time = time.time()
#    print ("Total scan time: %.2f"%(end_time-start_time))
#
#
#
#
#

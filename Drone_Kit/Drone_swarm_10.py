#! /usr/bin/env python

#   3D
#  Dr. Bilal Kadri's Navigation code for iris 
#   modified by Ahfaz Ahmed for SkywalkerX8 swarm

# this piece of code use artificial potential field to successfully navigate swarm of drones
#  to goal while avoiding the to obstacles

# it impliments same charge on uavs to form trianglr formation  and virtual leader is on the center which attracts
# forces are balance on the edges of the trianglr formation but it have 100 percent brobablity of failiur as any one uav fails


from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import numpy as np
import imutils
from collections import deque
from std_msgs.msg import Float32MultiArray
import sys
import time
import rospy
import logging

#added for APF
import random
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d                   #3D plot
import array as arr
from datetime import datetime

from pyquaternion import Quaternion

#global vehicle
vehicle = []

n = 10

Simul_iterations = 1200

connection_string = ['127.0.0.1:14551','127.0.0.1:14561','127.0.0.1:14571','127.0.0.1:14581','127.0.0.1:14591','127.0.0.1:14601','127.0.0.1:14611','127.0.0.1:14621','127.0.0.1:14631','127.0.0.1:14641']


takeoff_alt = 25

cycle_time = 0.25 #sec

#obs_r=   [40,30,30,20,10,60,10,40,30,40]

#normal simulation
obs_r   = [40,60,30,20,10,70,10,120,30,40]
obs_loc = [[300,350,75],[200,100,55],[130,300,55],[220,500,55],[150,450,55],[760,800,45],[900,500,55],[600,380,65],[450,700,55],[800,200,85] ] # 8 was at 540,500
goal    = [1000,1000, 50]   # home      goal Y   X

#concave simulation
#obs_r   = [20,20,20,20,20,20,20,20,20,20]
#obs_loc = [[610,430,50],[607,480,50],[605,527.5,50],[597.5,572,50],[585,617,50],[560,655,50],[530,690,50],[492.5,710,50],[450,717.5,50],[800,200,50] ]  
#obs_loc = [[800,100,50],[200,800,50],[700,100,50],[597.5,497,50],[585,542,50],[560,580,50],[540,625,50],[515,660,50],[600,450,50],[800,200,50] ]  

goal    = [1000,1000, 45]    # 45 for climbing 25 for same altitude

#d = 180            #60
r = 40             # radius of goal



ro   = 100           # PpAPF    area of influence around the boundary  80
ro_height = 30       # PpAPF     upto which climb force is applied
Krep = 550e6         # PpAPF  repulsive gain 50 -> 80 -> 500    500000 1600000
Katt =  10500        # PpAPF  50          # 50 to 500
ro_climb = 10        # PpAPF  previously 20m


Katt_lead = 10        #FcAPF
Krep_lead = 22.1695   #FcAPF
R_sphere  = 5         #FcAPF  2
q         = 35        #FcAPF prev 5

#q_red    = 2
#q_green  = 2
#q_blue   = 2

Z_offset = 25 #9.6 #degrees

############################### add goal #############################
"""
  Args:
    X =  position of UAV X-axis 
    Y =  position of UAV Y-axis 
    r = goal size
    loc = goal location
  Return :
    delx and dely
  
  This function is to add the goal and its potential field on the graph.
  Katt = 50
"""
def add_goal (X, Y, Z, r, loc_g):     #goal location r radis of goal

  #delx = np.zeros_like(X)
  #dely = np.zeros_like(Y)

  #for i in range(len(x)):
    #for j in range(len(y)):
      
  dist_xy= np.sqrt((loc_g[0]-X)**2 + (loc_g[1]-Y)**2)
  dist_xz= np.sqrt((loc_g[0]-X)**2 + (loc_g[2]-Z)**2)
  dist_z = abs(loc_g[2]-Z)

  # try mod of |x-xg|
  #theta = np.arctan2(loc_g[1]-Y[i][j], loc_g[0] - X[i][j])

  if dist_xy < r:                    #goal threshold
    delx  = 0
    dely  = 0
    delz  = 0    
#  elif dist<=d:
#    delx = -Katt * (X - loc_g[0])    #-Katt *x-xg
#    dely = -Katt * (Y - loc_g[1])    #-Katt *y-yg
#    delz = -Katt * (Z - loc_g[2])    #-Katt *z-zg
  else:
    delx = -Katt  * (X - loc_g[0])/dist_xy    # * d #Katt *d *(x-xg)/dist
    dely = -Katt  * (Y - loc_g[1])/dist_xy    # * d #Katt *d *(y-yg)/dist
    delz = -Katt  * (Z - loc_g[2])/dist_xy * 10 #* abs(Z - loc_g[2])#*dist_z/dist_xy * 5 # * d #* 20  #/dist_z   #Katt *d *(y-yg)/dist   # need revisit here


  return delx, dely, delz



############################### add obstacle #############################
"""
  Args:
    X =  position of UAV X-axis 
    Y =  position of UAV Y-axis 
    delx = Usual meaning
    dely = Usual Meaninig
    goal = location of the goal 
  Return:
    delx = Usual meaning
    dely = Usual Meaninig
    obstacle = location of the obstacle 
    r = size of the obstacle 
  This function first generate the oobstacle with diameter ranging from 1 to 5 i.e. radius 
  ranging from 0.5 to 2.5 randomly. Then it generate location of the obstacle randomly.
  Then inside the nested loop, distance from each point to the goal and ostacle is 
  calculated, Similarly angles are calculated. Then I simply used the formula give and 
  superimposed it to the Goals potential field.Also
  alpha = 50
  beta = 120
  s = 70
  
"""

def add_obstacle(X, Y, Z , delx, dely, delz, goal,obs_number):


  # generating obstacle with random sizes
  #r = random.randint(1,5)                   # radius of obstacle from 5 to 25

  r = obs_r[obs_number]

  # generating random location of the obstacle [100,100   950,950]
  #obstacle =  random.sample(range(1, 45), 2)
  obstacle = [obs_loc[obs_number][0],obs_loc[obs_number][1],obs_loc[obs_number][2]] 

  climb_height_thrshold = obstacle[2]  -  ro_climb       # obst height - climbing height



  #for i in range(len(x)):
    #for j in range(len(y)):

  xy_dist_obstacle = np.sqrt((obstacle[0]-X)**2 + (obstacle[1]-Y)**2)
  z_above_climb  = Z - climb_height_thrshold

  ro_xy = xy_dist_obstacle - r   #min dist from obstacle boundary
  ro_z  = Z - obstacle[2] 

  phi_obstacle  = np.arctan2(obstacle[1] - Y, obstacle[0]  - X)
  zeta_obstacle = 90 / 57.2958 * min( 1, ( z_above_climb / ro_climb ))   # saturate at 90 above height


  if xy_dist_obstacle <= r and Z <= obstacle[2]:        #obstacle physical area-> just to avoid infinity value inside 

    if  Z < climb_height_thrshold:        # xy disk repulsion              
        delx_temp = -35000 * np.cos(phi_obstacle)  # angle should be added here towards out
        dely_temp = -35000 * np.sin(phi_obstacle)  # old 5000
        delz_temp =  0
    else: 
        delx_temp = -35000 * np.cos(phi_obstacle) * np.cos(zeta_obstacle)  # angle should be added here towards out
        dely_temp = -35000 * np.sin(phi_obstacle) * np.cos(zeta_obstacle)  # old 5000
        delz_temp = -35000 * np.sin(zeta_obstacle) * -1
      
    
  elif ro_xy < ro and Z < (obstacle[2] + ro):          #obstacle influence area
    #gives unit vector in the direction of field
    del_rox =  (X - obstacle[0])/xy_dist_obstacle    # = (x-b0)/|x-b| used~ (x-obs0)/|x-obs|
    del_roy =  (Y - obstacle[1])/xy_dist_obstacle    # = (y-b1)/|x-b| used~ (y-obs1)/|x-obs|S
    del_roz =  (Z - obstacle[2])/ abs(Z - obstacle[2])

    if  Z < climb_height_thrshold :  
        delx_temp =  Krep * (1/ro_xy - 1/ro)  * del_rox/ro_xy**2   #* 2 * (-X + goal[0] )added due to paper
        dely_temp =  Krep * (1/ro_xy - 1/ro)  * del_roy/ro_xy**2   #* 2 * (-Y + goal[1] )
        delz_temp =  0
        #print( "obs ",obs_number+1,"Z < cl_thrs","roxy",ro_xy,delx_temp,dely_temp,delz_temp,"delrox",del_rox,"delroy",del_roy)

    elif  ro_xy > 0 and Z < obstacle[2] + ro_height:   #above climb to top outside r
        delx_temp =  0 #Krep * (1/ro_xy - 1/ro)  * del_rox/ro_xy**2  * np.cos(zeta_obstacle)              #  * 2 * (-X + goal[0] ) 
        dely_temp =  0 #Krep * (1/ro_xy - 1/ro)  * del_roy/ro_xy**2  * np.cos(zeta_obstacle)              #  * 2 * (-Y + goal[1] ) 
        delz_temp = 15000 #np.sqrt( (delx_temp * np.tan(zeta_obstacle))**2  +   (dely_temp * np.tan(zeta_obstacle))**2  )              # sin = tan / cos ,  Krep * (1/ro_z  - 1/ro)  * del_roz/ro_z**2   * 2 * np.sin(zeta_obstacle)  #  * (-Z + goal[2] )
        #print( "obs ",obs_number+1,"ro_xy > 0","roxy",ro_xy,delx_temp,dely_temp,delz_temp,"delrox",del_rox,"delroy",del_roy)

    else:                   # above obstacle in r region
        delx_temp =  0 
        dely_temp =  0 
        delz_temp =  15000 #Krep * (1/ro_z  - 1/ro)  * del_roz/ro_z**2   * 2 * np.sin(zeta_obstacle)  #  * (-Z + goal[2] )
        #print( "obs ",obs_number+1,"else","roxy",ro_xy,delx_temp,dely_temp,delz_temp,"delrox",del_rox,"delroy",del_roy)


    #print( "delx_temp",delx_temp,"dely_temp",dely_temp)

  else :
    delx_temp = 0
    dely_temp = 0
    delz_temp = 0


  delx +=  delx_temp
  dely +=  dely_temp
  delz +=  delz_temp 


  if   delx > 0 :                                # to prevent inf values
    delx = min(delx, 35000)
  elif delx < 0 :
    delx = max(delx,-35000)
  else:
    delx = 0

  if   dely > 0 :
    dely = min(dely, 35000)
  elif dely < 0 :
    dely = max(dely,-35000)
  else:
    dely = 0


  if   delz > 0 :
    delz = min(delz, 35000)
  elif delz < 0 :
    delz = max(delz,-35000)
  else:
    delz = 0

  #print( "xy_dist_obstacle",round(xy_dist_obstacle,1),"z_above_climb",round(z_above_climb,1),"delx_temp",round(delx_temp,1),"dely_temp",round(dely_temp,1),"delz_temp",round(delz_temp,1),"phi_obstacle",round(phi_obstacle*57.2958,1),"zeta_obstacle",round(zeta_obstacle*57.2958,1))
  
  return delx, dely, delz


############################### plot graph #############################

def plot_graph(obj, fig, ax, loc,r, color ):
  
  
  ax.add_patch(plt.Circle(loc, r, color=color))
  ax.add_patch(plt.Circle(loc, (r + ro), linestyle=':',fill = False ))

  ax.set_title('Robot path with {0} obstacles '.format(i))    #f
  ax.annotate(obj, xy=loc, fontsize=10, ha="center")
  return ax


############################### set_destination #############################

def set_destination(Plane ,lat, lon, alt, wp_index):

    #global vehicle

    print("Moving to Waypoint {0}".format(wp_index))
    
    aLocation = LocationGlobalRelative(lat, lon, float(alt))
    
    Plane.simple_goto(aLocation)
    dist_to_wp = dist_between_global_coordinates(Plane.location.global_frame, aLocation) 
    
    while dist_to_wp > 50:
        print("Distance to Waypoint {0}: {1}".format(wp_index, dist_to_wp))
        dist_to_wp = dist_between_global_coordinates(Plane.location.global_frame, aLocation) 
    
    print("Distance to Waypoint {0}: {1}".format(wp_index, dist_to_wp))
    print("Reached Waypoint {0}".format(wp_index))


    time.sleep(1)

############################### get_location_metres #############################
# get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.

def get_location_metres(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,alt)              #original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,alt)            #original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

############################### get_distance_metres #############################
#get_distance_metres - Get the distance between two LocationGlobal objects in metres

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


############################### get_Bearing #############################
#get_bearing - Get the bearing in degrees to a LocationGlobal

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

############################### dist_between_global_coordinates #############################

def dist_between_global_coordinates(aLocation1, aLocation2):
    # This formula has been copied from HEIFU project's repository. Link: https://gitlab.pdmfc.com/drones/ros1/heifu/-/blob/master/heifu_interface/formulas.py
    R = 6371e3; #in meters radius of earth
    latitude1 = math.radians(aLocation1.lat)
    latitude2 = math.radians(aLocation2.lat)
    latitudevariation = math.radians((aLocation2.lat-aLocation1.lat))
    longitudevariation = math.radians((aLocation2.lon-aLocation1.lon))
    a = math.sin(latitudevariation/2) * math.sin(latitudevariation/2) + math.cos(latitude1) * math.cos(latitude2) * math.sin(longitudevariation/2) * math.sin(longitudevariation/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c #in meters
    return distance

############################### arm_and_takeoff #############################

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready

    for loop in range(n): 

        while not vehicle[loop].is_armable:
            print(" Waiting for vehicle%s to initialise..."%loop+1)
            time.sleep(1)



    # Copter should arm in GUIDED mode
    #vehicle.mode = VehicleMode("GUIDED")

    print("Arming UAV's")

    for loop in range(n): 
        vehicle[loop].armed  = True

    break_flag = False

    while True: 
        for loop in range(n):
            if vehicle[loop].armed == True:
                break_flag = True
            else:
                break_flag = False
        print(" Waiting for arming...")

        if break_flag == True:
            break

        time.sleep(1)


    print("Taking off!")

    for loop in range(n): 
        vehicle[loop].mode   = VehicleMode("TAKEOFF")
        vehicle[loop].simple_takeoff(aTargetAltitude)    # Take off to target altitude


    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).

    while True:
        print(" Altitude: ", vehicle[0].location.global_relative_frame.alt)      
        if vehicle[0].location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

############################### goto 2D #############################
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """

def goto2D(Plane , dEast, dNorth):     #step, angle ):    #, gotoFunction = vehicle.simple_goto):   #east = x north = y


# added 200 m offset to avoid way point loiter curve

    desire_angle = np.arctan2(dNorth,dEast)

    Del   = math.sqrt( dEast**2 + dNorth**2)

    Plane.groundspeed   = Del / cycle_time

    N_offset =  dNorth + 200 * math.sin(desire_angle)
    E_offset =  dEast  + 200 * math.cos(desire_angle)


    currentLocation=Plane.location.global_relative_frame
    targetLocation=get_location_metres(currentLocation, dNorth, dEast, takeoff_alt)

    targetLocation_offset=get_location_metres(currentLocation, N_offset,  E_offset, takeoff_alt)
#   targetDistance=get_distance_metres(currentLocation, targetLocation)
    #gotoFunction(targetLocation)
    Plane.simple_goto(targetLocation_offset)

    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance
    
    #while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
    #    remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
    #    print ("Distance to target: ", remainingDistance)
    #   if remainingDistance<=targetDistance*0.15: #Just below target, in case of undershoot.
    #        print ("Reached target")
    #        break;
    #    time.sleep(2)
    return targetLocation , Del , desire_angle

###########################################################
############################### goto 3D #############################
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
    """

def goto3D(Plane , dEast, dNorth, dDown):     # x y z  we sent Ddwn +ve  step, angle ):    #, gotoFunction = vehicle.simple_goto):   #east = x north = y


# added 200 m offset to avoid way point loiter curve


    Delxy   = math.sqrt( dEast**2 + dNorth**2)
    Delz    = dDown 

    Del     = math.sqrt( Delxy**2 + Delz**2)

    desire_angle_xy = np.arctan2(dNorth,dEast)          #(y,x)
    desire_angle_z  = np.arctan2(Delz,Delxy)


    Plane.groundspeed   = Del / cycle_time

    N_offset =  dNorth + 200 * math.sin(desire_angle_xy) * math.cos(desire_angle_z)
    E_offset =  dEast  + 200 * math.cos(desire_angle_xy) * math.cos(desire_angle_z)
    D_offset =  dDown  + 200 * math.sin(desire_angle_z)


    currentLocation=Plane.location.global_relative_frame
    targetLocation=get_location_metres(currentLocation, dNorth, dEast, dDown)

    targetLocation_offset=get_location_metres(currentLocation, N_offset,  E_offset, D_offset)

    Plane.simple_goto(targetLocation_offset)

    return targetLocation , Del , desire_angle_xy, desire_angle_z

###########################################################

def set_servo(vehicle, servo_number, pwm_value):
    pwm_value_int = int(pwm_value)
    msg = vehicle.message_factory.command_long_encode(
        0, 0, 
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm_value_int,
        0,0,0,0,0
        )
    vehicle.send_mavlink(msg)

###########################################################    

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle1_red.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle1_red.send_mavlink(msg)
        print(x)
        time.sleep(1)

###########################################################   

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw] 

  ###########################################################   

def send_set_attitude_target(roll=0, pitch=0, yaw=0, thrust=0.5):
    '''
    1   ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE  Ignore body roll rate
    2   ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE Ignore body pitch rate
    4   ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE   Ignore body yaw rate
    32  ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET    Use 3D body thrust setpoint instead of throttle
    64  ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE    Ignore throttle
    128 ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE    Ignore attitude   


    '''


    #attitude = [np.radians(roll), np.radians(pitch), np.radians(yaw)]
    #attitude_quaternion = Quaternion.QuaternionBase(attitude)
    #attitude_quaternion = get_quaternion_from_euler(np.radians(roll), np.radians(pitch), np.radians(yaw))
    attitude_quaternion = to_quaternion(np.radians(roll), np.radians(pitch), np.radians(yaw))

    msg = vehicle1_red.message_factory.set_attitude_target_encode(
        0, 0, 0,  # time_boot_ms, target_system, target_component
        0b10111000,  # type_mask https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET_TYPEMASK
        #0b00000100,
        attitude_quaternion,  # Attitude quaternion
        0, 0, 0,  # Rotation rates (ignored)
        thrust  # Between 0.0 and 1.0
    )

    vehicle1_red.send_mavlink(msg)

    ###########################################


def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    '''
    Convert degrees to quaternions
    '''
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

##################################################################
############################### MAIN #############################
##################################################################

if __name__ == '__main__':


#   rows, cols = (5, 5)
#   arr = [[0 for i in range(cols)] for j in range(rows)]
#    arr = [[0 for i in range(Simul_iterations)] for j in range(n)]
#    print(arr)
#    arr[2][598] = 55
#    print(arr)

#    rows, cols = (5, 5)
#   arr = [[0]*cols]*rows
#    print(arr)


    for loop in range(n):               
        print(' Connecting to vehicle on: %s' % connection_string[loop])
        vehicle.append(connect(connection_string[loop], wait_ready=True, baud=921600))
        print("Connection to vehicle{0} Successfully Established!".format(loop+1))  



############?????????????/ to look here for actual home position??????????????????/

    Home_Location_L     = get_location_metres( vehicle[0].location.global_frame,0,2,0)
    print("global Home leader: {0}".format(Home_Location_L))

    goal_global = get_location_metres(Home_Location_L,goal[0],goal[1],goal[2])    #original_location, dNorth, dEast):
    print("Goal",goal_global)

    print("Vehicles Z ",vehicle[1].location.local_frame.down * -1)



    print("Starting Takeoff")
    arm_and_takeoff(takeoff_alt)
    #time.sleep(1)


    print("mode GUIDED")

    for loop in range(n):  
        vehicle[loop].mode   = VehicleMode("GUIDED")


    time.sleep(0.5) 

    return_point = get_location_metres(LocationGlobal(Home_Location_L.lat, Home_Location_L.lon, 25),40,-30,55)

    for loop in range(n):  
        vehicle[loop].simple_goto(return_point)    


    while get_distance_metres(vehicle[0].location.global_frame, Home_Location_L) > 20:
        time.sleep(0.5) 


    for loop in range(n):      
        vehicle[loop].groundspeed   = 15#14.5


    Lambda          =    15   #15 14.5  16
    Gamma           =    4    #3.5 3.5   2

    step_size_L     = Lambda * cycle_time     #7.5   #15  #15/5      #meter
    step_size_F     = Gamma  * cycle_time


    APFx            = [0]*Simul_iterations
    APFy            = [0]*Simul_iterations
    APFz            = [0]*Simul_iterations

    PosX_L          = [0]*Simul_iterations
    PosY_L          = [0]*Simul_iterations
    PosZ_L          = [0]*Simul_iterations

    TargetX_L       = [0]*Simul_iterations
    TargetY_L       = [0]*Simul_iterations
    TargetZ_L       = [0]*Simul_iterations

    Fattx       = [0]*n
    Fatty       = [0]*n
    Fattz       = [0]*n
    Frepx       = [0]*n
    Frepy       = [0]*n
    Frepz       = [0]*n

    theta_Force_xy    = [0]*n
    force_xy          = [0]*n
    force_z           = [0]*n
    force             = [0]*n
    theta_Force_z     = [0]*n

    DelX_t            = [0]*n
    DelY_t            = [0]*n
    DelZ_t            = [0]*n
    DelX              = [0]*n 
    DelY              = [0]*n
    DelZ              = [0]*n

    Target            = [0]*n
    Del               = [0]*n
    des_Ang_xy        = [0]*n
    des_Ang_z         = [0]*n



    Rij         = [[0 for i in range(n)] for j in range(n)]
    cos_phi     = [[0 for i in range(n)] for j in range(n)]
    sin_phi     = [[0 for i in range(n)] for j in range(n)]
    cos_zeta    = [[0 for i in range(n)] for j in range(n)]
    sin_zeta    = [[0 for i in range(n)] for j in range(n)]


    TargetX     = [[0 for i in range(Simul_iterations)] for j in range(n)]
    TargetY     = [[0 for i in range(Simul_iterations)] for j in range(n)]
    TargetZ     = [[0 for i in range(Simul_iterations)] for j in range(n)]


    PosX        = [[0 for i in range(Simul_iterations)] for j in range(n)]
    PosY        = [[0 for i in range(Simul_iterations)] for j in range(n)]
    PosZ        = [[0 for i in range(Simul_iterations)] for j in range(n)]

    Fx          = [[0 for i in range(Simul_iterations)] for j in range(n)]   
    Fy          = [[0 for i in range(Simul_iterations)] for j in range(n)]  
    Fz          = [[0 for i in range(Simul_iterations)] for j in range(n)]


    now = datetime.now()

    #toDate = today.strftime("%b_%d_%Y")
    dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")

    f = open("Test_Data/Test_{0}.txt".format(dt_string), "a")



    f.write("%Time i Dist_rem PosX_L PosY_L PosZ_L TargetX_L TargetY_L TargetZ_L APFx APFy APFz DelX_L DelY_L DelZ_L ")
#    f.write("force_L force_L_xy force_L_z theta_Force_L_xy theta_Force_L_z ")

#PosX_red PosX_green PosX_blue PosY_red PosY_green PosY_blue PosZ_red PosZ_green PosZ_blue  TargetX_red TargetX_green TargetX_blue TargetY_red TargetY_green TargetY_blue TargetZ_red TargetZ_green TargetZ_blue

    for loop in range(n):
        f.write("PosX{0} PosY{0} PosZ{0} DelX{0} DelY{0} DelZ{0} ".format(loop+1))   
        #f.write("DelX{0} DelY{0} DelZ{0} DelXt_{0} DelYt_{0} DelZt_{0} Fx_{0} Fy_{0} Fz_{0} force_{0} force_xy_{0} force_z_{0} theta_Force_xy_{0} theta_Force_z_{0} ".format(loop+1))

    f.write("\n")


    print("Simulation start")
    i   = 0
    #c_i = 0

    while i < Simul_iterations:   #800  1000:

        print (i)
        # ................inputs.................

        for loop in range(n):
            PosY[loop][i]                   = vehicle[loop].location.local_frame.north
            PosX[loop][i]                   = vehicle[loop].location.local_frame.east
            PosZ[loop][i]                   = vehicle[loop].location.local_frame.down * -1
            #Current_Location[loop]          = vehicle[loop].location.global_frame     #current position red
            #Current_Heading[loop]           = vehicle[loop].heading
            #Current_Speed[loop]             = vehicle[loop].groundspeed       #airspeed  
            print("X{0}= {1} Y{0}= {2} Z{0}= {3} ".format(loop+1, PosX[loop][i], PosY[loop][i], PosZ[loop][i]))  


        #leader
        #calculate current virtual leaders position   # z will be used in later stage 
        PosX_L_sum = 0
        PosY_L_sum = 0
        PosZ_L_sum = 0

        for loop in range(n):
            PosX_L_sum = PosX_L_sum + PosX[loop][i]
            PosY_L_sum = PosY_L_sum + PosY[loop][i]
            PosZ_L_sum = PosZ_L_sum + PosZ[loop][i]
            #print("{3} x= {0} y= {1}   z= {2} \n ".format(PosX[loop][i],PosY[loop][i],PosZ[loop][i],loop))


        PosX_L[i] = PosX_L_sum/n
        PosY_L[i] = PosY_L_sum/n
        PosZ_L[i] = PosZ_L_sum/n

        print("LX= {0} LY= {1}   LZ= {2} ".format(PosX_L[i],PosY_L[i],PosZ_L[i]))

        currentLocation_L = get_location_metres(Home_Location_L, PosY_L[i], PosX_L[i],PosZ_L[i])
        #print("currentLocation_L ")

        #.............Control Scheame per 0.2 second............. 

        if i % 1 == 0: #check 100

           
            #----------------------------Followers---------------------------------------

            for loop_i in range(n):
                for loop_j in range(n): 
                    if loop_i != loop_j:
                        Rij[loop_i][loop_j]      =  math.sqrt( (PosX[loop_i][i]   - PosX[loop_j][i])**2  +  (PosY[loop_i][i]   - PosY[loop_j][i])**2  +  (PosZ[loop_i][i]   - PosZ[loop_j][i])**2)
                        cos_phi[loop_i][loop_j]  =  (PosX[loop_i][i] - PosX[loop_j][i]) /  math.sqrt( (PosX[loop_i][i]   - PosX[loop_j][i])**2  +  (PosY[loop_i][i]   - PosY[loop_j][i])**2 )
                        sin_phi[loop_i][loop_j]  =  (PosY[loop_i][i] - PosY[loop_j][i]) /  math.sqrt( (PosX[loop_i][i]   - PosX[loop_j][i])**2  +  (PosY[loop_i][i]   - PosY[loop_j][i])**2 )
                        cos_zeta[loop_i][loop_j] =   (math.sqrt( (PosX[loop_i][i]   - PosX[loop_j][i])**2  +  (PosY[loop_i][i]   - PosY[loop_j][i])**2 )) / Rij[loop_i][loop_j]
                        sin_zeta[loop_i][loop_j] =  (PosZ[loop_i][i] - PosZ[loop_j][i]) /  Rij[loop_i][loop_j] 

            #print ("rg:",r_rg,"rb",r_rb,"gb",r_gb) 


            #---------------------calculate forces att + rep on all UAV using above formulas FcAPF-------------------
            sum_repx = 0
            sum_repy = 0
            sum_repz = 0
            
            for loop_i in range(n):
                #ith UAV attractive force
                Fattx[loop_i] = -Katt_lead * (PosX[loop_i][i] - PosX_L[i]) * ((PosX[loop_i][i] - PosX_L[i])**2 + (PosY[loop_i][i] - PosY_L[i])**2 + (PosZ[loop_i][i] - PosZ_L[i])**2 )#- R_sphere**2 )
                Fatty[loop_i] = -Katt_lead * (PosY[loop_i][i] - PosY_L[i]) * ((PosX[loop_i][i] - PosX_L[i])**2 + (PosY[loop_i][i] - PosY_L[i])**2 + (PosZ[loop_i][i] - PosZ_L[i])**2 )#- R_sphere**2 )
                Fattz[loop_i] = -Katt_lead * (PosZ[loop_i][i] - PosZ_L[i]) * ((PosX[loop_i][i] - PosX_L[i])**2 + (PosY[loop_i][i] - PosY_L[i])**2 + (PosZ[loop_i][i] - PosZ_L[i])**2 )#- R_sphere**2 )
                #ith UAV repulsive force

                for loop_j in range(n):
                    if loop_i != loop_j:
                        sum_repx =  sum_repx + q/Rij[loop_i][loop_j]**2 * cos_phi[loop_i][loop_j] * cos_zeta[loop_i][loop_j] 
                        sum_repy =  sum_repy + q/Rij[loop_i][loop_j]**2 * sin_phi[loop_i][loop_j] * cos_zeta[loop_i][loop_j]
                        sum_repz =  sum_repz  +q/Rij[loop_i][loop_j]**2 * sin_zeta[loop_i][loop_j] 

                Frepx[loop_i] =  Krep_lead * q * sum_repx
                Frepy[loop_i] =  Krep_lead * q * sum_repy 
                Frepz[loop_i] =  Krep_lead * q * sum_repz

             
            for loop in range(n):
                Fx[loop][i]    = Fattx[loop] + Frepx[loop]  
                Fy[loop][i]    = Fatty[loop] + Frepy[loop]
                Fz[loop][i]    = Fattz[loop] + Frepz[loop]

                theta_Force_xy[loop]    = np.arctan2(Fy[loop][i], Fx[loop][i])  
                force_xy[loop]          = math.sqrt(Fx[loop][i]**2 + Fy[loop][i]**2)
                force_z[loop]           = Fz[loop][i]
                force[loop]             = math.sqrt(force_xy[loop]**2 + force_z[loop]**2)
                theta_Force_z[loop]     = np.arctan2(force_z[loop], force_xy[loop]) 



            #--------------Calculate APF Feild for virtual leader    PpAPF------------------------
            APFx_t, APFy_t, APFz_t = add_goal(PosX_L[i], PosY_L[i], PosZ_L[i], r-20 , goal)   # APF due to goal at this loc

            for j in range(10):                        #APF added for 10 obstacles one by one
                APFx_t, APFy_t, APFz_t = add_obstacle(PosX_L[i],PosY_L[i], PosZ_L[i], APFx_t,APFy_t, APFz_t, goal,j)

            APFx[i] = APFx_t
            APFy[i] = APFy_t
            APFz[i] = APFz_t

            #print( "APFx_t",APFx_t,"APFy_t",APFy_t,"APFz_t",APFz_t)

            theta_Force_L_xy = np.arctan2(APFy_t, APFx_t)   # x y angle of APF
            force_L_xy = math.sqrt(APFx[i]**2 + APFy[i]**2)  
            force_L_z  = APFz[i]
            force_L = math.sqrt(force_L_xy**2 + force_L_z**2)  
            theta_Force_L_z =  min((np.arctan2(force_L_z, force_L_xy) + Z_offset/57.2958),60/57.2958)  # x y angle of APF max function
            
            #print ("F_L:",round(force_L,1),"tL_xy:",round(theta_Force_L_xy*57.2958,1),"tL_z:",round(theta_Force_L_z*57.2958,1),"F_r",round(force_red,1),"tr_xy:",round(theta_Force_red_xy*57.2958,1),"tr_z:",round(theta_Force_red_z*57.2958,1),"F_g",round(force_green,1)," tg_xy:",round(theta_Force_green_xy*57.2958,1),"F_b",round(force_blue,1)," tb_xy:",round(theta_Force_blue_xy*57.2958,1)," tb_z:",round(theta_Force_blue_z*57.2958,1))
 

            # ...............Output...................

            #-------------Leader------------------
            DelX_L = step_size_L * np.cos(theta_Force_L_xy) * np.cos(theta_Force_L_z)
            DelY_L = step_size_L * np.sin(theta_Force_L_xy) * np.cos(theta_Force_L_z)
            DelZ_L = step_size_L * np.sin(theta_Force_L_z)

            TargetX_L[i] = PosX_L[i] + DelX_L
            TargetY_L[i] = PosY_L[i] + DelY_L
            TargetZ_L[i] = PosZ_L[i] + DelZ_L

            print("DelX= {0} DelY= {1} DelZ= {2} \n".format(DelX_L,DelY_L,DelZ_L))


            #---------------------calculate Del for all UAV using Leader Del-------------------

            for loop in range(n):
                DelX_t[loop] = (min(force[loop],500)/500 * step_size_F ) * np.cos(theta_Force_xy[loop]) * np.cos(theta_Force_z[loop])   
                DelY_t[loop] = (min(force[loop],500)/500 * step_size_F ) * np.sin(theta_Force_xy[loop]) * np.cos(theta_Force_z[loop])
                DelZ_t[loop] = (min(force[loop],500)/500 * step_size_F ) * np.sin(theta_Force_z[loop])

                DelX[loop]  = DelX_L + DelX_t[loop]        
                DelY[loop]  = DelY_L + DelY_t[loop]
                DelZ[loop]  = DelZ_L + DelZ_t[loop]
                Target[loop], Del[loop], des_Ang_xy[loop] , des_Ang_z[loop]  = goto3D( vehicle[loop], DelX[loop], DelY[loop], DelZ[loop] )

                TargetX[loop][i]   =  PosX[loop][i] + DelX[loop]
                TargetY[loop][i]   =  PosY[loop][i] + DelY[loop]
                TargetZ[loop][i]   =  PosZ[loop][i] + DelZ[loop]

        remainingDistance = get_distance_metres(currentLocation_L, goal_global)
        #print(remainingDistance)


        f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} "
        .format(i * cycle_time, i, remainingDistance,PosX_L[i],PosY_L[i],PosZ_L[i],TargetX_L[i],TargetY_L[i],TargetZ_L[i],APFx[i],APFy[i],APFz[i],DelX_L,DelY_L,DelZ_L))

        for loop in range(n):
            f.write("{0} {1} {2} {3} {4} {5} "
            .format(PosX[loop][i],PosY[loop][i],PosZ[loop][i],DelX[loop],DelY[loop],DelZ[loop]))
            
        f.write("\n")

        if remainingDistance<= 70:         #Target_Distance*0.15: #Just below target, in case of undershoot
            Target_Achieved = 1
            break
        else:
            Target_Achieved = 0 
    

        time.sleep(cycle_time) 

        i += 1
        #print ("i = {0} and Simul_iterations = {1}".format(i,Simul_iterations))



#********************************************************************************************************

    for loop in range(n):
        vehicle[loop].close()

    
    #print ("i:", i,"GS:", round(Current_Speed,1),"T_phi:",round(Desired_Heading,1), "phi:",Current_Heading, "T_X:",round(Target_X[c_i-1],1),"X:",round(Pos_X[c_i-1],1),"T_Y:",round(Target_Y[c_i-1],1), "Y:",round(Pos_Y[c_i-1],1), "Z:",round(Pos_Z[c_i-1],1),"T_Dist:",round(Target_Distance,1), "Dis_rem:",round(remainingDistance,1),"goal:",Target_Achieved)
    
    
    fig, ax = plt.subplots()
    plot_graph('goal', fig, ax, [goal[0],goal[1]],r, 'blue')
    ax.quiver(PosX_L, PosY_L, APFx, APFy)  
    #ax.quiver(PosX_red, PosY_red, Fx_red, Fy_red,color='r')
    #ax.quiver(PosX_green, PosY_green, Fx_green, Fy_green,color='g')
    #ax.quiver(PosX_blue, PosY_blue, Fx_blue, Fy_blue,color='b')

    for p in range(10):
        plot_graph(('Obs#',p+1), fig, ax, [obs_loc[p][0],obs_loc[p][1]] ,obs_r[p], 'lightgray')


    plt.plot(TargetX_L, TargetY_L, label = "way", color='green', linestyle='dotted') #marker='o',
    plt.scatter(TargetX_L, TargetY_L, color= "red", marker= "*", s=30)

    plt.plot(PosX_L, PosY_L,        label = "Pos Leader", color='black', linestyle='dotted')
    plt.scatter(PosX_L, PosY_L,color= "black", marker= "*", s=30)
    plt.plot(PosX[0], PosY[0],    label = "Pos red",    color='red')
    plt.scatter(PosX[0], PosY[0],color= "red", marker= "*", s=30)
    plt.plot(PosX[1], PosY[1],label = "Pos green",  color='green')
    plt.scatter(PosX[1], PosY[1],color= "green", marker= "*", s=30)
    plt.plot(PosX[2], PosY[2],  label = "Pos blue",   color='blue')
    plt.scatter(PosX[2], PosY[2],color= "blue", marker= "*", s=30)

    #plt.plot(x, y, color='green', linestyle='dashed', linewidth = 3,marker='o', markerfacecolor='blue', markersize=12)
    #ax.set(xlim=(0, 1000), xticks=np.arange(1, 8),ylim=(0, 1000), yticks=np.arange(1, 8))

    # naming the x axis
    plt.xlabel('x - axis')
    # naming the y axis
    plt.ylabel('y - axis')
    # giving a title to my graph
    plt.title('UAVs motion')
    
    # show a legend on the plot
    plt.legend()
    plt.grid()

    time.sleep(1) 
    f.close()

    plt.show()
    

    '''
    fig, ax = plt.subplots()  # definesthe size of figuref   igsize = (20,20)
    ax = plt.axes(projection='3d')
    ax.quiver3D(PosX_L, PosY_L,PosY_L, APFx, APFy, APFz) 
    plt.show()'''
    
#"latitude:",round(target.lat,1), "longitude:",round(target.lon,1), "altitude:",round(target.alt,1)

 


#----------------------------------------------------------------------------------------------------



"""
# Connect to the Vehicle (in this case a simulated vehicle at 127.0.0.1:14550)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Get the set of commands from the vehicle
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

# Create and add commands
cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)
cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 10, 10, 10)
cmds.add(cmd1)
cmds.add(cmd2)
cmds.upload() # Send commands"""

'''
        f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} {17} {18} {19} {20} {21} {22} {23} {24} {25} {26} "
        .format(i * cycle_time, i, round(remainingDistance,1),round(PosX_L[c_i],1),round(PosX_red[c_i],1),round(PosX_green[c_i],1), round(PosX_blue[c_i],1), round(TargetX_L[c_i],1),round(TargetX_red[c_i],1),round(TargetX_green[c_i],1),round(TargetY_blue[c_i],1),round(PosY_L[c_i],1),round(PosY_red[c_i],1),round(PosY_green[c_i],1),round(PosY_blue[c_i],1),round(TargetY_L[c_i],1),round(TargetY_red[c_i],1),round(TargetY_green[c_i],1),round(TargetY_blue[c_i],1),round(PosZ_L[c_i],1),round(PosZ_red[c_i],1),round(PosZ_green[c_i],1),round(PosZ_blue[c_i],1),round(TargetZ_L[c_i],1),round(TargetZ_red[c_i],1),round(TargetZ_green[c_i],1),round(TargetZ_blue[c_i],1)))

        #Forces Leader 
        f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} "
        .format(round(APFx[c_i],1),round(APFy[c_i],1),round(APFz[c_i],1),round(DelX_L,1),round(DelY_L,1),round(DelZ_L,1),round(force_L,1),round(force_L_xy,1),round(force_L_z,1),round(theta_Force_L_xy * 57.2958,1),round(theta_Force_L_z * 57.2958,1)  ))

        #Forces red
        f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} "
        .format(round(DelX_red,1),round(DelY_red,1),round(DelZ_red,1),round(DelX_red_t,1),round(DelY_red_t,1),round(DelZ_red_t,1),round(Fx_red[c_i],1),round(Fy_red[c_i],1),round(Fz_red[c_i],1),round(force_red,1),round(force_red_xy,1),round(force_red_z,1),round(theta_force_red_xy * 57.2958,1),round(theta_Force_red_z *57.2958 ,1)  ))
 
        #Forces green
        f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} "
        .format(round(DelX_green,1),round(DelY_green,1),round(DelZ_green,1),round(DelX_green_t,1),round(DelY_green_t,1),round(DelZ_green_t,1),round(Fx_green[c_i],1),round(Fy_green[c_i],1),round(Fz_green[c_i],1),round(force_green,1),round(force_green_xy,1),round(force_green_z,1),round(theta_force_green_xy * 57.2958,1),round(theta_Force_green_z *57.2958 ,1)  ))

        #Forces blue
        f.write("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} \n"
        .format(round(DelX_blue,1),round(DelY_blue,1),round(DelZ_blue,1),round(DelX_blue_t,1),round(DelY_blue_t,1),round(DelZ_blue_t,1),round(Fx_blue[c_i],1),round(Fy_blue[c_i],1),round(Fz_blue[c_i],1),round(force_blue,1),round(force_blue_xy,1),round(force_blue_z,1),round(theta_force_blue_xy * 57.2958,1),round(theta_Force_blue_z * 57.2958,1)  ))'''


        #print
        #print("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} {17} {18} {19} {20} {21} {22} {23} {24} {25} {26} "
        #.format(i * cycle_time, i, round(remainingDistance,1),round(PosX_L[c_i],1),round(PosX_red[c_i],1),round(PosX_green[c_i],1), round(PosX_blue[c_i],1), round(TargetX_L[c_i],1),round(TargetX_red[c_i],1),round(TargetX_green[c_i],1),round(TargetY_blue[c_i],1),round(PosY_L[c_i],1),round(PosY_red[c_i],1),round(PosY_green[c_i],1),round(PosY_blue[c_i],1),round(TargetY_L[c_i],1),round(TargetY_red[c_i],1),round(TargetY_green[c_i],1),round(TargetY_blue[c_i],1),round(PosZ_L[c_i],1),round(PosZ_red[c_i],1),round(PosZ_green[c_i],1),round(PosZ_blue[c_i],1),round(TargetZ_L[c_i],1),round(TargetZ_red[c_i],1),round(TargetZ_green[c_i],1),round(TargetZ_blue[c_i],1)))
        


     
''' global vehicle1_red1        #127.0.0.1:14551
    global vehicle2_green1      #127.0.0.1:14561
    global vehicle3_blue1       #127.0.0.1:14571
    global vehicle4_red2        #127.0.0.1:14581
    global vehicle5_green2      #127.0.0.1:14591
    global vehicle6_blue2       #127.0.0.1:14601
    global vehicle7_red3        #127.0.0.1:14611
    global vehicle8_green3      #127.0.0.1:14621
    global vehicle9_blue3       #127.0.0.1:14631
    global vehicle10_red4       #127.0.0.1:14641


 ########################gorup1###############################

    connection_string1 = '127.0.0.1:14551'
    print('Connecting to Red main vehicle on: %s' % connection_string1)
    vehicle1_red1 = connect(connection_string, wait_ready=True, baud=921600)
    print("Connection to Red main Successfully Established!")    


    connection_string2 = '127.0.0.1:14561'
    print('Connecting to green vehicle on: %s' % connection_string2)
    vehicle2_green1 = connect(connection_string2, wait_ready=True, baud=921600)
    print("Connection to green Successfully Established!")    

    connection_string3 = '127.0.0.1:14571'
    print('Connecting to Blue vehicle on: %s' % connection_string3)
    vehicle3_blue1 = connect(connection_string3, wait_ready=True, baud=921600)
    print("Connection to Blue Successfully Established!")   


    
######################gorup2#################################

    connection_string4 = '127.0.0.1:14581'
    print('Connecting to Red main vehicle on: %s' % connection_string4)
    vehicle4_red2 = connect(connection_string4, wait_ready=True, baud=921600)
    print("Connection to Red main Successfully Established!")    


    connection_string5 = '127.0.0.1:14591'
    print('Connecting to green vehicle on: %s' % connection_string5)
    vehicle5_green2 = connect(connection_string5, wait_ready=True, baud=921600)
    print("Connection to green Successfully Established!")    

    connection_string6 = '127.0.0.1:14601'
    print('Connecting to Blue vehicle on: %s' % connection_string6)
    vehicle6_blue2 = connect(connection_string6, wait_ready=True, baud=921600)
    print("Connection to Blue Successfully Established!")  

######################gorup3#################################

    connection_string7 = '127.0.0.1:14611'
    print('Connecting to Red main vehicle on: %s' % connection_string7)
    vehicle7_red3 = connect(connection_string7, wait_ready=True, baud=921600)
    print("Connection to Red main Successfully Established!")    


    connection_string8 = '127.0.0.1:14621'
    print('Connecting to green vehicle on: %s' % connection_string8)
    vehicle8_green3 = connect(connection_string8, wait_ready=True, baud=921600)
    print("Connection to green Successfully Established!")    

    connection_string9 = '127.0.0.1:14631'
    print('Connecting to Blue vehicle on: %s' % connection_string9)
    vehicle9_blue3 = connect(connection_string9, wait_ready=True, baud=921600)
    print("Connection to Blue Successfully Established!")  

######################gorup4#################################

    connection_string10 = '127.0.0.1:14641'
    print('Connecting to Red main vehicle on: %s' % connection_string10)
    vehicle10_red4 = connect(connection_string10, wait_ready=True, baud=921600)
    print("Connection to Red main Successfully Established!")  

#############################################################

    vehicle1_red1.mode   = VehicleMode("GUIDED")
    vehicle2_green1.mode = VehicleMode("GUIDED")
    vehicle3_blue1.mode  = VehicleMode("GUIDED")

    vehicle4_red2.mode   = VehicleMode("GUIDED")
    vehicle5_green2.mode = VehicleMode("GUIDED")
    vehicle6_blue2.mode  = VehicleMode("GUIDED")

    vehicle7_red3.mode   = VehicleMode("GUIDED")
    vehicle8_green3.mode = VehicleMode("GUIDED")
    vehicle9_blue3.mode  = VehicleMode("GUIDED")

    vehicle10_red4.mode  = VehicleMode("GUIDED")

    Home_Location_red1   = vehicle1_red1.location.global_frame
    Home_Location_green1   = vehicle1_red1.location.global_frame
    Home_Location_blue1   = vehicle1_red1.location.global_frame

    Home_Location_red2   = vehicle1_red1.location.global_frame
    Home_Location_green2   = vehicle1_red1.location.global_frame
    Home_Location_blue2   = vehicle1_red1.location.global_frame

    Home_Location_red3   = vehicle1_red1.location.global_frame
    Home_Location_green3   = vehicle1_red1.location.global_frame
    Home_Location_blue3   = vehicle1_red1.location.global_frame

    Home_Location_red4   = vehicle1_red1.location.global_frame
    '''   
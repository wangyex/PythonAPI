#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
from types import ClassMethodDescriptorType

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
from carla import ColorConverter as cc
from carla import Transform, Rotation

import errno
import argparse
import collections
import datetime
import logging
import random
import re
import weakref
import threading
import math

import time
import socket
import struct
import numpy as np
from dataclasses import dataclass

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
# Unit conversions:
RTOD = 57.2958
DTOR = 0.0174533

# Global - parameterize sizes of viewports
scr_wid = 1024
scr_ht = 768

# Global - parameterize the width of the HUD panel...
hud_wid = 300;

class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    # This randomly generates vehicle type.
    #bps = world.get_blueprint_library().filter(filter)

    # Hard code ego car here: https://carla.readthedocs.io/en/latest/catalogue_vehicles/
    if GRP_ID == 143.00:
        bps = world.get_blueprint_library().filter('vehicle.dodge.charger_2020')
    elif GRP_ID == 666.00:
        bps = world.get_blueprint_library().filter('vehicle.audi.a2')
    elif GRP_ID == 5150.00:
        bps = world.get_blueprint_library().filter('vehicle.mini.cooper_s')
    elif GRP_ID == 2121.00:
        bps = world.get_blueprint_library().filter('walker.pedestrian.0001')
    elif GRP_ID == 1123.00:
        bps = world.get_blueprint_library().filter('vehicle.toyota.prius')

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

# ==============================================================================
# -- Automatically get IP address for multicast --------------------------------
# ==============================================================================
def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.254.254.254', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()

    return IP


# ==============================================================================
# -- Global variables for socket threads ---------------------------------------
# ==============================================================================
# Loopback interface for motion data. 
UDP_IP = "127.0.0.1"
# UDP port for motion data.
UDP_PORT = 5005

# Multicast interface (network) and group address.
NET_IP = get_ip()
MCAST_GRP = '224.1.1.1'

# Multicast port for telemetry data.
MCAST_PORT = 5007 
MULTICAST_TTL = 4

# March, 2025 - UNIQUE ID for each Carla participant.
#GRP_ID = 143.00 # starlifter
#GRP_ID = 666.00 # crownlands
#GRP_ID = 5150.00 # jetson
GRP_ID = 2121.00 # travl2
#GRP_ID = 1123.00 # chelab02

wfile       = None   # file identifier
sim_Running = True   # defines execution for the socket threads
keyb_on     = True   # Keyboard or Steering/pedals?
motion_on   = True   # are we sending out vehicle dynamics to loopback?
tele_send   = True   # do we want to SEND telemetry?
tele_recv   = True   # do we want to RECV telemetry?
tlr_socklen = 0      # length (bytes) of tlr send packet.
tlr_double  = 0      # length (bytes) of double precision array.
slp_snd     = 0.001  # Send sleep for MB/TL sockets.  Set to 1000 Hz.
tls_cnt     = 0      # Iteration counter for TL (send).

# Vehicle dynamics for MB & HUD
max_col = 1
aa = Vector(0,0,0)
wl = Vector(0,0,0)
wf = Vector(0,0,0)

# How many connected multicast clients?
num_cli = 0 # Start at 0 - this will increment automatically!

# Arrays for UDP multicast exchange.  Space for 10.
udp_spawn_points = None
spawn = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
vehicle_blueprint = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
cli_vehicle = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
cli_trans = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
neworexi = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# ==============================================================================
# -- Data classes for UDP sockets ----------------------------------------------
# ==============================================================================
dataclass 
class cars:
    cID: float = 0
    cNO: float = 0
    cTOT: float = 0

    cPX: float = 0
    cPY: float = 0
    cPZ: float = 0

    cRX: float = 0
    cRY: float = 0
    cRZ: float = 0

dataclass 
class motionSim_carla:

    # World position of the car
    posX: float = 0
    posY: float = 0
    posZ: float = 0

    # Velocity of the car
    velX: float = 0
    velY: float = 0
    velZ: float = 0

    # Linear acceleration of the car (*gravity not included)
    accX: float = 0
    accY: float = 0
    accZ: float = 0

    # Roll, pitch and yaw positions of the car ("euler angles")
    rollPos: float = 0
    pitchPos: float = 0
    yawPos: float = 0

    # Roll, pitch and yaw "velocities" of the car (angular velocity)
    rollRate: float = 0
    pitchRate: float = 0
    yawRate: float = 0

    # Roll, pitch and yaw "accelerations" of the car (angular acceleration)
    rollAcc: float = 0
    pitchAcc: float = 0
    yawAcc: float = 0

    # (Max) collision sensor
    collSens: float = 0

# Create a variable instance of the motionSim_carla() class to send via UDP (loopback).
var_mb = motionSim_carla()

# ==============================================================================
# -- Assign blueprint parameters for UDP vehicles ------------------------------
# ==============================================================================
def client_vehicle_bp(world, sim_world):
    # UDP client test.  March, 2025.

    # (global) spawn points for UDP multicast
    global udp_spawn_points
    udp_spawn_points = world.map.get_spawn_points()

    # changing a global array
    global vehicle_blueprint

    # crownlands (666)
    vehicle_blueprint[0] = sim_world.get_blueprint_library().find('vehicle.audi.a2')
    vehicle_blueprint[0].set_attribute('color', '255,0,0')

    # jetson (5150)
    vehicle_blueprint[1] = sim_world.get_blueprint_library().find('vehicle.mini.cooper_s')
    vehicle_blueprint[1].set_attribute('color', '0,255,0')

    # starlifter (143)
    vehicle_blueprint[2] = sim_world.get_blueprint_library().find('vehicle.dodge.charger_2020')
    vehicle_blueprint[2].set_attribute('color', '0,0,255')

    # travl2 (2121)
    vehicle_blueprint[3] = sim_world.get_blueprint_library().find('walker.pedestrian.0001')
    #vehicle_blueprint[3].set_attribute('gender', 'female')
    
    # chelab02
    vehicle_blueprint[4] = sim_world.get_blueprint_library().find('vehicle.toyota.prius')
    vehicle_blueprint[4].set_attribute('color', '255,165,0')
    
    # future 3
    #vehicle_blueprint[5] = sim_world.get_blueprint_library().find('vehicle.toyota.prius')
    #vehicle_blueprint[5].set_attribute('color', '0,255,255')

# ==============================================================================
# -- Assign VD values for UDP sockets ------------------------------------------
# ==============================================================================
# Initialize the variables for angular acceleration calculation.
def mb_dynamics(world):

    # Server (local) fps calculation.
    _server_clock = pygame.time.Clock()
    server_fps = _server_clock.get_fps()

    # Trans/Rotation - coordinates ok as is.
    t = world.player.get_transform().location
    r = world.player.get_transform().rotation

    # Note - these are in WORLD coordinates - and need to be transformed to VEHICLE coordinates.
    v = world.player.get_velocity()
    a = world.player.get_acceleration()
    w = world.player.get_angular_velocity()

    # Calculation for AngAcc - uses WORLD coords.
    global aa
    global wl
    global wf

    # Make sure we aren't diving by zero.
    if (1.0/max(1.0, server_fps)) > 0:
        # Update calculations.
        if wl.x == 0.0 and wf.x == 0:
            aa.x = (w.x-wl.x) / (1.0/max(1.0, server_fps))
            wl.x = w.x
            wf.x = 1
        elif ((w.x-wl.x) != 0.0):
            aa.x = (w.x-wl.x) / (1.0/max(1.0, server_fps))
            wl.x = w.x
            
        if wl.y == 0.0 and wf.y == 0:
            aa.y = (w.y-wl.y) / (1.0/max(1.0, server_fps))
            wl.y = w.y
            wf.y = 1
        elif ((w.y-wl.y) != 0.0):
            aa.y = (w.y-wl.y) / (1.0/max(1.0, server_fps))
            wl.y = w.y

        if wl.z == 0.0 and wf.z == 0:
            aa.z = (w.z-wl.z) / (1.0/max(1.0, server_fps))
            wl.z = w.z
            wf.z = 1
        elif ((w.z-wl.z) != 0.0):
            aa.z = (w.z-wl.z) / (1.0/max(1.0, server_fps))
            wl.z = w.z
            
    # Assign values to the motionSim class for loopback.  Uses VEHICLE coordinates!
    motionSim_carla.posX = t.x
    motionSim_carla.posY = t.y
    motionSim_carla.posZ = t.z
    motionSim_carla.velX = math.cos(math.radians(r.yaw))*v.x + math.sin(math.radians(r.yaw))*v.y
    motionSim_carla.velY = -math.sin(math.radians(r.yaw))*v.x + math.cos(math.radians(r.yaw))*v.y
    motionSim_carla.velZ = -v.z
    motionSim_carla.accX = math.cos(math.radians(r.yaw))*a.x + math.sin(math.radians(r.yaw))*a.y
    motionSim_carla.accY = -math.sin(math.radians(r.yaw))*a.x + math.cos(math.radians(r.yaw))*a.y
    motionSim_carla.accZ = -a.z
    motionSim_carla.rollPos = r.roll*DTOR
    motionSim_carla.pitchPos = r.pitch*DTOR
    motionSim_carla.yawPos = r.yaw*DTOR
    motionSim_carla.rollRate = (math.cos(math.radians(r.yaw))*w.x + math.sin(math.radians(r.yaw))*w.y)*DTOR
    motionSim_carla.pitchRate = (-math.sin(math.radians(r.yaw))*w.x + math.cos(math.radians(r.yaw))*w.y)*DTOR
    motionSim_carla.yawRate = -w.z*DTOR
    motionSim_carla.rollAcc = (math.cos(math.radians(r.yaw))*aa.x + math.sin(math.radians(r.yaw))*aa.y)*DTOR
    motionSim_carla.pitchAcc = (-math.sin(math.radians(r.yaw))*aa.x + math.cos(math.radians(r.yaw))*aa.y)*DTOR
    motionSim_carla.yawAcc = -aa.z*DTOR
    motionSim_carla.collSens = max_col # note: this (global) data will only transfer at present if the HUD is called.

    # write values to a file to observe.
    global wfile
    wfile.write("{:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} \n" .format(motionSim_carla.posX, motionSim_carla.posY, motionSim_carla.posZ, motionSim_carla.velX, motionSim_carla.velY, motionSim_carla.velZ, motionSim_carla.accX, motionSim_carla.accY, motionSim_carla.accZ, motionSim_carla.rollPos, motionSim_carla.pitchPos, motionSim_carla.yawPos, motionSim_carla.rollRate, motionSim_carla.pitchRate, motionSim_carla.yawRate, motionSim_carla.rollAcc, motionSim_carla.pitchAcc, motionSim_carla.yawAcc))


# ==============================================================================
# -- Send/Receive functions for UDP sockets ------------------------------------
# ==============================================================================
def mb_sender(mb_sock):

    while sim_Running == True:

        # Create a structure for the Carla motion data - all floats.
        carla_vd_floats = np.array([var_mb.posX, var_mb.posY, var_mb.posZ, var_mb.velX, var_mb.velY, var_mb.velZ, var_mb.accX, var_mb.accY, var_mb.accZ, var_mb.rollPos, var_mb.pitchPos, var_mb.yawPos, var_mb.rollRate, var_mb.pitchRate, var_mb.yawRate, var_mb.rollAcc, var_mb.pitchAcc, var_mb.yawAcc, var_mb.collSens], dtype=np.float32)

        # Pack the structured floats into a binary string.
        packed_data_mb = struct.pack('f' * len(carla_vd_floats), *carla_vd_floats)

        # Send the packed data for motion system.
        mb_sock.sendto(packed_data_mb, (UDP_IP, UDP_PORT))

        # Limit MB send rate to 1000 Hz.
        time.sleep(slp_snd)

def tl_sender(tls_sock):
    global tls_cnt

    while sim_Running == True:
        
        # Get current (before) timestamp.
        now = datetime.datetime.now()
        b_total = float(now.minute)*6e7 + float(now.second)*1e6 + float(now.microsecond)
        #print("/ BEF: #{:d} Tot_Mcs: {:.0f}" .format(tls_cnt, b_total))

        # Create a structure for the Carla telemetry data - all float 64.
        carla_tl_double = np.array([GRP_ID, tls_cnt, b_total, motionSim_carla.posX, motionSim_carla.posY, motionSim_carla.posZ, motionSim_carla.rollPos, motionSim_carla.pitchPos, motionSim_carla.yawPos], dtype=np.float64)

        # Pack the structured doubles into a binary string.
        packed_data_tl = struct.pack('d' * len(carla_tl_double), *carla_tl_double)
            
        # These (global) values are required by the receiver.
        global tlr_socklen 
        tlr_socklen = len(packed_data_tl)
        global tlr_double
        tlr_double = len(carla_tl_double)

        # Send the packed data for telemetry exchange.
        tls_sock.sendto(packed_data_tl, (MCAST_GRP, MCAST_PORT))

        # Increment the TLs iterator.
        tls_cnt += 1

        # Limit TL send rate to 1000 Hz.
        time.sleep(slp_snd)


def tl_receiver(tlr_sock, sim_world):
    # changing global arrays
    global num_cli
    global spawn
    global cli_vehicle
    global cli_trans
    global neworexi

    # local vars to determine speed of thread recv.
    end_time_last = 0
    avg_fps = 0
    cnt = 0
    summer = 0

    while sim_Running == True:

        # Receive (and decode) packed telemetry data from telemetry multicast.  Note: we only care about *other* client data.  (Not ours).
        if tlr_socklen != 0:
            try:
                data, addr = tlr_sock.recvfrom(tlr_socklen)

                # Decode (received) telemetry data from other machines.
                cars.cID, cars.cNO, cars.cTOT, cars.cPX, cars.cPY, cars.cPZ, cars.cRX, cars.cRY, cars.cRZ = struct.unpack('d' * tlr_double, data)

                # If (other client) datagram was received - print/store packet details.
                if cars.cID != GRP_ID:

                    # March, 2025: Is this a NEW or EXISTING client?
                    newspawn = 0

                    # This is a new - and the first - client.
                    if num_cli == 0:
                        neworexi[num_cli] = cars.cID
                        num_cli += 1
                        newspawn = 1
                        cli_ind = num_cli - 1
                    else:
                        flag = 1
                        for i in range(num_cli):
                            if neworexi[i] == cars.cID:
                                flag = 0
                                cli_ind = i
                                break # Existing client found; no need to continue.  But note the client index!
                        # Or - new client confirmed
                        if flag == 1:
                            neworexi[num_cli] = cars.cID
                            num_cli += 1
                            newspawn = 1
                            cli_ind = num_cli - 1

                    # March, 2025: Spawn, if new.
                    if newspawn == 1:
                        spawn[num_cli-1] = random.choice(udp_spawn_points) 
                        # which blueprint is used depends on the (received) group ID
                        if cars.cID == 666.00:
                            num = 0
                        elif cars.cID == 5150.00:
                            num = 1
                        elif cars.cID == 143.00:
                            num = 2
                        elif cars.cID == 2121.00:
                            num = 3
                        elif cars.cID == 1123.00:
                            num = 4
                        else:
                            print("default vehicle type - WHY")
                            num = 0 # just to be safe...
                        cli_vehicle[num_cli-1] = sim_world.try_spawn_actor(vehicle_blueprint[num], spawn[num_cli-1])

                    # Print received data - only temporary.
                    #print(f"     * RCV socket: {cars.cID:.0f} {cars.cNO:.0f} {cars.cTOT:.0f} {cars.cPX:.2f} {cars.cPY:.2f} {cars.cPZ:.2f} {cars.cRX:.2f} {cars.cRY:.2f} {cars.cRZ:.2f}")
                    #print("     * Received message from {} ".format(addr))

                    # March, 2025: Transform/Draw client vehicle for newly-received data.
                    cli_transf = Transform()
                    cli_transf.location = carla.Location(x=cars.cPX, y=cars.cPY, z=cars.cPZ)
                    rotation = Rotation(pitch=cars.cRY*RTOD, yaw=cars.cRZ*RTOD, roll=cars.cRX*RTOD)
                    cli_transf.rotation = rotation
                    cli_vehicle[cli_ind].set_transform(cli_transf)

                    # Get current (after) timestamp.  Calculate (and print) the time delta.
                    now = datetime.datetime.now()
                    a_total = float(now.minute)*6e7 + float(now.second)*1e6 + float(now.microsecond)
                    #print("# AFT: #{:.0f} Tot_Mcs: {:.0f} DELTA: {:.6f} sec." .format(cars.cNO, a_total, (a_total-cars.cTOT)/1.e6))
                else:
                    end_time = time.perf_counter()
                    cnt += 1
                    summer += (1.0/(end_time-end_time_last))
                    avg_fps = summer/cnt
                    end_time_last = end_time 
                    #print(" AVG FPS: %.2f" %avg_fps)

            except socket.error as e:
                # Check if the error is due to no data being available
                if e.errno == errno.EAGAIN or e.errno == errno.EWOULDBLOCK:
                    # No data available, continue the loop
                    continue
            except socket.timeout:
                print("Exception: RECV Timeout occurred")

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint_list = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            # March, 2025: override with coded color for ego car.
            if GRP_ID == 143.00:
                blueprint.set_attribute('color', '0, 0, 255')
            elif GRP_ID == 666.00:
                blueprint.set_attribute('color', '255, 0, 0')
            elif GRP_ID == 5150.00:
                blueprint.set_attribute('color', '0, 255, 0')
            elif GRP_ID == 1123.00:
                blueprint.set_attribute('color', '255, 165, 0')
            #blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        # Setup six cameras.
        self.camera_manager1 = CameraManager(self.player, {}, self._gamma, 0, 0, 0)
        self.camera_manager1.transform_index = cam_pos_index
        self.camera_manager1.set_sensor(cam_index, notify=False)

        #self.camera_manager2 = CameraManager(self.player, {}, self._gamma, 0, 1, 0)
        #self.camera_manager2.transform_index = cam_pos_index
        #self.camera_manager2.set_sensor(cam_index, notify=False)
        
        #self.camera_manager3 = CameraManager(self.player, {}, self._gamma, 1, 2, 0)
        #self.camera_manager3.transform_index = cam_pos_index
        #self.camera_manager3.set_sensor(cam_index, notify=False)
        #self.camera_manager4 = CameraManager(self.player, {}, self._gamma, 2, 0, 1)
        #self.camera_manager4.transform_index = cam_pos_index
        #self.camera_manager4.set_sensor(cam_index, notify=False)
        #self.camera_manager5 = CameraManager(self.player, {}, self._gamma, 3, 1, 1)
        #self.camera_manager5.transform_index = cam_pos_index
        #self.camera_manager5.set_sensor(cam_index, notify=False)
        #self.camera_manager6 = CameraManager(self.player, {}, self._gamma, 4, 2, 1)
        #self.camera_manager6.transform_index = cam_pos_index
        #self.camera_manager6.set_sensor(cam_index, notify=False)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager1.render(display)
        #self.camera_manager2.render(display)
        #self.camera_manager3.render(display)
        #self.camera_manager4.render(display)
        #self.camera_manager5.render(display)
        #self.camera_manager6.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager1.sensor,
            #self.camera_manager2.sensor,
            #self.camera_manager3.sensor,
            #self.camera_manager4.sensor,
            #self.camera_manager5.sensor,
            #self.camera_manager6.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot, cyb_device=None):
        self._autopilot_enabled = start_in_autopilot
        self._ackermann_enabled = False
        self._ackermann_reverse = 1
        self._cyb_device = cyb_device # Store the device object
        self.pedestrian_speed_factor = 100.0 # <<< ADJUST THIS FACTOR AS NEEDED
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_f:
                        # Toggle ackermann controller
                        self._ackermann_enabled = not self._ackermann_enabled
                        world.hud.show_ackermann_info(self._ackermann_enabled)
                        world.hud.notification("Ackermann Controller %s" %
                                               ("Enabled" if self._ackermann_enabled else "Disabled"))
                    if event.key == K_q:
                        if not self._ackermann_enabled:
                            self._control.gear = 1 if self._control.reverse else -1
                        else:
                            self._ackermann_reverse *= -1
                            # Reset ackermann control
                            self._ackermann_control = carla.VehicleAckermannControl()
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker
        #print(f"DEBUG: In parse_events. Control type: {type(self._control)}, Autopilot: {self._autopilot_enabled}")
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                # Apply control
                if not self._ackermann_enabled:
                    world.player.apply_control(self._control)
                else:
                    world.player.apply_ackermann_control(self._ackermann_control)
                    # Update control to the last one applied by the ackermann controller.
                    self._control = world.player.get_control()
                    # Update hud with the newest ackermann control
                    world.hud.update_ackermann_control(self._ackermann_control)

            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
                world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            if not self._ackermann_enabled:
                self._control.throttle = min(self._control.throttle + 0.1, 1.00)
            else:
                self._ackermann_control.speed += round(milliseconds * 0.005, 2) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            if not self._ackermann_enabled:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._ackermann_control.speed -= min(abs(self._ackermann_control.speed), round(milliseconds * 0.005, 2)) * self._ackermann_reverse
                self._ackermann_control.speed = max(0, abs(self._ackermann_control.speed)) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        if not self._ackermann_enabled:
            self._control.steer = round(self._steer_cache, 1)
            self._control.hand_brake = keys[K_SPACE]
        else:
            self._ackermann_control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        # --- Read SDK speed first if device exists ---
        sdk_based_speed = 0.0 # Default to 0
        if self._cyb_device:
            try:
                # Get speed from SDK
                raw_sdk_speed = self._cyb_device.GetMovementSpeed()
                print(f"DEBUG: Raw SDK Movement Speed = {sdk_speed}") # Keep the debug print
                # Apply the factor
                sdk_based_speed = sdk_speed * self.pedestrian_speed_factor
            except Exception as e:
                print(f"Error reading Cyberith SDK speed: {e}")
                # If SDK fails, sdk_based_speed remains 0 or you could fallback differently
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if (keys[K_UP] or keys[K_w]):
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize the joystick module
        pygame.joystick.init()

        # How many joystick devices are present?
        pygame.joystick.get_count()

        self._swheel = pygame.joystick.Joystick(0)
        self._swheel.init()

        self._pedals = pygame.joystick.Joystick(0)
        self._pedals.init()

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                elif event.button == 1:
                    world.hud.toggle_info()
                elif event.button == 2:
                    world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 23:
                    world.camera_manager.next_sensor()

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._parse_vehicle_pedals()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._swheel.get_numaxes()
        #print("Num axes swheel:", str(self._swheel.get_numaxes()))

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * self._swheel.get_axis(0))
        self._control.steer = steerCmd
        #print("Steering cmd:", steerCmd)

    def _parse_vehicle_pedals(self):
        numAxes = self._pedals.get_numaxes()
        #print("Num axes pedals:", str(self._pedals.get_numaxes()))

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # The pedal signals need to be inverted - hence the "-1.0"
        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(-0.7 * (self._pedals.get_axis(2)*-1.0) + 1.4) - 1.2) / 0.92
       
        # Limit signal ranges for both pedals.
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1
        self._control.throttle = throttleCmd
        #print("Throttle cmd:", throttleCmd)

        K3 = 1.6
        brakeCmd = K3 + (2.05 * math.log10(-0.7 * (self._pedals.get_axis(5)*-1.0) + 1.4) - 1.2) / 0.92

        # Limit signal ranges for both pedals.
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1
        self._control.brake = brakeCmd
        #print("Brake cmd:", brakeCmd)

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):

        self._notifications.tick(world, clock)
        if not self._show_info:
            return
       
        # Compass.
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''

        # Collisions.
        global max_col
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]

        # Vehicles.
        vehicles = world.world.get_actors().filter('vehicle.*')
        
        # HUD data.
        self._info_text = [
            'Server/Client:  % 4.0f %4.0f FPS' % (self.server_fps, clock.get_fps()),
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            'Keyboard control: % 11s' % keyb_on,
            'Motion: %s T_Send: %s T_Recv: %s' %(motion_on, tele_send, tele_recv),
            '',
            'Speed:   % 16.0f mph' % (2.236 * math.sqrt(motionSim_carla.velX**2 + motionSim_carla.velY**2 + motionSim_carla.velZ**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            ' Pos: % 20s' % ('(% 5.1f, % 5.1f, % 5.1f) m' % (motionSim_carla.posX, motionSim_carla.posY, motionSim_carla.posZ)),
            ' Vel: % 20s' % ('(% 5.3f, % 5.3f, % 5.3f) m/s' % (motionSim_carla.velX, motionSim_carla.velY, motionSim_carla.velZ)),
            ' Acc: % 20s' % ('(% 5.3f, % 5.3f, % 5.3f) m/s^2' % (motionSim_carla.accX, motionSim_carla.accY, motionSim_carla.accZ)),
            'RotP: % 20s' % ('(% 5.3f, % 5.3f, % 5.3f) rad' % (motionSim_carla.rollPos, motionSim_carla.pitchPos, motionSim_carla.yawPos)),
            'RotV:% 20s' % ('(% 5.3f, % 5.3f, % 5.3f) rad/s' % (motionSim_carla.rollRate, motionSim_carla.pitchRate, motionSim_carla.yawRate)),
            'RotA:% 20s' % ('(% 5.3f, % 5.3f, % 5.3f) rad/s^2' % (motionSim_carla.rollAcc, motionSim_carla.pitchAcc, motionSim_carla.yawAcc)),
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - motionSim_carla.posX)**2 + (l.y - motionSim_carla.posY)**2 + (l.z - motionSim_carla.posZ)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):

        if self._show_info:
            ss_wid = 0
            #info_surface = pygame.Surface((220, self.dim[1]))
            info_surface = pygame.Surface((hud_wid, scr_ht))
            info_surface.set_alpha(200)
            #display.blit(info_surface, (scr_wid, 0))
            display.blit(info_surface, (ss_wid, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + (ss_wid+8), v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((ss_wid+bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((ss_wid+bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((ss_wid+bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((ss_wid+bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (ss_wid+8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction, offset_var1, offset_var2, offset_var3):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self.offsety = offset_var1
        self.offseth = offset_var2
        self.offsetv = offset_var3

        # Note: these bounds change in accordance with the spawner vehicle dimensions!
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            # 1.0 * boundz is in the vehicle (CG).  Need to figure out how to make ego vehicle transparent.  
            #self._camera_transforms = [(carla.Transform(carla.Location(x=-0.0*bound_x, y=-0.0*bound_y, z=1.0*bound_z), carla.Rotation(pitch=0.0, yaw=self.offsety*60, roll=0.0)), Attachment.Rigid)]
            # 2.0 * boundz is in the vehicle (hood).  Need to figure out how to make ego vehicle transparent.  
            self._camera_transforms = [(carla.Transform(carla.Location(x=-0.0*bound_x, y=-0.0*bound_y, z=1.75*bound_z), carla.Rotation(pitch=0.0, yaw=self.offsety*60, roll=0.0)), Attachment.Rigid)]
        else:
            self._camera_transforms = [(carla.Transform(carla.Location(x=-5.0*bound_x, y=-0.0*bound_y, z=0.75*bound_z), carla.Rotation(pitch=0.0, yaw=self.offsety*60, roll=0.0)), Attachment.Rigid)]
            
            '''
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]
            '''

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
            ['sensor.camera.normals', cc.Raw, 'Camera Normals', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                
                # Assign viewport sizes.
                #bp.set_attribute('image_size_x', str(hud.dim[0]))
                #bp.set_attribute('image_size_y', str(hud.dim[1]))
                bp.set_attribute('image_size_x', str(scr_wid))
                bp.set_attribute('image_size_y', str(scr_ht))
                
                # assign FOV (horiz) angle for each of the 6 panels.
                bp.set_attribute('fov', str(60))

                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(self.sensors[index][-1], self._camera_transforms[self.transform_index][0], attach_to=self._parent, attachment_type=self._camera_transforms[self.transform_index][1])

           # We need to pass the lambda a weak reference to self to avoid circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (scr_wid*self.offseth, scr_ht*self.offsetv))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    # global array (for destroy)
    global cli_vehicle

    pygame.init()
    pygame.font.init()
    original_settings = None
    world = None
    # --- Add this section in game_loop() before the main loop ---
# --- Find this section in game_loop() ---
    try:
        from cybsdk import Virt
        cyb_device = Virt.FindDevice()
        # ... (rest of the try block) ...
    except ImportError:
        print("cybsdk not found. Install it or check path. Pedestrian speed will not use SDK.")
        cyb_device = None
    except Exception as e:
        print(f"An error occurred during Cyberith SDK setup: {e}")
        cyb_device = None
    # --- End of existing try...except block ---

    # --- ADD THIS DEBUG PRINT LINE ---
    print(f"DEBUG: cyb_device object is: {cyb_device}")
    sdk_speed = cyb_device.GetMovementSpeed()
    print(f"DEBUG: cyb_device speed is: {sdk_speed}")
    # --- END ADDED LINE ---


    
    # Setup MB socket.
    if motion_on == True:
        print("- MB send Socket open.")
        mb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Setup Telemetry socket (send).
    if tele_send == True:
        print("/ TL send Socket open.")
        tls_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        tls_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)

    # Setup Telemetry socket (receive).
    if tele_recv == True:
        print("# TL recv Socket open.")
        tlr_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        tlr_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind the receive socket.
        tlr_sock.bind((NET_IP, MCAST_PORT))
        tlr_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(NET_IP))
        tlr_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(NET_IP))
        # Set a max wait time (seconds) for blocking recvfrom
        tlr_sock.setblocking(False)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        sim_world = client.get_world()
        
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()

            if not settings.synchronous_mode:
                # Set synchronous mode.
                settings.synchronous_mode = True

                # Sync to 60 (see below)...
                settings.fixed_delta_seconds = 0.016666
            
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could experience some issues with the traffic simulation")

        display = pygame.display.set_mode((args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        
        if keyb_on == True:
            controller = KeyboardControl(world, args.autopilot)
        else:
            controller = DualControl(world, args.autopilot)

        # March, 2025: Assign vehicle properties (database) for multicast clients.
        client_vehicle_bp(world, sim_world)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        # Socket threads HERE - not bound by display update clock.
        # Send MB socket - use threads
        if motion_on == True:
            threading.Thread(target=mb_sender, args=(mb_sock, ), daemon=True).start()

        # send TL socket - use threads
        if tele_send == True:
            threading.Thread(target=tl_sender, args=(tls_sock, ), daemon=True).start()

        # recv TL socket - use threads.
        if tele_recv == True:
            threading.Thread(target=tl_receiver, args=(tlr_sock, sim_world, ), daemon=True).start()     

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()

            # Sync to 60 (see above)...
            clock.tick_busy_loop(60)

            # calculate dynamics - updates required for send (MB, TL) threads.
            mb_dynamics(world)
            
            if keyb_on == True:
                if controller.parse_events(client, world, clock, args.sync):
                    return
            else:
                if controller.parse_events(world, clock):
                    return
            
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        # Cleanup vehicle actor(s).  March 2025.
        for i in range(num_cli):
            cli_vehicle[i].destroy()
        
        # Delete client.
        del client

        # Terminate socket execution status
        global sim_Running
        sim_Running = False

        # Close sockets.
        if motion_on == True:
            mb_sock.shutdown(socket.SHUT_RDWR)
            print("- MB send Socket closed.")
        if tele_send == True:
            tls_sock.shutdown(socket.SHUT_RDWR)
            print("/ TL send Socket closed.")
        if tele_recv == True:
            tlr_sock.shutdown(socket.SHUT_RDWR)
            print("# TL recv Socket closed.")

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    global wfile
    wfile = open("output.txt", "w")

    try:
        game_loop(args)

    #wfile.close()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()

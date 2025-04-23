#!/usr/bin/env python
"""
Modified T2_one_manual_control.py with Cyberith Integration for Walker Control

Usage:
    python T2_one_manual_control.py [--host HOST] [--port PORT] [--cyberith]

--cyberith : When provided and if the spawned actor is a walker, control is derived
             from the Cyberith Virtualizer’s data (speed and orientation) rather than keyboard.
"""

import glob
import os
import sys
import random
import time
import math
import argparse
import pygame
from pygame.locals import K_ESCAPE, K_q

# =============================================================================
# -- Find Carla module ---------------------------------------------------------
# =============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# =============================================================================
# -- Cyberith SDK Import & Initialization --------------------------------------
# =============================================================================
USE_CYBERITH = False
cyber_device = None
if '--cyberith' in sys.argv:
    USE_CYBERITH = True
    try:
        from cybsdk import *
    except ImportError:
        print("Cyberith SDK not found. Please ensure the cybsdk module is available.")
        sys.exit(-1)
    cyber_device = Virt.FindDevice()
    if cyber_device is None:
        print("No Cyberith Virtualizer device found!")
        sys.exit(-1)
    if not cyber_device.Open():
        print("Unable to connect to Cyberith Virtualizer!")
        sys.exit(-1)

# =============================================================================
# -- Minimal HUD Implementation -----------------------------------------------
# =============================================================================
class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        self.font = pygame.font.Font(None, 24)
    
    def render(self, display, text):
        surface = self.font.render(text, True, (255, 255, 255))
        display.blit(surface, (10, 10))

# =============================================================================
# -- Keyboard Control (for vehicle/walker) -------------------------------------
# =============================================================================
class KeyboardControl(object):
    def __init__(self, world):
        self.world = world
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
        else:
            raise NotImplementedError("Actor type not supported")
    
    def parse_events(self):
        exit_requested = False
        # Process events to keep the window responsive.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit_requested = True
            elif event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE or event.key == K_q:
                    exit_requested = True

        if isinstance(self._control, carla.WalkerControl):
            keys = pygame.key.get_pressed()
            # If forward is pressed, set a constant forward speed.
            if keys[pygame.K_UP] or keys[pygame.K_w]:
                self._control.speed = 2.0
            else:
                self._control.speed = 0.0
            # Basic left/right directional control.
            if keys[pygame.K_LEFT] or keys[pygame.K_a]:
                self._control.direction = carla.Vector3D(-1.0, 0.0, 0.0)
            elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                self._control.direction = carla.Vector3D(1.0, 0.0, 0.0)
            else:
                self._control.direction = carla.Vector3D(0.0, 1.0, 0.0)
        return exit_requested
    
    def get_control(self):
        return self._control

# =============================================================================
# -- Cyberith Walker Control --------------------------------------------------
# =============================================================================
class CyberithWalkerControl(object):
    """
    This control class reads the Virtualizer’s data stream (movement speed and player orientation)
    and converts it into a WalkerControl command.
    """
    def __init__(self, walker):
        self.walker = walker
        self._control = carla.WalkerControl()
    
    def update_from_cyberith(self, device):
        # Get the orientation (normalized value [0,1]) and convert to degrees.
        orientation_deg = device.GetPlayerOrientation() * 360.0
        # Get movement speed in m/s.
        speed = device.GetMovementSpeed()
        yaw_rad = math.radians(orientation_deg)
        # Create a 2D unit vector from the yaw angle.
        direction = carla.Vector3D(math.cos(yaw_rad), math.sin(yaw_rad), 0)
        self._control.speed = speed
        self._control.direction = direction
        self._control.jump = False  # Change as needed for jump functionality.
    
    def get_control(self):
        return self._control

# =============================================================================
# -- Main Function -------------------------------------------------------------
# =============================================================================
def main():
    argparser = argparse.ArgumentParser(
        description="CARLA Manual Control with Cyberith Integration")
    argparser.add_argument('--host', default='127.0.0.1', help='CARLA host')
    argparser.add_argument('--port', default=2000, type=int, help='CARLA port')
    argparser.add_argument('--cyberith', action='store_true', help='Use Cyberith control for walker')
    args = argparser.parse_args()
    
    global USE_CYBERITH
    if args.cyberith:
        USE_CYBERITH = True

    pygame.init()
    display = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("CARLA Manual Control with Cyberith Integration")
    clock = pygame.time.Clock()
    
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    
    blueprint_library = world.get_blueprint_library()
    # If using Cyberith, assume we want a pedestrian.
    if USE_CYBERITH:
        actor_bp = random.choice(blueprint_library.filter("walker.pedestrian.*"))
    else:
        actor_bp = random.choice(blueprint_library.filter("vehicle.*"))
    
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("No spawn points available")
        sys.exit(1)
    spawn_point = random.choice(spawn_points)
    
    player = world.try_spawn_actor(actor_bp, spawn_point)
    if player is None:
        print("Unable to spawn actor")
        sys.exit(1)
    world.player = player  # Attach the actor to the world for easier access.
    
    hud = HUD(800, 600)
    
    # Decide on the control mode based on the flag and actor type.
    if USE_CYBERITH and isinstance(player, carla.Walker):
        control_mode = "Cyberith"
        cyberith_control = CyberithWalkerControl(player)
    else:
        control_mode = "Keyboard"
        keyboard_control = KeyboardControl(world)
    
    running = True
    while running:
        # Always process events to avoid freezing the window.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE or event.key == K_q:
                    running = False

        # Update control based on the chosen mode.
        if USE_CYBERITH and isinstance(player, carla.Walker):
            # In Cyberith mode, update using the Virtualizer data.
            cyberith_control.update_from_cyberith(cyber_device)
            player.apply_control(cyberith_control.get_control())
            orientation_deg = cyber_device.GetPlayerOrientation() * 360.0
            speed = cyber_device.GetMovementSpeed()
            display_text = "Cyberith Control: Orientation = {:.2f}°, Speed = {:.2f} m/s".format(
                orientation_deg, speed)
        else:
            # Use keyboard control.
            if keyboard_control.parse_events():
                running = False
            player.apply_control(keyboard_control.get_control())
            display_text = "Keyboard Control"
        
        display.fill((0, 0, 0))
        hud.render(display, display_text)
        pygame.display.flip()
        clock.tick(20)
    
    player.destroy()
    pygame.quit()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import carla
import pygame
import time
import numpy as np

def find_weather_presets():
    """Utility function for listing all weather presets."""
    rgx = lambda x: x if hasattr(carla.WeatherParameters, x) else None
    name = [x for x in dir(carla.WeatherParameters) if rgx(x)]
    return [(x, getattr(carla.WeatherParameters, x)) for x in name if rgx(x)]

def get_keyboard_control(keys, current_control, step=0.01):
    """
    Interpret keyboard input and modify the walker control accordingly.
    :param keys: Pressed keys from pygame.key.get_pressed()
    :param current_control: The current WalkerControl object.
    :param step: Speed increment step.
    :return: Updated WalkerControl.
    """

    # By default, let’s not jump
    current_control.jump = False

    # WASD for movement
    forward = keys[pygame.K_w]
    backward = keys[pygame.K_s]
    left = keys[pygame.K_a]
    right = keys[pygame.K_d]
    jump = keys[pygame.K_SPACE]

    # Convert these to a direction vector
    # This is a simplistic approach: we’ll interpret
    # the walker’s "forward" direction as +Y in the world frame
    # and "left" direction as -X in the world frame. You may want
    # to rotate this based on walker orientation.
    move_x = 0.0
    move_y = 0.0

    if forward:
        move_y += 1.0
    if backward:
        move_y -= 1.0
    if left:
        move_x -= 1.0
    if right:
        move_x += 1.0

    # If there's any movement
    direction = carla.Vector2D(move_x, move_y)

    # Normalize direction if not zero to get consistent speeds
    length = np.hypot(direction.x, direction.y)
    if length > 0:
        direction.x /= length
        direction.y /= length

    # Set the speed you want your walker to have
    # We'll use a constant speed whenever a direction key is pressed
    # You can make this more sophisticated if needed.
    speed = 2.0 if (forward or backward or left or right) else 0.0

    # Convert direction 2D -> 3D as required by WalkerControl (X, Y, Z)
    current_control.direction = carla.Vector3D(direction.x, direction.y, 0)
    current_control.speed = speed

    # Handle jump input
    if jump:
        current_control.jump = True

    return current_control

def main():
    # -------------------------------------------------------------------------
    # 1. Connect to Carla
    # -------------------------------------------------------------------------
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world
    world = client.get_world()
    original_settings = world.get_settings()

    # -------------------------------------------------------------------------
    # 2. Set synchronous mode (optional, but recommended for controlling time)
    # -------------------------------------------------------------------------
    # Note: If your CARLA server is running in synchronous mode, make sure
    # to set these settings accordingly. If not, you can remove or adapt them.
    synchronous_mode = True
    delta_seconds = 0.05

    settings = world.get_settings()
    settings.synchronous_mode = synchronous_mode
    settings.fixed_delta_seconds = delta_seconds
    world.apply_settings(settings)

    # -------------------------------------------------------------------------
    # 3. Spawn a walker
    # -------------------------------------------------------------------------
    blueprint_library = world.get_blueprint_library()
    walker_bp = blueprint_library.find('walker.pedestrian.0001')

    # We pick a spawn point randomly for demonstration
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0] if spawn_points else carla.Transform()

    # Spawn the walker actor
    walker_actor = None
    try:
        walker_actor = world.spawn_actor(walker_bp, spawn_point)
        print(f"Spawned walker with id: {walker_actor.id}")
    except:
        print("Failed to spawn walker. Check spawn points or blueprint.")
        return

    # -------------------------------------------------------------------------
    # 4. Initialize pygame for user input
    # -------------------------------------------------------------------------
    pygame.init()
    width = 640
    height = 480
    pygame.display.set_mode((width, height))
    pygame.display.set_caption("CARLA Walker Control Example")
    clock = pygame.time.Clock()

    # -------------------------------------------------------------------------
    # 5. Main Control Loop
    # -------------------------------------------------------------------------
    walker_control = carla.WalkerControl()
    try:
        # If synchronous, we need to tick once to let the walker appear
        if synchronous_mode:
            world.tick()
        else:
            world.wait_for_tick()

        running = True
        while running:
            # Limit the loop to 30 FPS or as you prefer
            clock.tick(30)

            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # Get pressed keys
            keys = pygame.key.get_pressed()

            # Update control based on keys
            walker_control = get_keyboard_control(keys, walker_control)

            # Apply control to the walker
            walker_actor.apply_control(walker_control)

            # Tick the world (if in sync mode)
            if synchronous_mode:
                world.tick()
            else:
                world.wait_for_tick()

    finally:
        # ---------------------------------------------------------------------
        # 6. Cleanup
        # ---------------------------------------------------------------------
        print("Cleaning up...")
        if walker_actor is not None:
            walker_actor.destroy()
        world.apply_settings(original_settings)
        pygame.quit()
        print("Done.")

if __name__ == '__main__':
    main()

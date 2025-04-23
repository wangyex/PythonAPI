#!/usr/bin/env python3

import carla
import pygame
import queue
import numpy as np
import time

# ------------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------------

def sensor_callback(image, data_queue):
    """
    This function is called each time a camera image is generated.
    We just store the image in a queue for later use.
    """
    data_queue.put(image)

def convert_image_to_surface(image):
    """
    Convert a CARLA raw image to a pygame.Surface.
    """
    # The image is BGRA, so we have to transform it into a numpy array, reshape, then swap channels.
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    # Convert BGRA to RGB
    array = array[:, :, :3][:, :, ::-1]
    # Create a Surface from the array
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    return surface

def get_keyboard_control(keys, current_control):
    """
    Interpret keyboard input (WASD + SPACE) to create walker control.
    direction is a 3D vector, but we only manipulate X and Y here.
    """
    # Reset jump each loop
    current_control.jump = False

    # Basic WASD
    forward = keys[pygame.K_w]
    backward = keys[pygame.K_s]
    left = keys[pygame.K_a]
    right = keys[pygame.K_d]
    jump = keys[pygame.K_SPACE]

    move_x = 0.0
    move_y = 0.0

    # Forward/backward => +Y / -Y
    if forward:
        move_y += 1.0
    if backward:
        move_y -= 1.0

    # Left/right => -X / +X
    if left:
        move_x -= 1.0
    if right:
        move_x += 1.0

    # Normalize direction
    length = np.hypot(move_x, move_y)
    if length > 0:
        move_x /= length
        move_y /= length

    # Set speed when any movement key is pressed
    speed = 2.0 if (forward or backward or left or right) else 0.0

    # Update the control
    current_control.direction = carla.Vector3D(move_x, move_y, 0)
    current_control.speed = speed

    # Jump
    if jump:
        current_control.jump = True

    return current_control

# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main():
    actor_list = []
    pygame.init()

    try:
        # Connect to CARLA
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # Store original settings to revert later
        original_settings = world.get_settings()

        # Set synchronous mode so we can control ticks
        synchronous_mode = True
        delta_seconds = 0.05  # 20 FPS
        settings = world.get_settings()
        settings.synchronous_mode = synchronous_mode
        settings.fixed_delta_seconds = delta_seconds
        world.apply_settings(settings)

        # Create a display window in pygame
        width = 1920
        height = 1080
        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("CARLA Walker Control")
        clock = pygame.time.Clock()

        # Get blueprint library and pick a walker
        blueprint_library = world.get_blueprint_library()
        walker_bp = blueprint_library.find('walker.pedestrian.0001')

        # Spawn point
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("No spawn points available!")
            return
        spawn_point = spawn_points[0]

        # Spawn the walker
        walker_actor = world.spawn_actor(walker_bp, spawn_point)
        actor_list.append(walker_actor)
        print(f"Spawned walker: {walker_actor.id}")

        # -- Spawn a camera sensor and attach to the walker. --
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        # Set desired resolution
        camera_bp.set_attribute('image_size_x', str(width))
        camera_bp.set_attribute('image_size_y', str(height))
        camera_bp.set_attribute('fov', '90')

        # Attach behind and slightly above the walker (3rd-person style).
        # If you want first-person, set a smaller offset or 0 for X, Y.
        camera_transform = carla.Transform(
            carla.Location(x=-3.0, y=0.0, z=2.0),  # 3 meters behind, 2 meters up
            carla.Rotation(pitch=0, yaw=0, roll=0)
        )
        camera_actor = world.spawn_actor(camera_bp, camera_transform, attach_to=walker_actor)
        actor_list.append(camera_actor)
        print(f"Spawned camera: {camera_actor.id}")

        # Create a queue to hold camera images
        image_queue = queue.Queue()
        camera_actor.listen(lambda image: sensor_callback(image, image_queue))

        # Tick once so actors appear
        if synchronous_mode:
            world.tick()
        else:
            world.wait_for_tick()

        # WalkerControl object
        walker_control = carla.WalkerControl()

        running = True
        while running:
            # Limit the loop to ~ 30 FPS for pygame
            clock.tick(30)

            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # Get keyboard keys
            keys = pygame.key.get_pressed()

            # Update the walker control
            walker_control = get_keyboard_control(keys, walker_control)
            walker_actor.apply_control(walker_control)

            # Advance the world (synchronous)
            if synchronous_mode:
                world.tick()
            else:
                world.wait_for_tick()

            # Retrieve camera image (if available)
            image = None
            try:
                image = image_queue.get_nowait()
            except queue.Empty:
                pass

            # If we got an image, convert and display it
            if image is not None:
                surface = convert_image_to_surface(image)
                screen.blit(surface, (0, 0))
            else:
                # If no image was yet received, just fill black
                screen.fill((0, 0, 0))

            pygame.display.flip()

    finally:
        # Cleanup
        print("Destroying actors...")
        for actor in actor_list:
            actor.destroy()

        # Revert world settings
        world.apply_settings(original_settings)
        pygame.quit()
        print("Done.")

if __name__ == "__main__":
    main()

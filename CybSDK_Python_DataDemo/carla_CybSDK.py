# Add near the top imports
import time
import os
from distutils.dir_util import copy_tree
# Ensure CybSDK module and DLLs are accessible
# copy_tree("../CybSDK", ".") # Adjust path as needed
try:
    from cybsdk import *
    cybsdk_available = True
except ImportError:
    print("!!! CybSDK library not found. Running with keyboard controls only.")
    cybsdk_available = False
    device = None # Define device as None if SDK not available

# --- Global variable for CybSDK device ---
cyb_device = None

# --- Add CybSDK Initialization (e.g., before the main game_loop function or early inside it) ---
def initialize_cybsdk():
    global cyb_device
    if not cybsdk_available:
        return False

    print("Attempting to find CybSDK device...")
    cyb_device = Virt.FindDevice()
    if cyb_device is None:
        print("CybSDK Error: No Device Found!")
        return False

    info = cyb_device.GetDeviceInfo()
    print("CybSDK Device Found: {0}".format(info.ProductName))
    print("CybSDK Firmware Version: {0}.{1}".format(info.MajorVersion, info.MinorVersion))

    if not cyb_device.Open():
        print("CybSDK Error: Unable to connect to Virtualizer!")
        cyb_device = None
        return False

    print("CybSDK Device Connected Successfully.")
    return True

# --- Modify the KeyboardControl class ---
class KeyboardControl(object):
    # ... (keep existing __init__ but add a check for cyb_device) ...
    def __init__(self, world, start_in_autopilot):
        global cyb_device # Access the global device object
        # ... (rest of existing __init__ for vehicle/walker) ...

        if isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
            if cyb_device is None:
                 world.hud.notification("CybSDK not active. Using Keyboard for Walker.", seconds=5.0)
            else:
                 world.hud.notification("CybSDK Active. Controlling Walker.", seconds=5.0)
        # ...

    # Modify parse_events or create a new method to handle CybSDK input for walker
    def parse_events(self, client, world, clock, sync_mode):
        global cyb_device # Access the global device object

        # --- Handle Walker Control with CybSDK ---
        # Check if the player is a walker and CybSDK is active
        if isinstance(self._control, carla.WalkerControl) and cyb_device is not None:
            # Read data from CybSDK
            speed = cyb_device.GetMovementSpeed()
            orientation_normalized = cyb_device.GetPlayerOrientation() # 0.0 to 1.0

            # Convert orientation (0-1) to yaw degrees (0-360)
            # Adjust if CARLA's yaw convention differs (e.g., -180 to 180)
            yaw_degrees = orientation_normalized * 360.0
            # Example adjustment if CARLA uses -180 to 180:
            # if yaw_degrees > 180:
            #    yaw_degrees -= 360

            # Update walker control
            self._control.speed = speed # May need scaling: e.g., speed * MAX_WALKER_SPEED_FACTOR
            self._rotation.yaw = yaw_degrees
            self._control.direction = self._rotation.get_forward_vector()

            # Apply the control directly, bypassing keyboard checks for walker movement
            world.player.apply_control(self._control)

            # Still process pygame events for other things (quit, UI, etc.)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True
                    # Handle other non-movement keyboard shortcuts (F1, H, TAB, C, etc.)
                    # ... (add back necessary key handlers from original parse_events)

            return False # Continue running

        # --- Fallback to original Keyboard/Vehicle Control ---
        else:
            # ... (Paste the original content of parse_events here) ...
            # This will handle vehicle control and walker control if CybSDK isn't active
            # Make sure the original walker key parsing part is conditional
            # on cyb_device being None if you merge logic.

            # Example modification within the original key parsing section:
            # elif isinstance(self._control, carla.WalkerControl):
            #    if cyb_device is None: # Only parse keys if CybSDK isn't controlling
            #       self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
            #       world.player.apply_control(self._control)
            #    # else: CybSDK is handling control above

            # --- The original parsing logic from T2_one_manual_control.py ---
            if isinstance(self._control, carla.VehicleControl):
                 current_lights = self._lights
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    # --- Keep all the original KEYUP event handling ---
                    if self._is_quit_shortcut(event.key):
                         return True
                    # ... (all other elif event.key == ... conditions) ...
                    elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                         # ... (recorder logic) ...
                         pass # Placeholder

            if not self._autopilot_enabled:
                if isinstance(self._control, carla.VehicleControl):
                     self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                     # ... (rest of vehicle control application) ...
                     world.player.apply_control(self._control) # Or ackermann
                elif isinstance(self._control, carla.WalkerControl):
                     # *** Add check here if merging logic ***
                     if cyb_device is None:
                         self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
                         world.player.apply_control(self._control)


    # Keep _parse_vehicle_keys as is
    def _parse_vehicle_keys(self, keys, milliseconds):
        # ... (original content) ...
        pass

    # Keep _parse_walker_keys for fallback keyboard control
    def _parse_walker_keys(self, keys, milliseconds, world):
        # ... (original content) ...
        pass

    # Keep _is_quit_shortcut
    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# --- In the main game_loop function ---
def game_loop(args):
    # ... (existing setup) ...
    global cyb_device # Ensure game_loop knows about the global

    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    # --- Initialize CybSDK ---
    cybsdk_initialized = initialize_cybsdk()
    # Optionally, handle the case where initialization fails (e.g., exit or fallback)
    # if not cybsdk_initialized and args.require_cybsdk: # Add a hypothetical arg
    #    print("Exiting because CybSDK connection failed.")
    #    return

    try:
        # ... (original client, sim_world, display, hud, world setup) ...

        # --- Controller selection (KeyboardControl handles CybSDK internally now) ---
        controller = KeyboardControl(world, args.autopilot)

        # ... (rest of the original game_loop) ...

        clock = pygame.time.Clock()
        while True:
            # ... (tick, mb_dynamics) ...

            # --- Updated Controller parsing ---
            if controller.parse_events(client, world, clock, args.sync):
                 return # Quit requested

            # ... (world.tick, world.render, display.flip) ...

    finally:
        # ... (original cleanup) ...
        # Close CybSDK device if it was opened
        global cyb_device
        if cyb_device is not None:
            print("Closing CybSDK device.")
            # cyb_device.Close() # Assuming there's a close method
        pygame.quit()

# --- Make sure to call main() ---
if __name__ == '__main__':
    main()
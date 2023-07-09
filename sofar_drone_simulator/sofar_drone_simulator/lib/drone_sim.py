import arcade
import random
import numpy as np
from threading import Thread

WIDTH = 1200
HEIGHT = 400
TITLE = "Drone Simulator"

# Gravity constant [m/s^2]
G = 9.8

# Drone class
class Drone(arcade.Sprite):
    
    def __init__(self, resource, screen_width, screen_height):
        super().__init__(resource, scale=0.125)

        # Drone mass [Kg]
        self.m = 1.2
        # Drone thrust [N]
        self.T = 0.0
        
        # Coordinates for proper rendering
        self.base = 20
        self.max_x = screen_width - self.base
        self.max_y = screen_height - self.base
        
        # Initial position
        self.center_x = screen_width / 6
        self.center_y = self.base

        # Drone velocity
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.max_vel_x = 2.5
        self.max_vel_y = 1.5

    # Method to render drone
    def render(self):
        self.draw()

    # Method to update drone's vertical velocity based on thrust and gravity
    def update_vertical_velocity(self, delta_time: float):
        self.vel_y += ((self.T - G)) * delta_time / self.m
        if self.vel_y >= self.max_vel_y:
            self.vel_y = self.max_vel_y
        if self.vel_y < -self.max_vel_y:
            self.vel_y = -self.max_vel_y

    def set_thrust(self, thrust: float):
        self.T = thrust

    def set_speed(self, speed: float):
        self.vel_x = speed
        if self.vel_x > self.max_vel_x:
            self.vel_x = self.max_vel_x
        if self.vel_x < -self.max_vel_x:
            self.vel_x = -self.max_vel_x

    def get_position_and_velocity(self):
        return self.center_x, self.center_y, self.vel_x, self.vel_y

    def update(self, delta_time):
        # If drone is on the ground and no thrust is applied, stay still...
        if self.center_y <= self.base and abs(self.T) <= G:
            self.center_y = self.base
            self.vel_y = 0.0
            self.vel_x = 0.0
        # Else, update velocity based on thrust and gravity
        else:
            self.update_vertical_velocity(delta_time)
        # Update position
        self.center_x += self.vel_x
        self.center_y += self.vel_y
        # Prevent drone from going outside boundaries
        if self.center_x >= self.max_x:
            self.center_x = self.max_x
            self.vel_x = 0.0
        if self.center_x <= self.base:
            self.center_x = self.base
            self.vel_x = 0.0
        if self.center_y >= self.max_y:
            self.center_y = self.max_y
            self.vel_y = 0.0

# Graspable box class
class Box(arcade.Sprite):

    def __init__(self, resource, scale, x, y):
        super().__init__(resource, scale)
        self.center_x = x
        self.center_y = y
        self.grasped = False
        self.grounded = True
        self.vel_y = 0.0

    def set_deposit_sprite(self, deposit_sprite):
        self.deposit_sprite = deposit_sprite

    def is_grasped(self):
        self.grasped = True
        self.grounded = False

    def is_released(self):
        self.grasped = False

    def update(self, delta_time):
        if not self.grasped:
            if not self.grounded:
                self.vel_y -= 2*G/3
                self.center_y += self.vel_y * delta_time
                is_touching_pedestal = arcade.check_for_collision(self, self.deposit_sprite)
                if is_touching_pedestal:
                    self.grounded = True
                    self.vel_y = 0.0


    

# Thread to run simulation in background
class SimThread(Thread):
   def __init__(self):
      Thread.__init__(self)
   
   def run(self):
      arcade.run()
            
# Main class representing simulation environment
class DroneSim(arcade.Window):

    def __init__(self, resources_path):
        super().__init__(WIDTH, HEIGHT, TITLE)
        arcade.set_background_color(arcade.color.LIGHT_GRAY)

        self.resources_path = resources_path
        self.thread = SimThread()

        self.grasped_idx = None
        self.offset = 30
        self.max_dist = 40

    def setup(self):
        self.scene = arcade.Scene()
        # Instantiate drone
        self.drone = Drone(self.resources_path + "/resource/drone.png", WIDTH, HEIGHT)
        # Create and randomize boxes on pedestals
        self.pedestals = arcade.SpriteList(use_spatial_hash=True)
        for i in range(1,5):
            pedestal_i = arcade.Sprite(self.resources_path + "/resource/pedestal.png", 0.5)
            pedestal_i.center_x = 200 * (i+1)
            pedestal_i.center_y = 15
            self.pedestals.append(pedestal_i)
        self.boxes = arcade.SpriteList(use_spatial_hash=True)
        locations = {1,2,3,4}
        for i in range(3):
            loc = random.choice(tuple(locations))
            box_i = Box(":resources:images/tiles/boxCrate_double.png", 0.25, 200 * (loc+1), 45)
            self.boxes.append(box_i)
            self.scene.add_sprite("Box {0}".format(i+1), box_i)
            locations.remove(loc)
        self.deposit = self.pedestals[list(locations)[0] - 1]
        self.scene.add_sprite("Deposit", self.deposit)
        for box in self.boxes:
            box.set_deposit_sprite(self.deposit)
        # Start background thread on setup completion
        self.thread.start()

    def on_draw(self):
        self.clear()
        self.drone.render()
        self.pedestals.draw()
        self.boxes.draw()

    def on_update(self, delta_time: float):
        # Update drone's state
        self.drone.update(delta_time)
        # Update state for individual boxes
        for box in self.boxes:
            box.update(delta_time)
            if box.grasped:
                box.center_x = self.drone.center_x
                box.center_y = self.drone.center_y - self.offset
            elif not box.grasped and not box.grounded:
                indices = {0,1,2}
                indices.remove(self.grasped_idx)
                for i in list(indices):
                    other = self.boxes[i]
                    is_colliding_with_other_box = arcade.check_for_collision(box, other)
                    if is_colliding_with_other_box:
                        box.grounded = True

    # Method to retrieve drone state
    def get_drone_state(self):
        return self.drone.get_position_and_velocity()

    # Method to update drone's thrust
    def set_drone_thrust(self, thrust: float):
        self.drone.set_thrust(thrust)

    # Method to update drone's linear speed
    def set_drone_speed(self, speed: float):
        self.drone.set_speed(speed)

    # Method to have drone grasp closest box
    def grasp(self):
        # Find closest box
        distances = []
        for box in self.boxes:
            bx = box.center_x
            by = box.center_y
            b = np.array([bx, by])
            dx = self.drone.center_x
            dy = self.drone.center_y
            d = np.array([dx, dy])
            distances.append(np.linalg.norm(b - d))
        if any(item < self.max_dist for item in distances):
            self.grasped_idx = np.argmin(distances)
            self.boxes[self.grasped_idx].is_grasped()
            return True
        else:
            return False

    # Method to release currently grasped box
    def release(self):
        if self.grasped_idx is not None:
            self.boxes[self.grasped_idx].is_released()
    


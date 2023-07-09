from .lib.drone_sim import DroneSim
import rclpy
from enum import Enum
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from sofar_drone_simulator_interface.msg import DroneState

#from sofar_drone_simulator_interface.srv import Locations, Vtol

from ament_index_python.packages import get_package_share_directory

# Enumerator for Vertical Take-off and Landing (VTOL) operations
class VtolEnum(Enum):
   TAKEOFF = 1
   LAND = -1

class DroneSimNode(Node):

    def __init__(self):
        super().__init__("drone_sim_node")

        # Drone simulator
        self.sim = DroneSim(get_package_share_directory("sofar_drone_simulator"))
        self.sim.setup()

        # Publisher for drone's state (position and linear velocity)
        self.state_pub = self.create_publisher(DroneState, "/drone/state", 10)
        self.state_pub_timer = self.create_timer(0.0333, self.on_state_timer_elapsed)

        # Publisher for TAKE-OFF or LAND command
        self.vtol_cmd_pub = self.create_publisher(Point, "/controller/goal", 10)

        # Subscribers for drone's control
        self.create_subscription(Float64, "/drone/control/thrust", self.on_thrust_received, 10)
        self.create_subscription(Float64, "/drone/control/speed", self.on_speed_received, 10)

        # Subscriber to grasp/ungrasp boxes
        self.create_subscription(Bool, "/drone/grasp", self.on_grasp_cmd, 10)

        # Service for retrieving useful locations
        #self.create_service(Locations, "/world/locations", self.on_locations_srv_request)

        # Service for executing take-off or landing operations
        #self.create_service(Vtol, "/drone/vtol", self.on_vtol_request)

        # Store initial drone's location for VTOL operations
        self.vtol_x = self.sim.drone.center_x

    def on_state_timer_elapsed(self):
        x, y, vx, vy = self.sim.get_drone_state()
        state_msg = DroneState()
        state_msg.position.x = float(x)
        state_msg.position.y = float(y)
        state_msg.velocity.x = float(vx)
        state_msg.velocity.y = float(vy)
        self.state_pub.publish(state_msg)

    def on_thrust_received(self, msg: Float64):
        self.sim.set_drone_thrust(msg.data)

    def on_speed_received(self, msg: Float64):
        self.sim.set_drone_speed(msg.data)

    def on_grasp_cmd(self, msg: Bool):
        if msg.data:
            if self.sim.grasp():
                self.get_logger().info("Drone picked up box!")
            else:
                self.get_logger().info("Nothing to grasp...")
        else:
            self.sim.release()

    '''
    # Return coordinates of drone's hovering location, boxes and deposit
    def on_locations_srv_request(self, request: Locations.Request, response: Locations.Response):
        response.hovering.x = self.vtol_x
        response.hovering.y = 200.0
        for box in self.sim.boxes:
            box_loc = Point()
            box_loc.x = float(box.center_x)
            box_loc.y = 80.0
            response.boxes.append(box_loc)
        response.deposit.x = float(self.sim.deposit.center_x)
        response.deposit.y = 200.0
        return response
    
    # Send controller's setpoint to satisfy VTOL request
    def on_vtol_request(self, request: Vtol.Request, response: Vtol.Response):
        command = VtolEnum(request.command.data)
        if command == VtolEnum.TAKEOFF:
            setpoint = Point()
            setpoint.x = self.vtol_x
            setpoint.y = 200.0
            self.vtol_cmd_pub.publish(setpoint)
        elif command == VtolEnum.LAND:
            setpoint = Point()
            setpoint.x = self.vtol_x
            setpoint.y = 20.0
            self.vtol_cmd_pub.publish(setpoint)
        return response
    '''


        
def main(args=None):
    
    rclpy.init(args=args)
    sim_node = DroneSimNode()

    print("Press Ctrl+C to exit...")
    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()
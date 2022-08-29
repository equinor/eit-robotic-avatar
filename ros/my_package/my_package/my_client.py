import rclpy
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from open_manipulator_msgs.msg import KinematicsPose
from rclpy.node import Node
from gpiozero import Motor, OutputDevice
import json
import socket

def map(value, outputMin, outputMax, inputMin = -0.5, inputMax = 0.5):
    # Figure out how 'wide' each range is
    inputSpan = inputMax - inputMin
    outputSpan = outputMax - outputMin

    # Convert the input range into a 0-1 range (float)
    valueScaled = float(value - inputMin) / float(inputSpan)

    # Convert the 0-1 range into a value in the output range.
    return round(outputMin + (valueScaled * outputSpan),3)

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

class Motors():
    def __init__(self):
        self.front_left = Motor(forward=20, backward=16)
        self.front_left_enable = OutputDevice(21)

        self.front_right = Motor(forward=13, backward=19)
        self.front_right_enable = OutputDevice(26)

        self.back_left = Motor(forward=23, backward=24)
        self.back_left_enable = OutputDevice(25)

        self.back_right = Motor(forward=17, backward=27)
        self.back_right_enable = OutputDevice(22)

    def tank_drive(self, l , r):
        self.front_left.value = l
        self.front_left_enable.on()

        self.front_right.value = r
        self.front_right_enable.on()

        self.back_left.value = l
        self.back_left_enable.on()
            
        self.back_right.value = r
        self.back_right_enable.on()

def proses_motor(data): 
    leftController = data['l']
    y = leftController['y']
    x = leftController['x']

    w = (1-abs(y))*(x) + x
    v = (1-abs(x))*(y) + y

    left = -(v-w)/2
    right = -(v+w)/2

    return (left, right)


class MyClient(Node):
    def __init__(self):
        super().__init__('my_client')
        
        # Create Service Clients
        self.cli_joint = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.cli_tool = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetJointPosition.Request()
        self.req_tool = SetJointPosition.Request()



    def send_request(self, data):
        # Translate data to radians
        rx = map(clamp(data['rx'], -0.5, 0.5), outputMin=1.57, outputMax=-1.57) 
        ry = map(clamp(data['ry'], -0.5, 0.5), outputMin=-1.57, outputMax=1.57)
        rz = map(clamp(data['rz'], -0.25, 0.25), outputMin=-0.01, outputMax=0.01)

        # # Set joint states
        self.req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.req.joint_position.position = [ry, -1.33, 0.367, rx + 0.93]
        self.req.path_time = 0.15

        # Set gripper/tool state
        self.req_tool.joint_position.joint_name = ['gripper']
        self.req_tool.joint_position.position = [rz - 0.004]
        self.req_tool.path_time = 0.15

        # Spin the nodes
        self.future_tool = self.cli_tool.call_async(self.req_tool)
        rclpy.spin_until_future_complete(self, self.future_tool)
        self.future = self.cli_joint.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

def main():
    HOST = "0.0.0.0"
    PORT = 6666 

    motors = Motors()

    rclpy.init()
    my_client = MyClient()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, PORT))

    while(rclpy.ok()):
        data, addr = s.recvfrom(512)
        data = json.loads(data)
        (l,r) = proses_motor(data)
        motors.tank_drive(l,r)
        response = my_client.send_request(data)
    my_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

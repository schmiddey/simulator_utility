import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist

from gamepad_bridge.gamepad_rcv import GamepadReceiver
from gamepad_bridge.joy import Joy



class JoyBridgeNode(Node):
  def __init__(self):
    super().__init__('joy_bridge_node')
    #parameters
    self.declare_parameter('host', '192.168.64.1')
    self.declare_parameter('port', 1336)
    self.declare_parameter('rate', 50.0)
    self.declare_parameter('max_linear_vel', 2.0)
    self.declare_parameter('max_angular_vel', 3.0)

    host = self.get_parameter('host').get_parameter_value().string_value
    port = self.get_parameter('port').get_parameter_value().integer_value
    rate = self.get_parameter('rate').get_parameter_value().double_value
    self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
    self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value

    #print params
    self.get_logger().info('host: {}'.format(host))
    self.get_logger().info('port: {}'.format(port))
    self.get_logger().info('rate: {}'.format(rate))
    self.get_logger().info('max_linear_vel: {}'.format(self.max_linear_vel))
    self.get_logger().info('max_angular_vel: {}'.format(self.max_angular_vel))




    self.pub_vel = self.create_publisher(Twist, 'joy_vel/cmd_vel', 10)
    self.timer = self.create_timer(1/rate, self.timer_callback)
    self.gamepad_rcv = GamepadReceiver(host=host, port=port, callback=self.joy_callback)


  def connect(self):
    self.gamepad_rcv.connect()

  def close(self):
    self.gamepad_rcv.close()

  def joy_callback(self, joy_msg: Joy):
    twist_msg = Twist()
    twist_msg.linear.x = (joy_msg.trigger_r - joy_msg.trigger_l) * self.max_linear_vel
    twist_msg.linear.y = joy_msg.stick_r_x * self.max_linear_vel
    twist_msg.angular.z = joy_msg.stick_l_x * self.max_angular_vel * -1
    self.pub_vel.publish(twist_msg)
    
  def timer_callback(self):
    self.gamepad_rcv.tick()




def main(args=None):
  rclpy.init(args=args)

  node = JoyBridgeNode()
  node.connect()
  #fix exception
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  node.close()

  node.destroy_node()
  # rclpy.shutdown()


if __name__ == '__main__':
    main()

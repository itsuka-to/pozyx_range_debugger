import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check



class RangeDebugger(Node):

    def __init__(self):
        super().__init__("range_debugger")
        self.range_pub = self.create_publisher(String, "range", 10)

        # serial port setting
        serial_port = "/dev/ttyACM0"
        seiral_port = get_first_pozyx_serial_port()
        if serial_port is None:
            print("No Pozyx connected. CHeck your USB cable or your driver!")
            quit()

        self.pozyx = PozyxSerial(serial_port)

        # remote and destination
        self.remote_id = None
        self.destination_id = 0x605b

        self.ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION
        self.range_timer_ = self.create_timer(
            1.0, self.range_callback
        )


    def range_callback(self):
        device_range = DeviceRange()
        status = self.pozyx.doRanging(
            self.destination_id, device_range, self.remote_id
        )
        if status == POZYX_SUCCESS:
            print(type(device_range))
            print(device_range)
            self.range_pub.publish(device_range)
        else:
            error_code = SingleRegister()
            status = self.pozyx.getErrorCode(error_code)
            if status == POZYX_SUCCESS:
                print("ERROR Ranging, local %s" %
                       self.pozyx.getErrorMessage(error_code))
            else:
                print("ERROR Ranging, couldn't retrieve local error")
    

    def publishMarkerArray(self, distance):
        """
        Visualization pozyx anchors for Rviz2

        Parameters
        ----------
        distance : float
        """




def main(args=None):
    rclpy.init(args=args)

    range_debugger = RangeDebugger()
    rclpy.spin(range_debugger)

    range_debugger.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from pypozyx import (PozyxSerial, PozyxConstants, Coordinates, PozyxRegisters, version, DeviceCoordinates,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, get_first_pozyx_serial_port, Quaternion)

from pypozyx.tools.version_check import perform_latest_version_check



class RangeDebugger(Node):

    def __init__(self):
        super().__init__("range_debugger")
        self.range_pub = self.create_publisher(String, "range", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "odometry/pozyx/markers", 10)
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
            0.02, self.range_callback
        )

        self.anchors = [
            DeviceCoordinates(0x605b, 1, Coordinates(   0, 0, 0)),  # test
            DeviceCoordinates(0x603b, 1, Coordinates(1000, 0, 0)),  # test
        ]


    def range_callback(self):
        for anchor in self.anchors:
            self.destination_id = anchor.network_id  # Update

            device_range = DeviceRange()
            status = self.pozyx.doRanging(
                self.destination_id, device_range, self.remote_id
            )
            if status == POZYX_SUCCESS:
                self.publishMarkerArray(device_range.distance, anchor)
                self.get_logger().info(f"{device_range.distance}")
            else:
                error_code = SingleRegister()
                status = self.pozyx.getErrorCode(error_code)
                if status == POZYX_SUCCESS:
                    print("ERROR Ranging, local %s" %
                        self.pozyx.getErrorMessage(error_code))
                else:
                    print("ERROR Ranging, couldn't retrieve local error")
    

    def publishMarkerArray(self, distance, anchor):
        """
        Visualization pozyx anchors for Rviz2

        Parameters
        ----------
        distance : float
        """

        pos_x = 0
        pos_y = 0.0
        pos_z = 0.0

        markerArray = MarkerArray()
        m_id = 0
        
        # marker of pozyx pos
        marker_pos = Marker()
        marker_pos.header.frame_id = "/pozyx"
        marker_pos.header.stamp = self.get_clock().now().to_msg()
        marker_pos.ns = "pozyx_pos"  # namespace
        marker_pos.id = m_id
        m_id += 1

        marker_pos.type = Marker.CUBE
        marker_pos.action = Marker.ADD
        marker_pos.lifetime.sec = 1
        marker_pos.scale.x = 0.07
        marker_pos.scale.y = 0.07
        marker_pos.scale.z = 0.02
        marker_pos.pose.position.x = pos_x / 1000
        marker_pos.pose.position.y = pos_y / 1000
        marker_pos.pose.position.z = pos_z / 1000
        marker_pos.pose.orientation.x = 0.0
        marker_pos.pose.orientation.y = 0.0
        marker_pos.pose.orientation.z = 0.0
        marker_pos.pose.orientation.w = 1.0
        marker_pos.color.r = 0.0
        marker_pos.color.g = 1.0
        marker_pos.color.b = 0.0
        marker_pos.color.a = 1.0
        markerArray.markers.append(marker_pos)

        # marker of pozyx distance
        marker_pos = Marker()
        marker_pos.header.frame_id = "/pozyx"
        marker_pos.header.stamp = self.get_clock().now().to_msg()
        marker_pos.ns = "pozyx_distance"  # namespace
        marker_pos.id = m_id
        m_id += 1
        
        marker_pos.type = Marker.SPHERE
        marker_pos.action = Marker.ADD
        marker_pos.lifetime.sec = 1
        marker_pos.scale.x = float(distance) * 2 / 1000
        marker_pos.scale.y = float(distance) * 2 / 1000
        marker_pos.scale.z = float(distance) * 2 / 1000
        marker_pos.pose.position.x = pos_x / 1000
        marker_pos.pose.position.y = pos_y / 1000
        marker_pos.pose.position.z = pos_z / 1000
        marker_pos.pose.orientation.x = 0.0
        marker_pos.pose.orientation.y = 0.0
        marker_pos.pose.orientation.z = 0.0
        marker_pos.pose.orientation.w = 1.0
        marker_pos.color.r = 0.0
        marker_pos.color.g = 0.5
        marker_pos.color.b = 0.5
        marker_pos.color.a = 0.5
        markerArray.markers.append(marker_pos)

        # marker of pozyx distance label
        marker_pos = Marker()
        marker_pos.header.frame_id = "/pozyx"
        marker_pos.header.stamp = self.get_clock().now().to_msg()
        marker_pos.ns = "pozyx_distance_label"  # namespace
        marker_pos.id = m_id
        m_id += 1
        
        marker_pos.type = Marker.TEXT_VIEW_FACING
        marker_pos.action = Marker.ADD
        marker_pos.lifetime.sec = 1
        marker_pos.scale.x = 1.0
        marker_pos.scale.y = 1.0
        marker_pos.scale.z = 1.0
        marker_pos.pose.position.x = pos_x / 1000
        marker_pos.pose.position.y = pos_y / 1000
        marker_pos.pose.position.z = (pos_z + float(distance)) / 1000
        marker_pos.pose.orientation.x = 0.0
        marker_pos.pose.orientation.y = 0.0
        marker_pos.pose.orientation.z = 0.0
        marker_pos.pose.orientation.w = 1.0
        marker_pos.color.r = 1.0
        marker_pos.color.g = 1.0
        marker_pos.color.b = 1.0
        marker_pos.color.a = 1.0
        marker_pos.text = f"{float(distance / 1000):.2f}m"

        markerArray.markers.append(marker_pos)

        # Publish markers!
        self.markers_pub.publish(markerArray)




def main(args=None):
    rclpy.init(args=args)

    range_debugger = RangeDebugger()
    rclpy.spin(range_debugger)

    range_debugger.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
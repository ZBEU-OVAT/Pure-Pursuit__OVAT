#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
import csv
import os

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # MAVROS SensorDataQoS ile uyumlu QoS - BEST_EFFORT
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos)

        self.output_file = os.path.expanduser(
            '~/gpsurus/src/gpspackage/config/waypoints_gps.csv')
        self.waypoints = []
        self.last_lat = None
        self.last_lon = None
        self.min_dist_deg = 0.000025  # yaklasik 2.5 metre (Virajlar icin sikilastirildi)

        self.get_logger().info("=== WAYPOINT KAYDEDICI BASLATILDI ===")
        self.get_logger().info(f"Kayit dosyasi: {self.output_file}")
        self.get_logger().info("Araci gezdirin, her 2.5 metrede bir otomatik kaydedecek.")
        self.get_logger().info("Durdurmak icin Ctrl+C")

    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        if lat == 0.0 and lon == 0.0:
            return

        if self.last_lat is None:
            self.save_point(lat, lon)
        else:
            dlat = abs(lat - self.last_lat)
            dlon = abs(lon - self.last_lon)
            dist = (dlat**2 + dlon**2) ** 0.5
            if dist >= self.min_dist_deg:
                self.save_point(lat, lon)

    def save_point(self, lat, lon):
        self.waypoints.append((lat, lon))
        self.last_lat = lat
        self.last_lon = lon
        self.get_logger().info(
            f"[{len(self.waypoints)}] Kaydedildi: lat={lat:.7f}, lon={lon:.7f}")

    def save_to_file(self):
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['latitude', 'longitude'])
            for lat, lon in self.waypoints:
                writer.writerow([f"{lat:.8f}", f"{lon:.8f}"])
        print(f"TOPLAM {len(self.waypoints)} WAYPOINT KAYDEDILDI -> {self.output_file}")


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_file()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

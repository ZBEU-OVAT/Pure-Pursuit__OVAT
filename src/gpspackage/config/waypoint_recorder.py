#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
import csv
import os
import math
import threading

MESAFE_ESIGI = 0.5  # metre — her 50cm'de bir kaydet

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

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
        self.recording = False

        self.get_logger().info("=== OTOMATIK WAYPOINT KAYDEDICI (50cm) ===")
        self.get_logger().info(f"Kayit dosyasi: {self.output_file}")
        self.get_logger().info("ENTER = kaydi baslat/durdur  |  q + ENTER = bitir ve kaydet")

        kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        kb_thread.start()

    def gps_to_dist(self, lat1, lon1, lat2, lon2):
        R = 6378137.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        dx = dlon * R * math.cos(math.radians(lat1))
        dy = dlat * R
        return math.hypot(dx, dy)

    def gps_callback(self, msg):
        if msg.status.status < 0:
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        lat = msg.latitude
        lon = msg.longitude

        if not self.recording:
            return

        if self.last_lat is None:
            self.last_lat = lat
            self.last_lon = lon
            self.waypoints.append((lat, lon))
            print(f"[1] Baslangic noktasi kaydedildi: {lat:.7f}, {lon:.7f}")
            return

        dist = self.gps_to_dist(self.last_lat, self.last_lon, lat, lon)
        if dist >= MESAFE_ESIGI:
            self.waypoints.append((lat, lon))
            self.last_lat = lat
            self.last_lon = lon
            print(f"[{len(self.waypoints)}] {lat:.7f}, {lon:.7f}  (+{dist:.2f}m)")

    def keyboard_loop(self):
        while True:
            try:
                key = input()
            except EOFError:
                break

            if key.strip().lower() == 'q':
                self.save_to_file()
                rclpy.shutdown()
                break
            else:
                self.recording = not self.recording
                if self.recording:
                    self.last_lat = None
                    self.last_lon = None
                    print(">>> KAYIT BASLADI — arac hareket etsin")
                else:
                    print(f">>> KAYIT DURDURULDU — {len(self.waypoints)} nokta birikti")

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

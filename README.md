# Pure Pursuit — GPS Tabanlı Otonom Araç Kontrolü

> **ZBEU OVAT** ekibinin geliştirdiği, ROS 2 üzerinde çalışan GPS destekli otonom araç yol takip sistemi.

---

## Proje Hakkında

Bu proje, bir otonom aracın GPS koordinatlarından oluşan bir rota boyunca hareket etmesini sağlayan **Pure Pursuit** kontrol algoritmasını gerçek donanım üzerinde uygular. Araç, MAVROS aracılığıyla alınan GPS verisini kullanarak önceden kaydedilmiş waypoint'leri takip eder. Direksiyon ve hız komutları **Teensy** mikrodenetleyicisine seri haberleşme (USB/ACM) üzerinden iletilir.

---

## Sistem Mimarisi

```
GPS (MAVROS)
     │
     ▼
pure_pursuit_node  ──► steering_angle_topic ──► teensy_comm_node ──► Teensy (Serial)
     │                                               │
     ├──► speed_on  (ROS2 Service)                  ├──► Direksiyon PWM
     └──► brake_on  (ROS2 Service)                  └──► Hız / Fren komutu
```

---

## Özellikler

- **Pure Pursuit** geometrik yol takip algoritması
- **GPS waypoint** tabanlı navigasyon (CSV formatı)
- Seçilebilir **duraklama noktası** (waypoint indeksi ve süre ayarı)
- Teensy ile **4800 baud** seri haberleşme, port otomatik tarama
- ROS 2 **Humble** uyumlu
- 3D araç modeli (`.stl`) ile RVIZ görselleştirme desteği

---

## Gereksinimler

| Gereksinim | Sürüm |
|---|---|
| ROS 2 | Humble |
| MAVROS | ≥ 2.x |
| C++ | 17 |
| Teensy | 4.x (4800 baud) |

ROS 2 bağımlılıkları:
```
rclcpp, sensor_msgs, nav_msgs, geometry_msgs,
std_srvs, visualization_msgs, std_msgs, tf2, robot_state_publisher
```

---

## Kurulum

```bash
# Workspace'i klonla
git clone https://github.com/ZBEU-OVAT/Pure-Pursuit__OVAT.git
cd Pure-Pursuit__OVAT

# Bağımlılıkları yükle
rosdep install --from-paths src --ignore-src -r -y

# Derle
colcon build

# Ortamı kaynak al
source install/setup.bash
```

---

## Kullanım

### 1. Waypoint Kaydet

Waypoint'leri CSV olarak kaydet (manuel veya `waypoint_recorder.py` ile):

```
src/gpspackage/config/waypoints_gps.csv
```

Format:
```
latitude,longitude
41.45035120,31.76340100
41.45035560,31.76339670
...
```

### 2. Parametreleri Ayarla

`src/gpspackage/src/pure_pursuit.cpp` içindeki sabitler:

```cpp
WHEELBASE        = 1.484;   // Araç dingil mesafesi (m)
LOOKAHEAD        = 3.0;     // İleri bakış mesafesi (m)
ARRIVAL_DIST     = 1.5;     // Waypoint'e varış toleransı (m)
STEER_MULTIPLIER = 1.2;     // Direksiyon kazancı
STEER_MIN        = 25;      // Minimum direksiyon değeri
STEER_MAX        = 165;     // Maksimum direksiyon değeri
STEER_CENTER     = 100;     // Merkez (düz) direksiyon değeri
STOP_WP_INDEX    = 15;      // Duraklama yapılacak waypoint (-1 = kapalı)
STOP_DURATION_S  = 7.0;     // Duraklama süresi (saniye)
```

### 3. Çalıştır

```bash
source install/setup.bash
ros2 run gpspackage pure_pursuit_node
# Ayrı terminalde:
ros2 run gpspackage teensy_comm_node
```

---

## Paket Yapısı

```
gpspackage/
├── src/
│   ├── pure_pursuit.cpp      # Ana kontrol düğümü
│   └── teensy_comm.cpp       # Teensy seri haberleşme düğümü
├── config/
│   ├── waypoints_gps.csv     # GPS waypoint listesi
│   ├── waypoint_recorder.py  # Waypoint kayıt aracı
│   └── ekf.yaml              # EKF filtre yapılandırması
├── meshes/
│   └── araba.stl             # 3D araç modeli (RVIZ)
├── include/
│   └── gpspackage/
├── CMakeLists.txt
└── package.xml
```

---

## Katkıda Bulunanlar

**ZBEU OVAT** — Zonguldak Bülent Ecevit Üniversitesi Otonom Araç Takımı

---

## Lisans

Bu proje akademik ve yarışma amaçlı geliştirilmiştir.

# ros2
# ROS 2 ile Akıllı Navigasyon: Ara Nokta ve Koridor Bulma Algoritması

Bu proje, bir TurtleBot3 robotunun, karmaşık ve engellerle dolu bir Gazebo simülasyon ortamında, önceden belirlenmiş bir hedefe akıllıca ulaşmasını sağlayan bir ROS 2 Python düğümüdür.

Proje, basit engelden kaçınma algoritmalarının yetersiz kaldığı, özellikle dar ve simetrik geçitler içeren ortamlarda başarılı olmak üzere tasarlanmıştır. Bu amaçla, iki aşamalı bir görev planlama stratejisi kullanır.

![Simülasyon Ortamı](https://i.imgur.com/b00f00.png)

---

## 🚀 Kullanılan Algoritma: Ara Nokta (Waypoint) Navigasyonu

Robotun başarısını garanti altına almak için, görev iki ana aşamaya bölünmüştür:

1.  **Aşama 1: Ara Noktaya Git (`GO_TO_WAYPOINT`)**
   * Robot, çalışmaya başladığında ilk olarak, başlangıçtaki zorlu ve dar geçidi güvenli bir şekilde aşmasını sağlayacak, stratejik olarak belirlenmiş bir **ara noktaya (waypoint)** odaklanır.
   * Bu aşamada robot, engelleri analiz etmek yerine, mümkün olan en düz ve kararlı yolda bu ara noktaya ilerler. Bu, başlangıçtaki en büyük zorluğun hatasız bir şekilde aşılmasını sağlar.

2.  **Aşama 2: Ana Hedefe Git (`GO_TO_FINAL_GOAL`)**
   * Ara noktaya ulaşıldığı anda, robot bu ilk görevi tamamlamış sayılır.
   * Sonrasında, çok daha akıllı olan **"Koridor Bulma"** algoritmasını devreye sokar. Bu algoritmada robot:
       * Lazer tarayıcı verilerini analiz ederek engellerin arasındaki **geçilebilir boşlukları** tespit eder.
       * Bu boşluklar arasından, ana hedefine en uygun yönde olanı seçer.
       * Seçtiği koridorun ortasına doğru pürüzsüz bir şekilde ilerler.
   * Bu yöntem, robotun ortamın geri kalanında dinamik ve akıllı bir şekilde yolunu bularak nihai hedefe ulaşmasını sağlar.

---

## 🛠️ Kurulum ve Gereksinimler

### Gereksinimler
* Ubuntu 20.04 veya 22.04
* ROS 2 (Foxy veya Humble)
* Gazebo Simülatörü
* TurtleBot3 Simülasyon Paketleri: `turtlebot3`, `turtlebot3_simulations`
* Python 3 ve `numpy` kütüphanesi

### Çalışma Alanı Kurulumu

1.  **ROS 2 Çalışma Alanı Oluşturun (eğer yoksa):**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   ```

2.  **Yeni Bir Paket Oluşturun:**
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_robot_package
   ```

3.  **Python Kodunu Yerleştirin:**
   * Bu projede oluşturduğumuz son Python kodunu, `obstacle_avoider.py` (veya benzer bir isimle) kaydedin.
   * Bu dosyayı `~/ros2_ws/src/my_robot_package/my_robot_package/` dizininin içine taşıyın.

4.  **Paketi Çalıştırılabilir Hale Getirin:**
   * `~/ros2_ws/src/my_robot_package/setup.py` dosyasını bir metin editörü ile açın.
   * `entry_points` bölümünü aşağıdaki gibi düzenleyin:
       ```python
       entry_points={
           'console_scripts': [
               'navigator_node = my_robot_package.obstacle_avoider:main',
           ],
       },
       ```
   * `setup.py` dosyasını kaydedin.

5.  **Çalışma Alanını Derleyin:**
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

---

## ▶️ Projeyi Çalıştırma

Projeyi çalıştırmak için **2 ayrı terminal** kullanmanız yeterlidir.

### 1. Terminal: Gazebo Simülasyonunu Başlatın
Bu terminal, robotun ve engellerin bulunduğu sanal dünyayı çalıştırır.

```bash
# Kendi simülasyon dünyanızı başlatan launch dosyasını kullanın
ros2 launch <paket_adiniz> <launch_dosyanizin_adi>.launch.py

2. Terminal: Navigasyon Düğümünü Çalıştırın
Bu terminal, robotu kontrol eden Python kodumuzu çalıştırır.
# Önce çalışma alanını tanıtın
cd ~/ros2_ws
source install/setup.bash

# Düğümü çalıştırın
ros2 run my_robot_package navigator_node

Bu adımlardan sonra, robotun simülasyon ortamında hareket ettiğini ve terminaldeki log mesajlarından hangi aşamada olduğunu takip edebilirsiniz.

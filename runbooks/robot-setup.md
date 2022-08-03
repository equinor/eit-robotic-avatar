# Robot Setup

## Setup Raspberry Pi
1. Flash new SD card with Ubuntu Server 20.04 (64-bit)
1. Connect the SD card to an external PI and connect to internet via an Ethernet cable
1. Default username and password is: ubuntu and ubuntu
1. Find IP address with: `ip a`
1. Connect to over ssh using: `ssh ubuntu@<pi ip>`
1. Update date and time with:
    1. `sudo apt install ntpdate`
    1. `sudo ntpdate time.nist.gov`
1. Ubuntu server updates automatically
    * You will get an error `Waiting for cache lock: Could not get lock /var/lib/dpkg/lock-frontend. It is held by process 2794 (unattended-upgr)` while running apt commands. Wait it out.
    * Or you can kill it with `sudo pkill unattended-upgr` and do a normal `sudo apt update` and `sudo apt upgrade`
1. Connect PI to WIFI [Ubuntu Guide](https://ubuntu.com/core/docs/networkmanager/configure-wifi-connections)
    1. Install network manager with: `sudo apt install network-manager`
    1. Connect using: `sudo nmcli d wifi connect <wifi name> password <password>`

## Install and Setup ROS
1. Move ssh session over to wifi and disconnect Ethernet

6. SSH in to the PI from computer on same network: `ssh ubuntu@10.10.10.10`
7. In terminal:  
    1. `sudo apt update`  
    1. `get the file from our repo named install_ros2_foxy.sh, DONT KNOW EXACTLY HOW TO DO THIS YET`  
    1. `chmod 755 ./install_ros2_foxy.sh`  
    1. `bash ./install_ros2_foxy.sh`  
8. Install dependent packages for OpenMANIPULATOR-X:  
    1. `sudo apt install ros-foxy-rqt* ros-foxy-joint-state-publisher`  
    1. `cd ~/colcon_ws/src/`  
    1. `git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git`  
    1. `git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git`  
    1. `git clone -b foxy-devel https://github.com/ROBOTIS-GIT/open_manipulator.git`  
    1. `git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git`  
    1. `git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git`  
    1. `git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git`  
    1. `cd ~/colcon_ws && colcon build --symlink-install`   
    1. Set USB latency to 1 ms using: `ros2 run open_manipulator_x_controller create_udev_rules`
9. `sudo apt install python3-pip`
10. `pip3 install gpiozero`
11. Set static ip by doing the following changes: (https://ubuntu.com/server/docs/network-configuration)  
`cd /etc/netplan/`  
`sudo nano 99_config.yaml`  
``` yaml
network:  
  version: 2  
  renderer: networkd  
  ethernets:  
    eth0:  
      addresses:  
        - 192.168.0.1/24
``` 
12. After making the changes, apply them in terminal: `sudo netplan apply`
13. Set static IP on your computer: (https://pureinfotech.com/set-static-ip-address-windows-10/#:~:text=How%20to%20Assign%20Static%20IP%20Address%20Using%20Settings,Windows%2010.%206%20...%20%28more%20items%29%20See%20More.)
14. Move SD card to robot and boot up
15. SSH into the robot over Wi-Fi
16. Turn on VR headseat and enter VR
17. In one terminal write: `ros2 run my_package my_client`
18. In anoher terminal write: `ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py`
19. The robot should now be fully funcional. 
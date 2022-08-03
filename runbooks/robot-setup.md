# Robot Setup
A ROS 2 robot platform for use in testing.
## Run-Book from scratch:
1. Flash new SD card with Ubuntu 20.04
2. Connect the SD card to an external PI and connect to internet via an Ethernet cable
3. In case time is wrong, update time: `sudo ntpdate time.nist.gov`
4. `sudo apt update && sudo apt upgrade`
5. Connect PI to WIFI (https://ubuntu.com/core/docs/networkmanager/configure-wifi-connections)
6. SSH in to the PI from computer on same network: `ssh ubuntu@10.10.10.10`
7. In terminal:  
`sudo apt update`  
`get the file from our repo named install_ros2_foxy.sh, DONT KNOW EXACTLY HOW TO DO THIS YET`  
`chmod 755 ./install_ros2_foxy.sh`  
`bash ./install_ros2_foxy.sh`  
8. Install dependent packages for OpenMANIPULATOR-X:  
`sudo apt install ros-foxy-rqt* ros-foxy-joint-state-publisher`  
`cd ~/colcon_ws/src/`  
`git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git`  
`git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git`  
`git clone -b foxy-devel https://github.com/ROBOTIS-GIT/open_manipulator.git`  
`git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git`  
`git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git`  
`git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git`  
`cd ~/colcon_ws && colcon build --symlink-install`   
`ros2 run open_manipulator_x_controller create_udev_rules`
9. `sudo apt install python3-pip`
10. `pip3 install gpiozero`
11. Set static ip by doing the following changes: (https://ubuntu.com/server/docs/network-configuration)  
`cd /etc/netplan/`  
`sudo nano 99_config.yaml`  
network:  
    version: 2  
    renderer: networkd  
    ethernets:  
        eth0:  
            addresses:  
                - 192.168.0.1/24  
12. After making the changes, apply them in terminal: `sudo netplan apply`
13. Set static IP on your computer: (https://pureinfotech.com/set-static-ip-address-windows-10/#:~:text=How%20to%20Assign%20Static%20IP%20Address%20Using%20Settings,Windows%2010.%206%20...%20%28more%20items%29%20See%20More.)
14. Move SD card to robot and boot up
15. SSH into the robot over Wi-Fi
16. Turn on VR headseat and enter VR
17. In one terminal write: `ros2 run my_package my_client`
18. In anoher terminal write: `ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py`
19. The robot should now be fully funcional. 
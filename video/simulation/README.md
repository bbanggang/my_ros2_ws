# simulation

How to download all of files in this repository.

$ git clone https://github.com/2sungryul/simulation.git

How to check vendor id and product id for usb devices

$ lsusb

![image](https://github.com/2sungryul/simulation/assets/67367753/9dc8b13c-c6cb-4fd4-b42f-721e83bc2311)

How to create udev rules for rplidar C1 and U2D2 on Jetson nano

$ wget  https://raw.githubusercontent.com/2sungryul/simulation/refs/heads/main/usb2uart.rules

$ cp usb2uart.rules /etc/udev/rules.d

$ sudo service udev reload

$ sudo service udev restart

$ sudo reboot

$ ls -l /dev/rplidar

![image](https://github.com/2sungryul/simulation/assets/67367753/e7b3d3a6-798b-4614-bf43-a63a0d0acb5f)

$ ls -l /dev/u2d2

![image](https://github.com/2sungryul/simulation/assets/67367753/c5de696c-532a-4518-8632-a345c956452c)

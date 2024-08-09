# Integrate Arduino with Unitree Go2 Robotic Dog's Jetson Orin Board
SO3E03, 08.10.2024

# 1. Objectives

## 1.1 Previous workflow

The topic of [the previous note](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E02_unitree_go2_arduino.md) was to prepare an Arduino device with a button and a laser pen. As shown in figure 1.1, the workflow consists of the following steps:

1. Connect two functional modules to the Arduino Uno microcontroller, one is the HW483 button module, and the other is the HW493 laser module.
   
2. Connect the Arduino to the computer.
   
3. Install the Arduino IDE on the computer, write a C program in the IDE, compile it, and deploy it to the Arduino microcontroller.
   
4. Write a Python program on the computer using the VSCode IDE or PyCharm IDE, the Python program will run on the computer.
   
5. When the user presses the button, the button module sends a signal to the Arduino microcontroller. The C program running on the Arduino receives the button signal, performs preliminary processing, and then forwards the preliminary result to the computer.
   
6. The Python program running on the computer, after receiving the preliminary result forwarded by the Arduino, performs further processing and then sends a laser command to the Arduino.
   
7. The C program running on the Arduino microcontroller, after receiving the laser command, performs further processing and then sends the final laser command to the laser module.
   
8. The laser module emits or turns off the laser according to the command.

![Figure 1.1 Arduino device connected to a computer](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0101_computer_arduino.png "Figure 1.1 Arduino device connected to a computer")


## 1.2 Final workflow

The topic of this note is to replace the computer in figure 1.1 with a Unitree Go2 robotic dog, as shown in the 
figure 1.2.

![Figure 1.2 Arduino device connected to a unitree dog](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0102_unitree_arduino_standalone.png "Figure 1.2 Arduino device connected to a unitree dog")

The Unitree Robotic Dog EDU version comes with an Nvidia Jetson Orin NX board installed on the back of the dog. The operating system running on this Orin board is Ubuntu 20.04.

The workflow for this note is as follows:

1. Connect two functional modules to the Arduino Uno microcontroller, one is the HW483 button module, and the other is the HW493 laser module.
2. First, connect the Arduino to the computer.
3. Install the Arduino IDE on the computer, write a C program in the IDE, compile it, and deploy it to the Arduino microcontroller.
4. Write a Python program on the computer using the VSCode IDE or PyCharm IDE, then deploy the Python program to the robotic dog's Orin board.
5. Disconnect the Ethernet cable connecting the computer and the robotic dog's Orin board, and disconnect the Ethernet cable connecting the computer and the Arduino microcontroller.
6. Connect the Arduino microcontroller to the robotic dog's Orin board with an Ethernet cable.
7. When the user presses the button, the button module sends a signal to the Arduino microcontroller. The C program running on the Arduino receives the button signal, performs preliminary processing, and then forwards the preliminary result to the robotic dog's Orin board.
8. The Python program running on the robotic dog's Orin board, after receiving the preliminary result forwarded by the Arduino, performs further processing and then sends a laser command to the Arduino.
9. The C program running on the Arduino microcontroller, after receiving the laser command, performs further processing and then sends the final laser command to the laser module.
10. The laser module emits or turns off the laser according to the command.


## 1.3 Technical challenges

In the workflow of this article, the technical challenges lie in step 4. The Nvidia Jetson Orin NX board installed on the back of the Unitree robotic dog has no screen, no keyboard or mouse, and no internet access. Hence, we need to address these two challenges, 

1. How to log in to the Orin board and perform various operations without a screen, keyboard, or mouse.
2. How to enable internet access for the Orin board.



# 2. Access Orin

The Unitree Go2 EDU version of the robotic dog comes with an Nvidia Jetson Orin NX board mounted on its back. This Orin board does not have a screen, keyboard, or mouse, nor does it have an internet device. However, it has an RJ45 Ethernet port, a USB Type-A port and a USB Type-C port, as well as a 5V-3A|12V-3A DC power port, as shown in  figure 2.1.

How to log in and use the Orin board that lacks a screen, keyboard, and mouse? We have tried three methods.

![Figure 2.1 The orin ports](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0201_jetson_ports.jpg "Figure 2.1 The ports on the Orin board")

## 2.1 SSH to Orin

Using an Ethernet cable, connect the Orin board to a computer, and then upload programs to the Orin board using SCP and log in to the board using SSH to run the programs.

Here are an issue we encountered when connecting an Ethernet cable to the Orin board, and our solution to it:

1. The RJ45 port on the Orin board may have poor contact.
   
2. Normally, after the Ethernet cable is connected to both the Orin board and the computer, the signal light on the RJ45 port of the Orin motherboard should blink rapidly. The blinking of the signal light indicates that the data communication between the Orin motherboard and the computer is normal.
   
3. If the signal light does not blink, you may try cleaning the dust inside the RJ45 port or replacing the Ethernet cable.


### 2.1.1 Setup IP address

When the Unitree Go2 robotic dog is shipped, it comes with several preset private IP addresses. One of the private IP addresses is for the dog's body, 192.168.123.161, and the private IP address for the Orin board is 192.168.123.18.

If you want to log in to the Orin board from a computer using an Ethernet cable, you need to set the IP address of the computer's Ethernet port to 192.168.123.xx, ensuring that it is on the same subnet as the Orin board. Here, "xx" should be replaced with any number between 2 and 254 that is not already in use on the network, to avoid IP conflicts.

~~~
# Previously the IP address of the computer ethernet port is 10.42.0.1
$ ifconfig
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:57:aa:a4:58  txqueuelen 0  (Ethernet)
        ...

enx207bd51a15b6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.42.0.1  netmask 255.255.255.0  broadcast 10.42.0.255
        inet6 fe80::1908:422f:6af:35e6  prefixlen 64  scopeid 0x20<link>
        ether 20:7b:d5:1a:15:b6  txqueuelen 1000  (Ethernet)
        RX packets 572  bytes 35263 (35.2 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 252  bytes 42204 (42.2 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        ...

wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.120  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::2585:b1c9:fad1:2c02  prefixlen 64  scopeid 0x20<link>
        ether 70:d8:23:b8:39:36  txqueuelen 1000  (Ethernet)
        ...


# The following command set the IP address of the computer's ethernet port to 192.168.123.117
$ sudo ifconfig enx207bd51a15b6 192.168.123.117 netmask 255.255.255.0 up

# Verify that the IP address of the computer's ethernet port has being changed to 192.168.123.117
$ ifconfig
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:57:aa:a4:58  txqueuelen 0  (Ethernet)
        ...

enx207bd51a15b6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.123.117  netmask 255.255.255.0  broadcast 192.168.123.255
        inet6 fe80::1908:422f:6af:35e6  prefixlen 64  scopeid 0x20<link>
        ether 20:7b:d5:1a:15:b6  txqueuelen 1000  (Ethernet)
        RX packets 619  bytes 37425 (37.4 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 263  bytes 44658 (44.6 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        ...

wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.120  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::2585:b1c9:fad1:2c02  prefixlen 64  scopeid 0x20<link>
        ether 70:d8:23:b8:39:36  txqueuelen 1000  (Ethernet)
        ...
~~~


After then, verify that the computer can communicate with the Orin board,
~~~
# Ping the IP address of the robotic dog's body.
$ ping 192.168.123.161
PING 192.168.123.161 (192.168.123.161) 56(84) bytes of data.
^C
--- 192.168.123.161 ping statistics ---
5 packets transmitted, 0 received, 100% packet loss, time 4096ms

# Ping the IP address of the orin board on the back of the dog. 
$ ping 192.168.123.18
PING 192.168.123.18 (192.168.123.18) 56(84) bytes of data.
^C
--- 192.168.123.18 ping statistics ---
4 packets transmitted, 0 received, 100% packet loss, time 3066ms
~~~


### 2.1.2 SSH and SCP
1. Using SSH to access the Orin board, with login name and password: unitree/123

~~~
$ ssh unitree@192.168.123.18
unitree@192.168.123.18's password: 
Welcome to Ubuntu 20.04.5 LTS (GNU/Linux 5.10.104-tegra aarch64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage

This system has been minimized by removing packages and content that are
not required on a system that users do not log into.

To restore this content, you can run the 'unminimize' command.

 * Introducing Expanded Security Maintenance for Applications.
   Receive updates to over 25,000 software packages with your
   Ubuntu Pro subscription. Free for personal use.

     https://ubuntu.com/pro

Expanded Security Maintenance for Applications is not enabled.

242 updates can be applied immediately.
199 of these updates are standard security updates.
To see these additional updates run: apt list --upgradable

34 additional security updates can be applied with ESM Apps.
Learn more about enabling ESM Apps service at https://ubuntu.com/esm

Last login: Fri Jan  2 08:30:52 1970 from 192.168.123.117
ros:foxy(1) noetic(2) ?
1

unitree@ubuntu:~$ 
~~~

2. Using SCP to upload files to the Orin board and download files, with login name and password: unitree/123

~~~
# From a computer, upload a file to the Orin board
$ scp -r -P22 <file on the computer> unitree@192.168.123.18:~/<file on the board>

# From the Orin board, download a file to the computer 
$ scp -r -P22 unitree@192.168.123.18:~/<file on the board> <file on the computer>
~~~


## 2.2 NoMachine Remote Desktop

Still using an Ethernet cable, connect the Orin board to a computer. Set the IP address of the computer's Ethernet port to 192.168.123.xx, ensuring it is on the same subnet as the Orin board. 

Then, on the computer, run the NoMachine client. [NoMachine is a remote desktop software](https://www.nomachine.com/getting-started-with-nomachine) that allows you to remotely operate the Nvidia board from the computer using the NoMachine interface.


## 2.3 Computer Peripherals

Plug a USB hub into the USB port, which turns the Orin board to be with multiple USB data interfaces as well as an HDMI high-definition screen interface. 

Then connect the keyboard and mouse to the USB interfaces and the screen to the HDMI interface. In this way, the Orin board is equipped with a keyboard, mouse, and screen, just like a complete computer.

![Figure 2.2 Plug a USB hub to the USB port on the Orin board](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0202_jetson_port_replica.jpeg "Figure 2.2 Plug a USB hub to the USB port on the Orin board")

Comparing method three with method two:

1. Method three provides a clearer system interface and faster response times during operation.

2. Method two involves using a remote desktop, which is simple to use and does not require a keyboard, mouse, or screen. 
   However, because it needs to synchronize the remote desktop in real time, there is a sense of lag and some stuttering during operation.


# 3. Connect Orin to Internet

To enable the Orin rboard to connect to the internet, we have tried three methods:

## 3.1 Using a WiFi card

This is the method recommended by Unitree. Plug [a WiFi card](https://item.jd.com/10104528292124.html) into the USB port of the Orin board to allow it to connect to the internet via Wi-Fi.

Before using the wifi card, you need to install the driver. During the installation process, we encountered many error warnings, but in the end, we successfully installed the driver. 

If the driver installation fails, it may be necessary to install other dependency packages first, which can be quite troublesome. Hence, we may need method 3.3, internet sharing method.

![Figure 3.1 A wifi network card](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0301_wifi_card.png "Figure 3.1 A wifi network card")


## 3.2 Using a 4G/5G mobile router

![Figure 3.2 A 4g/5g mobile router](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0302_mobile_router.png "Figure 3.2 A 4g/5g mobile router")

### 3.2.1 Usage

When the robotic dog enters an area without Wi-Fi, we can consider purchasing two devices:

1. [A WiFi card](https://item.jd.com/10104528292124.html), as mentioned in the first method, plug the WiFi card into the USB port of the Orin board.
  
2. [A 4g/5g mobile router](https://item.jd.com/10049596756645.html), which communicates with the Orin board via WiFi and connects to the internet using a 4g/5g mobile network.

### 3.2.2 WiFi Direct

A 4G/5G mobile router can not only provide internet access to dozens of dogs in the vicinity, but also offer WiFi Direct services for these dogs. 

When dogs A and B communicate, they can use the router to achieve WiFi point-to-point communication, significantly improving the communication efficiency between the dogs.


## 3.3 Internet sharing

Using an Ethernet cable to connect the Orin board to a computer allows the Orin board to access the internet indirectly through the computer. There are several articles online that describe the operational details of this method [ [1](https://askubuntu.com/questions/169473/sharing-connection-to-other-pcs-via-wired-ethernet) ][ [2](https://askubuntu.com/questions/171914/how-to-connect-share-your-internet-connection-wired-wireless) ][ [3](https://forums.developer.nvidia.com/t/internet-sharing-with-laptop-on-jetson-nano/180191) ].

The principle of indirect internet access through the computer seems straightforward, but we were not successful in our actual operation, and the root cause of the failure is still under investigation.

Here are the steps we followed in our operation:

### 3.3.1 On the computer

1. Execute the following commands in the CLI terminal of the computer,

~~~
$ sudo ifconfig enx207bd51a15b6 192.168.123.117 netmask 255.255.255.0 up

$ ifconfig
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:28:09:a2:a3  txqueuelen 0  (Ethernet)
        ...
        
enx207bd51a15b6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.123.117  netmask 255.255.255.0  broadcast 192.168.123.255
        inet6 fe80::256b:f72c:378c:9ecf  prefixlen 64  scopeid 0x20<link>
        ether 20:7b:d5:1a:15:b6  txqueuelen 1000  (Ethernet)
        RX packets 5291  bytes 504932 (504.9 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 3206  bytes 579858 (579.8 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        ...

wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.120  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::2585:b1c9:fad1:2c02  prefixlen 64  scopeid 0x20<link>
        ether 70:d8:23:b8:39:36  txqueuelen 1000  (Ethernet)
        ...

$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.1     0.0.0.0         UG    600    0        0 wlo1
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 wlo1
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.0.0     0.0.0.0         255.255.255.0   U     600    0        0 wlo1

$ ip route
default via 192.168.0.1 dev wlo1 proto dhcp metric 600 
169.254.0.0/16 dev wlo1 scope link metric 1000 
172.17.0.0/16 dev docker0 proto kernel scope link src 172.17.0.1 linkdown 
192.168.0.0/24 dev wlo1 proto kernel scope link src 192.168.0.120 metric 600 

$ arp -n
Address                  HWtype  HWaddress           Flags Mask            Iface
192.168.0.100            ether   98:97:cc:0b:01:01   C                     wlo1
192.168.0.101            ether   98:97:cc:0b:07:81   C                     wlo1
192.168.0.102            ether   98:97:cc:36:65:bf   C                     wlo1
192.168.0.103            ether   98:97:cc:36:63:17   C                     wlo1
192.168.0.1              ether   98:97:cc:0a:ec:39   C                     wlo1
192.168.0.104            ether   98:97:cc:36:63:d7   C                     wlo1
192.168.0.114            ether   fe:4f:5f:b9:ca:45   C                     wlo1
~~~

2. Open the the system setting software of Ubuntu OS of the computer,

![Figure 3.3 Network setting tool in the computer](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0303_computer_network_setting.png "Figure 3.3 Network setting tool in the computer")   


### 3.3.2 On the Orin board

1. Execute the following commands in the CLI terminal of the Orin board,

![Figure 3.4 Network setting commands of the Orin board](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0304_orin_network_setting_terminal.png "Figure 3.4 Network setting commands of the Orin board")

2. Open the the system setting software of Ubuntu OS of the Orin board,

![Figure 3.5 Network setting tool of the orin board](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0305_orin_network_setting_tool.png "Figure 3.5 Network setting tool of the orin board")

3. After the above operations, we verified the communication between the computer and the board, but failed.

![Figure 3.6 Test the network communication between the computer and the board](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0307_orin_ping.jpg "Figure 3.6 Test the network communication between the computer and the board")


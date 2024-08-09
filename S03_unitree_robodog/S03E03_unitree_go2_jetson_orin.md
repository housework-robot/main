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

![Figure 1.1 Arduino device connected to a computer](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0101_computer_arduino.png)


## 1.2 Final workflow

The topic of this note is to replace the computer in figure 1.1 with a Unitree Go2 robotic dog, as shown in the 
figure 1.2.

![Figure 1.2 Arduino device connected to a unitree dog](https://github.com/housework-robot/main/blob/main/S03_unitree_robodog/S03E03_src/0102_unitree_arduino_standalone.png)

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

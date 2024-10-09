# 1. Objectives

The objectives of this article is data visualization, making the motion status data of the Unitree robotic dog Go2 more intuitive.

The Unitree robotic dog Go2 provides SDKs in both C++ and Python languages. Third-party developers can use the SDK to obtain and control the motion status data of the robotic dog in real-time, including both low-level and high-level motion status data.

1. [Low-level motion status data](https://support.unitree.com/home/en/developer/Basic_services)

The robotic dog has a total of 4 legs, each with 3 joints, which are the hip joint, thigh joint, and calf joint, making a total of 12 joints. When third-party developers select the low-level motion control mode, they can directly access and control the rotation angles, acceleration, and so on, of all motors in all joints of the robotic dog.

Low-level motion data also includes the status of the battery, data from the IMU gyroscope and accelerometer, as well as data from the remote controller.

2. [High-level motion status data](https://support.unitree.com/home/en/developer/sports_services)

Third-party developers can, of course, achieve various actions/gaits of the robotic dog by controlling the rotation angles of each joint, including actions/gaits such as crawling, sitting, standing, walking, running, and jumping. However, the implementation is quite challenging.

Unitree's engineers have implemented these standard actions/gaits and encapsulated them into high-level motion control.

However, if third-party developers use high-level motion control, they must first enter the `SportModeState_` high-level motion control mode. Once in this mode, low-level motion control is no longer available to third-party developers to prevent interference from self-defined low-level actions with high-level standard actions.

In [Unitree's official tutorials](https://support.unitree.com/home/en/developer/Software_Interface_Services), high-level motion control is divided into two subcategories: one is [high-level motion control](https://support.unitree.com/home/en/developer/sports_services), and the other is [AI motion control](https://support.unitree.com/home/en/developer/AI_motion_service). However, to use either of these motion controls, one must first enter the `SportModeState_` high-level motion control mode, and the APIs for both are identical. Therefore, for the sake of simplicity in this article, we don't distinct high-level motion control from AI motion control.

Whether it is low-level motion control data or high-level motion control data, approximately ten readings can be taken per second, with each data packet containing dozens of data items. That is a large amount of data in a short time. 

Directly reading the data is not easy to discover data patterns, including the distribution of data, such as the range of data values, as well as the relationships among different data items, such as the relationship between speed and position, etc.

This article uses the [Matplotlib Python toolkit](https://matplotlib.org/) to visually display the data, allowing engineers to more intuitively understand the distribution and relationships of the data.

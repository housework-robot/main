# Migrate Stanford aloha from ROS1 to ROS2
SO1E01, 2024.02.18

# 1. Preparation

1. We use a linux desktop with GPU, [Lamdba Tensorbook](https://lambdalabs.com/deep-learning/laptops/tensorbook/specs).

2. [Ubuntu 22.04.3 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) is installed on this desktop. 

3. [Conda is uninstalled](https://docs.anaconda.com/free/anaconda/install/uninstall/), because it causes many troubles. 

4. Connection routing, 
   
   some people complain they cannot connnect to githubusercontent. A solution is to modify /etc/hosts file, and add githubusercontent's IP address, like 185.199.108.133 to it. 
   ~~~
    # /etc/hosts

    127.0.0.1       localhost
    127.0.1.1       robot-test
    185.199.108.133 raw.githubusercontent.com

    # The following lines are desirable for IPv6 capable hosts
    ::1     ip6-localhost ip6-loopback
    fe00::0 ip6-localnet
    ff00::0 ip6-mcastprefix
    ff02::1 ip6-allnodes
    ff02::2 ip6-allrouters
   ~~~



# 2. Install ROS2/Humble

1. Following [ROS2/Humble official website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), to install ROS2/Humble.

2. To verify the success of installation, and to learn ROS2 quickly, following chapters of ROS2 tutorials are helpful, 

    * [Configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#configuring-environment), 
  
        If environmental settings are not configured correctly, we may encounter many problems later on. Therefore, it is worthy to spend some time to get familiar with ROS2 environment configurations.  

    * Understanding [nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html), [topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html), [services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html), [actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html) are essential.

    * [Launching nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html), 
  
  
        because Stanford mobile aloha robot consists of multiple nodes, launch xml is the enterpoint to start up the Stanford aloha.

      

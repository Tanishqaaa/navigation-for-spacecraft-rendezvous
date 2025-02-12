# **Navigation algorithm for space rendezvous and docking** 

> **A service or chaser spacecraft from 100 km away moves towards a target and has sensors such as GPS, IMU, LIDAR and star tracker. It uses an unscented Kalman filter to better estimate system state. The algorithm runs the dynamics of the system in phases from 100 km to 5 km, 5 km to 2 m, and 2 m to docking (terminal conditions)**  

---
! *Work in progress*

## **Table of Contents**  
- [About The Project](#about-the-project)
- [Prerequisites](#prerequisites)  
- [Getting Started](#getting-started)    
- [Instruction for use](#instruction-for-use) 
- [License](#license)  
- [Contact](#contact)  
- [Acknowledgments](#acknowledgments)  

---

## **About The Project**  
### **Purpose**  
NewSpace involves debris removal, avoidance and in-orbit repairs or service of satellites. This project presents a solution to the navigation problem during rendezvous. The navigation problem in space is rather complicated, requiring multi-sensor integration, using different sensors in different phases of far-range or close-range rendezvous. The kinematics and dynamics of the model may/ may not change for each phase. Further, the solution needs to be tuned with biases and noise.
To achieve this, the following criteria were kept in mind:

### **Key Challenges/ Constraint**  
- 🛠️ Sensors have varying update rates, noise characteristics, and fields of view  
- 🛠️ Biases must be estimated and corrected in real-time, and noise must be filtered out without introducing lag or instability  
- 🛠️ Space is a highly uncertain environment with factors like radiation, debris, thermal variations
- 🛠️ Calibration errors or initialization inaccuracies can lead to significant navigation errors

### **Built With**  
List of frameworks/libraries/tools used:  
- [MATLAB 2024](https://de.mathworks.com/)  

---
### **Prerequisites**  
- Unscented Kalman Filter
- Sensor Modelling
- Orbital dynamics
- Coding with MATLAB
  
## **Getting Started**  

**Objective**: The predicted state should be close to the true state

### **Instruction for use**  
Instructions for setting up the project.  
1. Download all the files in the same folder  
2. Add it to the MATLAB path
3. Run main.m


## **License**  
This project is licensed as Personal Work. The contents of this repository, including all code, designs, and documentation, are for personal use only and may not be copied, distributed, or shared without explicit written permission from the author. 

---

## **Contact**  
Connect – [tanishqa_jk](https://www.linkedin.com/in/tanishqa-jk/) 

---

## **Acknowledgments**  
Shoutouts to those who helped:  
- Xie, Yongchun & Chen, Changqing & Liu, Tao & Wang, Min. (2021). Guidance, Navigation, and Control for Spacecraft Rendezvous and Docking: Theory and Methods. 10.1007/978-981-15-6990-6.   

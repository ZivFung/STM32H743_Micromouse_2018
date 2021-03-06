# STM32H743_Micromouse_2018
  This is the software running on my micromouse tracer's Microcontroller STM32H743. The Project is developed by Keil IDE and completed at December 2, 2018. Tracer attended the 39th All Japan Micromouse Contest-Classicmouse Contest at Tokyo Polytechnic University. He completed the maze search in 31.403 seconds at the contest.  
  
# Micromouse Contest
  Typically everybody has 5 chance to search the maze and the shortest search time will be recorded. It is useful that once your mouse completely search the maze with low speed, you can make your mouse record the whole maze. Then, the next turn, your mouse would find the shortest way by algorithm(tracer use flood fill algorithm) to get to the destination with his highest speed (because it is easier to control the mouse when all his future motion is known), which we called "rush".  

# Photo of Tracer
>![image](https://github.com/ZivFung/STM32H743_Micromouse_2018/blob/master/Tracer.JPG)   

# Video Records of Tracer's Successful Rush
  The first gif is a normal speed video of one successful rush. Tracer already searched the maze and found the optimized way to the destination.  
>![image](https://github.com/ZivFung/STM32H743_Micromouse_2018/blob/master/Rush.gif)    
	
  This is another successful rush of tracer, but the video is took at slow-motion mode.  
>![image](https://github.com/ZivFung/STM32H743_Micromouse_2018/blob/master/Rush_slowMotion.gif)  

# Directory Overview
"USER": this directory contains code for all the real function of tracer.  
"HARDWARE": this directory contains code for few divers of peripherals for STM32H743. This is copied from others'.  
"CORE": this directory contains core code for controller.  
"UCOSIII": this directory contains code of RTOS UCOS-III.  

# Hardware
MCU: STMicroelectronics STM32H743  
Infrared Emitter: SFH4550  
Infrared Transistor: TSL262   
IMU: ICM20602  
Motor: DC Coreless Motor  
Encoder: Optical Encoder  
Debug Communication: Bluetooth Module HC-06  

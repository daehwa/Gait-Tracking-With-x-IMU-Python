# Gait Tracking with x-imu Python
This is the Python code of the [Gait-Tracking-With-x-IMU](https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU) by [xioTechnologies](https://github.com/xioTechnologies), which originally run on MATLAB.


The project Gait-Tracking-With-x-IMU is for the foot tracking algorithm demonstrated in Seb Madgwick's "[3D Tracking with IMU](https://www.youtube.com/watch?v=6ijArKE8vKU)" video. The foot tracking is enabled through [dead reckoning](https://en.wikipedia.org/wiki/Dead_reckoning) and integral drift corrected for each time the foot hit the ground.
Please also see their [original post](http://www.x-io.co.uk/gait-tracking-with-x-imu/).


## Several Examples of Result 
I provide several results of this Python code, comparing with the results of the original code on MATLAB.

### spiralStairs_CalInertialAndMag.csv

Python (This repository)           |  MATLAB ([xioTechnologies](https://github.com/xioTechnologies)'s)
:-------------------------:|:-------------------------:
![](./image/3_4_python.png)  |  ![](./image/3_4_matlab.png)
![](./image/3_1_python.png)  |  ![](./image/3_1_matlab.png)
![](./image/3_2_python.png)  |  ![](./image/3_2_matlab.png)
![](./image/3_3_python.png)  |  ![](./image/3_3_matlab.png)

### straightLine_CalInertialAndMag.csv

Python (This repository)           |  MATLAB ([xioTechnologies](https://github.com/xioTechnologies)'s)
:-------------------------:|:-------------------------:
![](./image/1_4_python.png)  |  ![](./image/1_4_matlab.png)
![](./image/1_1_python.png)  |  ![](./image/1_1_matlab.png)
![](./image/1_2_python.png)  |  ![](./image/1_2_matlab.png)
![](./image/1_3_python.png)  |  ![](./image/1_3_matlab.png)

### stairsAndCorridor_CalInertialAndMag.csv

Python (This repository)           |  MATLAB ([xioTechnologies](https://github.com/xioTechnologies)'s)
:-------------------------:|:-------------------------:
![](./image/2_4_python.png)  |  ![](./image/2_4_matlab.png)
![](./image/2_1_python.png)  |  ![](./image/2_1_matlab.png)
![](./image/2_2_python.png)  |  ![](./image/2_2_matlab.png)
![](./image/2_3_python.png)  |  ![](./image/2_3_matlab.png)
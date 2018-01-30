# Spider motion analysis
## Motion Capture
Analysis of the motion of *Grammostola Rosea* spider. Motion was captured by fast camera at 125 fps and resolution of 1280x512.
Spider was moving in a especially designed tunnel with two mirrors on both sides which allowed to analyse motion in all three directions.
On each frame 50 markers were manually inserted (see picture bellow). File `analiza.xlsx` contains data of X and Y positions 
of all markers in all frames.

![alt text](https://github.com/KarolloS/Spyder-motion-analysis/blob/master/markery.png)

In order to be able to convert pixels into meters a few additional markers were inserted (see picture bellow). 
This data set is included in the file `wymiary.xlsx`.

![alt text](https://github.com/KarolloS/Spyder-motion-analysis/blob/master/wymiary.png)

## MatLab analysis
Analysis was performed using MatLab software (file `analiza.m`). Firstly, whole data set was filtered with Butterworth filter. 
Then, one motion cycle was determined. Cycle length, duration and average velocity was calculated.
Sequence of leg movement was obtained. Finally, four angles for each leg were defined: coxa-femur, femur-tibia, tibia-metatarsus
and between leg section plane and sagittal plane. Then, the way how these angles change during whole motion cycle was analysed. 

**Most important conclusions:**
* during motion, spider repeated some motion patterns (similar for all cycles)
* similarity between the movement of left and right legs (time shift - half of the one motion cycle)
* similarity in the motion of R1 and R2, L1 and L2
* similarity in the motion of R3 and R4, L3 and L4

Project was done as a part of Intermediate Engineering Project - „Investigation of spider's movement properties based on video analysis”
at Warsaw University Technology. It received the highest possible mark 5/5.

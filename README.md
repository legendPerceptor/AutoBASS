# :battery: + :robot: + :factory: = AutoBASS 
AutoBASS deploys Automated Battery Assembly System. The motivation of this project is to meet the need of scaling-up and accuate assembly of in-house i.e. non-commercially acquired cells in battery reaserach, therefore enables the data-driven studies on optimization of active materials, electrolyte formulations, processing and manufacturing in a reproducible manner for the investigation of new chemistries.

![](AutoBASS_2_overview_no_text.png)

### Building my own
Have a look at the stl files in the mechanical parts folder.
You will need
1. A Crimper
2. 3x Mecademic Meca 500 r.3
3. 2x Meca compatible Schunk Grippers
4. Jenny Science Linax Linear rail
5. Satorius rLine dispensing module
6. A powerful vacuum pump
7. 2x 2/2 way Solenoid valve
8. Ardurino Uno + Relay schield
9. Optical table
10. A 3D printer
11. Possibly our help
12. A very large glovebox

Please reach out to us if you are planning to set one up in your lab as there are some practical issues to be considered like the Athmosphere of your glovebox. We recommend to not use Argon as robots can fail very fast due to overheating issues. The GUI Bojing wrote makes a lot of the aligning tasks a lot easier (you will need this too).

Consider getting a glovebox with forced laminar flow as otherwise there can be issues of trapped gases.

The crimper is typically quite heavy and does not need to be fixed additonally. Overall you "just" need to assemble all parts and teach the robot the positions. We taught the robot ALL prositions manually.


### Lurking
Have a look at the data or code then? Or read the paper which ist published at [JOURNAL]


## Motivation
In battery research, manual coin cell assembly is still the most widely used methode to manufacture the in-house cells for testing, but the precise placement of electrodes and timing of electrolyte injection are challenging for researchers who manually perform the assembly inside of a glovebox. The small variations in manufacturing processe strongly impacts the intrinsic variability the overall system performance such as capacity, resistance and degradation rate between cells, which is a crucial issue that needs to be addressed while performing the data-driven stuides, therefore, we see it35 a pressing need to automate the assembly process, enabling the manufacturing of larger numbers of cells in a reproducible manner for the investigation of new chemistries. We, therefore, build the automatic battery assembly system (AutoBASS) which is capable of assmeblying up to 64 coin cells in a batch, the main parts of AutoBASS consist of two 6-axis robotic arms (Mecademic meca500 rev.2), a linear rail (Jenny Science Linax LXS 1800), and a programmable syring pump, providing a accuracy of placement in 0.2mm and that of electrolyte despersing in 1nL. The assembly procedure is accroding to the standard assembly procedure of Coin Cell 2032 recommonded by BIG-MAP. A graphic user interface is specificly designed for proper operating of the system. Image of electrodes upon placement and real-time record of events during operating of the system were created to help keep track of the variablilty of the cells.

## What's new
1. New electrolyte dispersing unit: can choose from up to 16 different electrolytes without cross-contamination
2. Multi-threading operation: perfom pick-and-place, crimping and despersing in synchornized manner
3. Active-imaging system: misgrab detection and self-correction
4. New UI: more user-friendly, easy to assgin assembly task

## Enviroment setup
AutoBASS is very easy-to-read and user-friendly,  besides hardware drivers you just need a working python 3.7 installation and the drivers for the corresponding hardware as well. If you are using mecademic robots you need to install their (awesome!) python driver too.

## Launch script
AutoBaSS.py script can automatically guide you through the procedure from scratch:
    Launch 'AutoBass.py'
    Click "Assembly Coin Cell"
    Click "Initialize system" and wait for the progress bar to finish
    Type in the position of cell you want to start and to finish in numbers (No.1 refers to the cell on the up-left corner, No.64 refers to the one on the down-right corner)
    Click "Start Assembly" and the procedure will start


## Acknowledgements

This project has received funding from the European Union’s [Horizon 2020 research and innovation programme](https://ec.europa.eu/programmes/horizon2020/en) under grant agreement [No 957189](https://cordis.europa.eu/project/id/957189).

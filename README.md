[//]: # (Image References)

[final_result]: ./results.jpg "Final Result"

# Overview
This project is the implementation of a simple 2D particle filter in C++

### Approach
The approach for the project was to simply follow the outlined structure in the ```ParticleFilter``` class. Changes were made to the following functions, ```init()```, ```prediction()```, ```dataAssociation()```, ```updateWeights()```, and ```resample()```. In each function, the TODO defined by the projects were followed.

### Modifications
Note that I made two modifications in the project template outside of the particle_filter.cpp file.

1. The ```Map``` class had a publically defined struct to store land mark meta data which was different than the struct ```LandmarkObs``` defined in helper_functions.h. This seemed like poor overall program architecture, and so it was changed such that the map is a vector of the same ```LandmarkObs``` used in the ```ParticleFilter``` class. This was beneificial in the method ```updateWeights()``` since when forming a new vector of in-range landmarks from the map, there was no need to construct a ```LandmarkObjs``` from a different data type, and I was able to simply pass the same structure around.

2. There were redundant versions of weights, one that was a member of the ```Particle``` struct, and a second set of weights that were defined as a stand alone vector of the ```ParticleFilter``` class. This second set of weights was redundant, and removed.

### Results
Using 256 particles, the error is approximately x=0.109m, y=0.101m, yaw=0.004 radians, and the execution time is approximately 50 time steps per second of execution time. Only minor attempts at improving efficiency were made in this project.


![alt text][final_result]

Note: The project rubric specifies the following, however this information is out of date, since these parameters are not actually defined in the main.cpp file.

1. **Accuracy**: my particle filter localized vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`,

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

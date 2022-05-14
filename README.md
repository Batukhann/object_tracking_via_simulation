# object_tracking_via_simulation
Object Tracking Via Simulation (Airsim + Unreal Engine)

First of all, Airsim and Unreal Engine installations need to be done.

https://microsoft.github.io/AirSim/


There are many environments in Unreal Engine's market where this algorithm can be tested. We worked in 3 different environments on this project.

- https://www.unrealengine.com/marketplace/en-US/product/city-park-environment-collection
- https://www.unrealengine.com/marketplace/en-US/product/vehicle-game
- https://www.unrealengine.com/marketplace/en-US/product/landscape-mountains

After the environment is established, the object to be tracked needs to be added. We decided to initialize a car for this project. You can access the assets of the vehicle in the Assets folder.

At the same time, some parameters of the drone need to be adjusted. We shared the settings we use in the Settings folder.

There are 3 scripts in the project. 
The Location Tracker python file tracks the object by knowing the location of the vehicle. We created a dataset using this script.
The Tracker w Vector Python file was written to track the vehicle by processing images.
There are scripts in the Helper Functions python file to draw graphs and perform various calculations.

Required environment and library settings
There should be drone named my_drone in airsim settings and car my_car in unreal engine environment.
OpenCV 4.5.5.64
Tornado 4.5.3
Numpy
MatPlotLib


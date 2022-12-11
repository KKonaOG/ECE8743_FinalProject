# ECE8743_FinalProject
Important Notes:
-	All Algorithms are contained within the folder 'Algorithms'
-	Algorithms/RRT/RRT.m
-	Algorithms/RRTStar/RRTStar.m
-   Algorithms/MVRRT/MVRRT.m
-	Algorithms/SVGRRT/SVGRRT.m
-	All Maps are contained within the folder 'Maps'
-	Maps/Maps1.m
-	Maps/Maps2.m
-	Maps/Maps3.m
-	There are two options for interfacing with algorithms and running them:
-	Main Mode (Main.m)
-	Analysis Mode (Analysis.m)

Helpers:
-	p_poly_dist.m finds the point on a polygon closest to a provided point
-	Authors: Michael Yoshpe, Alejandro Weinstein, and Eric Schmitz
-	CreateObstacle.m provides an interface to add obstacles to the obstacle array for a Map
-	CheckCollision.m used to very no collisions occur on a given trajectory


Each Map file is responsible for configuring the environment variables such robotStart, robotTarget, robotStepsize, and targetThreshold. It additionally configures environment obstacles.

Each Algorithm file is responsible for implementing its respective Algorithm (I.e. RRT.m is the RRT algorithm). ALGORITHMS CANNOT BE RUN DIRECTLY. The Main.m acts as a controlling manager for all algorithms, and as such, must be used for algorithm execution.

Analysis Mode simply iterates through every map and every algorithm a set number of times (Modify Line 11 to increase number of iterations). Additionally, it saves the workspace data for each run to the DataLog folder. The naming convention is as follows: AlgorithmName_Map#_Iteration#.mat

Main Mode provides the user with an interactive client allowing for the selection a desired map and algorithm. It will display the output to the user for analysis.

Both algorithms save a recording of the latest algorithm run to Graphics/Map#_Map_AlgorithmName_Algorithm.gif. If an increased number of frames per iteration is desired, reduce the value of line 68 (dataRefreshCount) to a lower value. 
The data update rate is calculated as follows: mod(AlgorithmIterations, dataRefreshCount) == 0

Please verify that the following folders are on your path if you have any issues:
-	Graphics
-	DataLog
-	Algorithms (and all sub-folders)
-	Maps
-	Helpers

Additionally, using the included .prj (MATLAB Project File) can aid in making sure necessary folders are on the path.

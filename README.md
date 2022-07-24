# Dijkstra Path Planning - Point Robot with Obstacles
Project - 03 (Phase 01) for the course ENPM661 - Planning for Autonomous Robots at the University of Maryland, College Park.

Implementation of the Dijkstra algorithm for path planning of a point robot with a clearance in a map consisting of convex and non-covex obstacles. 

## Team Members
- Kumara Ritvik Oruganti (okritvik@umd.edu)
- Adarsh Malapaka (amalapak@umd.edu)

## Required Libraries:  
* cv2 : To add arrows, lines or circles in the map at the desired coordinates. Version 4.20.0
* time: To calculate the running time for the A* algorithm.
* numpy: To define the obstacle map matrix. Version 1.17.4
* heapq: Heap Queue to store opened_list nodes
* copy


## Test Case 1: 
  [co-ordinates with respect to bottom left corner origin of the window] 
	<Format: (x-coord, y-coord)> 
	
	Start-Node: (16, 16)

	Goal-Node:  (350, 150)
<!-- 
### Node Exploration
<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/159383496-facc8bd3-9bfd-4b3f-8aa7-253a0aba1881.gif" width="400" height="250">
</p>
 -->

<!-- 
### Optimal Path
<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/159372263-9075bff4-e728-4403-9912-189b5e1658ca.png" width="400" height="250">
</p>
 -->

## Test Case 2: 
  [co-ordinates with respect to bottom left corner origin of the window] 
	<Format: (x-coord, y-coord)>
	
	Start-Node: (180, 215) 

	Goal-Node:  (360, 140)

### Node Exploration
<!-- <p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/159382089-91ac6bfe-8a6c-4b02-a2c5-bb6879efc6d5.gif " width="400" height="250">
</p>
 -->
### Optimal Path
<!-- <p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/159372566-d5347785-85e0-434f-a1ae-d713e76944fc.png " width="400" height="250">
</p>
 -->


### Note: 
The shapes in the map including the outer boudary walls have been bloated by robot clearance on all sides.


## Running the Code:
The code accepts the start and goal positions from the user through the command-line interface as mentioned below.

**Format/Syntax:**  
		python3 Dijkstra-pathplanning-KumaraRitvik-Oruganti.py


**For Test Case 1:**	
		
		python3 Dijkstra-pathplanning-KumaraRitvik-Oruganti.py

		Enter the X Coordinate of the Start Node: 16
		Enter the Y Coordinate of the Start Node: 16
		Enter the X Coordinate of the Goal Node: 350
		Enter the Y Coordinate of the Goal Node: 150

**For Test Case 2:**	
		
		python3 Dijkstra-pathplanning-KumaraRitvik-Oruganti.py

		Enter the X Coordinate of the Start Node: 180
		Enter the Y Coordinate of the Start Node: 215
		Enter the X Coordinate of the Goal Node: 360
		Enter the Y Coordinate of the Goal Node: 140

# Exiting From the Code:

1. Press any key after the visualization is done. Make sure that the image window is selected while giving any key as an input.

2. You can always use ctrl+c to exit from the program.

<!-- ### Note: 
During the visualization of the explored nodes, certain arrows might cross through small portions of the obstacle region. This happens when the width of the obstacle portion is lesser than the step size and since the code only checks if the current position and next position lies within the obstacle space and not other points in between, it does not neglect this path. 
	 -->

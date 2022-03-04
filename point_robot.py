"""
@author Kumara Ritvik Oruganti
@brief This code is written as part of the Project 2 of ENPM 661 to find a path for the point robot using Dijkstra Algorithm
There will be 8 action sets, Up, Down, Left, Right and Four Diagonals.
Cost for up, down, left, right actions is 1 and for diagonal actions, the cost is 1.4.

A geometrical obstacle map is given. For the given obstacle map, mathematical equations are developed and the path is found from a given start node to goal node.

"""
import copy
import numpy as np
import cv2

def take_inputs():
    """
    @brief: This function takes the initial node state and final node state to solve the puzzle
    :return: Initial state and final state of the puzzle to be solved
    """
    initial_state = []
    final_state = []
    while True:
        state = input("Enter the X Coordinate of Start Node: ")
        if(int(state)<0):
            print("Enter Valid X Coordinate")
            continue
        else:
            initial_state.append(int(state))
        state = input("Enter the Y Coordinate of Start Node: ")
        if(int(state)<0):
            print("Enter Valid Y Coordinate")
            continue
        else:
            initial_state.append(int(state))
        
        state = input("Enter the X Coordinate of Goal Node: ")
        if(int(state)<0):
            print("Enter Valid X Coordinate")
            continue
        else:
            final_state.append(int(state))

        state = input("Enter the Y Coordinate of Goal Node: ")
        if(int(state)<0):
            print("Enter Valid X Coordinate")
            continue
        else:
            final_state.append(int(state))
        break
    return initial_state,final_state

def draw_obstacles(canvas):
    # cv2.circle(canvas, (300,65),45,(255,0,0),-1)
    # cv2.fillPoly(canvas, pts = [np.array([[115,40],[36,65],[105,150],[80,70]])], color=(255,0,0)) #Arrow
    # cv2.fillPoly(canvas, pts = [np.array([[200,110],[235,130],[235,170],[200,190],[165,170],[165,130]])], color=(255,0,0)) #Hexagon
    
    height,width,_ = canvas.shape
    # print(shape)
    for i in range(width):
        for j in range(height):
            if(i-5<=0) or (i-395>=0) or (j-5 <=0) or (j-245>=0):
                canvas[j][i] = [255,0,0]

            if ((i-300)**2+(j-65)**2-(45**2))<=0:
                canvas[j][i] = [255,0,0]
            
            if (j+(0.57*i)-218.53)>=0 and (j-(0.57*i)+10.04)>=0 and (i-240)<=0 and (j+(0.57*i)-310.04)<=0 and (j-(0.57*i)-81.465)<=0 and (i-160)>=0:
                canvas[j][i] = [255,0,0]

            if ((j+(0.316*i)-71.1483)>=0 and (j+(0.857*i)-145.156)<=0 and (j-(0.114*i)-60.909)<=0) or ((j-(1.23*i)-28.576)<=0 and (j-(3.2*i)+202.763)>=0 and (j-(0.114*i)-60.909)>=0):
                canvas[j][i] = [255,0,0]
    return canvas

if __name__ == '__main__':
    # initial_state,final_state = take_inputs()
    # print(initial_state,final_state)

    canvas = np.ones((250,400,3),dtype="uint8")
    # canvas = draw_obstacles(canvas)
    # cv2.imshow("Canvas",canvas)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    

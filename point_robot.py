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
import heapq as hq
import time

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

def action_move_up(node,canvas):
    """
    @brief: This function checks if the upward movement is possible for the current position
    :param node: present node state
    :return: boolean (true if upward movement is possible, false otherwise) and the generated node(same state if next state is not possible)
    """
    next_node = copy.deepcopy(node)
    # if(canvas.shape[0] - (current_node[1]+1)>0) and (canvas[canvas.shape[0]-current_node[1]+1][current_node[0]][0]<255):
    #     current_node = [current_node[0], canvas.shape[0] - (current_node[1] + 1)]
    if(next_node[1]-1 > 0) and (canvas[next_node[1]-1][next_node[0]][0]<255):
        next_node[1] = next_node[1] - 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)

def action_move_down(node,canvas):
#     """
#     @brief: This function checks if the downward movement is possible for the current position
#     :param node: present node state
#     :return: boolean (true if downward movement is possible, false otherwise) and the generated node(same state if next state is not possible)
#     """
    next_node = copy.deepcopy(node)
    # if(canvas.shape[0] - (current_node[1]+1)>0) and (canvas[canvas.shape[0]-current_node[1]+1][current_node[0]][0]<255):
    #     current_node = [current_node[0], canvas.shape[0] - (current_node[1] + 1)]
    if(next_node[1]+1 < canvas.shape[0]) and (canvas[next_node[1]+1][next_node[0]][0]<255):
        next_node[1] = next_node[1] + 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)

def action_move_left(node,canvas):
#     """
#     @brief: This function checks if the left movement is possible for the current position
#     :param node: present node state
#     :return: boolean (true if left movement is possible, false otherwise) and the generated node(same state if next state is not possible)
#     """
    next_node = copy.deepcopy(node)
    # if(canvas.shape[0] - (current_node[1]+1)>0) and (canvas[canvas.shape[0]-current_node[1]+1][current_node[0]][0]<255):
    #     current_node = [current_node[0], canvas.shape[0] - (current_node[1] + 1)]
    if(next_node[0]-1 > 0) and (canvas[next_node[1]][next_node[0]-1][0]<255):
        next_node[0] = next_node[0] - 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)
def action_move_right(node,canvas):
#     """
#     @brief: This function checks if the right movement is possible for the current position
#     :param node: present node state
#     :return: boolean (true if upward movement is possible, false otherwise) and the generated node(same state if next state is not possible)
#     """
    next_node = copy.deepcopy(node)
    # if(canvas.shape[0] - (current_node[1]+1)>0) and (canvas[canvas.shape[0]-current_node[1]+1][current_node[0]][0]<255):
    #     current_node = [current_node[0], canvas.shape[0] - (current_node[1] + 1)]
    if(next_node[0]+1 < canvas.shape[1]) and (canvas[next_node[1]][next_node[0]+1][0]<255):
        next_node[0] = next_node[0] + 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)
def action_move_top_right(node,canvas):
#     """
#     @brief: This function checks if the right movement is possible for the current position
#     :param node: present node state
#     :return: boolean (true if upward movement is possible, false otherwise) and the generated node(same state if next state is not possible)
#     """
    next_node = copy.deepcopy(node)
    # if(canvas.shape[0] - (current_node[1]+1)>0) and (canvas[canvas.shape[0]-current_node[1]+1][current_node[0]][0]<255):
    #     current_node = [current_node[0], canvas.shape[0] - (current_node[1] + 1)]
    if(next_node[1]-1 > 0) and (next_node[0]+1 <canvas.shape[1]) and (canvas[next_node[1]-1][next_node[0]+1][0]<255):
        next_node[1] = next_node[1] - 1
        next_node[0] = next_node[0] + 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)
def action_move_bottom_right(node,canvas):
#     """
#     @brief: This function checks if the right movement is possible for the current position
#     :param node: present node state
#     :return: boolean (true if upward movement is possible, false otherwise) and the generated node(same state if next state is not possible)
#     """
    next_node = copy.deepcopy(node)
    if(next_node[1]+1 < canvas.shape[0]) and (next_node[0]+1 <canvas.shape[1]) and (canvas[next_node[1]+1][next_node[0]+1][0]<255):
        next_node[1] = next_node[1] + 1
        next_node[0] = next_node[0] + 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)

def action_move_bottom_left(node,canvas):
#     """
#     @brief: This function checks if the right movement is possible for the current position
#     :param node: present node state
#     :return: boolean (true if upward movement is possible, false otherwise) and the generated node(same state if next state is not possible)
#     """
    next_node = copy.deepcopy(node)
    if(next_node[1]+1 < canvas.shape[0]) and (next_node[0]-1 >0) and (canvas[next_node[1]+1][next_node[0]-1][0]<255):
        next_node[1] = next_node[1] + 1
        next_node[0] = next_node[0] - 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)

def action_move_top_left(node,canvas):
#     """
#     @brief: This function checks if the right movement is possible for the current position
#     :param node: present node state
#     :return: boolean (true if upward movement is possible, false otherwise) and the generated node(same state if next state is not possible)
#     """
    next_node = copy.deepcopy(node)
    if(next_node[1]-1 > 0) and (next_node[0]-1 >0) and (canvas[next_node[1]-1][next_node[0]-1][0]<255):
        next_node[1] = next_node[1] - 1
        next_node[0] = next_node[0] - 1 
        return True,tuple(next_node)
    else:
        return False,tuple(next_node)


def dijkstra(initial_state,final_state,canvas):
    open_list = []
    closed_list = {}
    back_track_flag = False
    hq.heapify(open_list)
    hq.heappush(open_list,[0,initial_state,initial_state])
    while(len(open_list)>0):
        # 0: cost, 1: parent node, 2: present node
        node = hq.heappop(open_list)
        print(node)
        closed_list[(node[2][0],node[2][1])] = node[1]
        present_cost = node[0]
        if list(node[2]) == final_state:
            back_track_flag = True
            print("Back Track")
            break

        flag,next_node = action_move_up(node[2],canvas)
        if(flag):
            # print("Move up")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        # print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1)<open_list[i][0]):
                            open_list[i][0] = present_cost+1
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
        
        flag,next_node = action_move_top_right(node[2],canvas)
        if(flag):
            # print("Move top right")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        # print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1.4)<open_list[i][0]):
                            open_list[i][0] = present_cost+1.4
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1.4, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
                
        flag,next_node = action_move_right(node[2],canvas)
        if(flag):
            # print("Move right")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        # print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1)<open_list[i][0]):
                            open_list[i][0] = present_cost+1
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
    
        flag,next_node = action_move_bottom_right(node[2],canvas)
        if(flag):
            # print("Move bottom right")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        # print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1.4)<open_list[i][0]):
                            open_list[i][0] = present_cost+1.4
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1.4, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
        
        flag,next_node = action_move_down(node[2],canvas)
        if(flag):
            # print("Move down")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        # print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1)<open_list[i][0]):
                            open_list[i][0] = present_cost+1
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
        
        flag,next_node = action_move_bottom_left(node[2],canvas)
        if(flag):
            # print("Move bottom left")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        # print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1.4)<open_list[i][0]):
                            open_list[i][0] = present_cost+1.4
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1.4, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
        
        flag,next_node = action_move_left(node[2],canvas)
        if(flag):
            # print("Move left")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1)<open_list[i][0]):
                            open_list[i][0] = present_cost+1
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
        
        flag,next_node = action_move_top_left(node[2],canvas)
        if(flag):
            # print("Move top left")
            if next_node not in closed_list:
                temp = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == list(next_node)):
                        # print("Found in Open list: ", open_list[i][2],next_node)
                        temp = True
                        if((present_cost+1.4)<open_list[i][0]):
                            open_list[i][0] = present_cost+1.4
                            open_list[i][1] = node[2]
                            hq.heapify(open_list)
                        break
                if(not temp):
                    hq.heappush(open_list,[present_cost+1.4, node[2], list(next_node)])
                    hq.heapify(open_list)
                    # print("Pushed",temp)
        
        hq.heapify(open_list)
        print("Open List length",len(open_list))
        print("Closed List Length", len(closed_list))
        # print("Closed List: ",closed_list)
    
    if(back_track_flag):
        back_track(final_state,closed_list,canvas)
    else:
        print("Solution Cannot Be Found")
        
            
def back_track(final_state,closed_list,canvas):
    keys = closed_list.keys()
    for key in keys:
        canvas[key[1]][key[0]] = [255,255,255]
        cv2.imshow("Back Tracking",canvas)
        cv2.waitKey(1)

if __name__ == '__main__':
    start_time = time.time()
    canvas = np.ones((250,400,3),dtype="uint8")
    canvas = draw_obstacles(canvas)
    # cv2.imshow("Canvas",canvas)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    initial_state,final_state = take_inputs()
    #Changing the cartesian coordinates:
    # initial_state[1] = canvas.shape[0] - initial_state[1]
    # final_state[1] = canvas.shape[0] - final_state[1]

    #Check if the initial and final states are in the obstacle space
    # print(initial_state,final_state)
    cv2.circle(canvas,tuple(initial_state),3,(0,255,0),-1)
    cv2.circle(canvas,tuple(final_state),3,(0,0,255),-1)
    cv2.imshow("Canvas",canvas)
    dijkstra(initial_state,final_state,canvas)
    end_time = time.time()
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("Code Execution Time: ",end_time-start_time)
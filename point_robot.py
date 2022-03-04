"""
@author Kumara Ritvik Oruganti
@brief This code is written as part of the Project 2 of ENPM 661 to find a path for the point robot using Dijkstra Algorithm
There will be 8 action sets, Up, Down, Left, Right and Four Diagonals.
Cost for up, down, left, right actions is 1 and for diagonal actions, the cost is 1.4.

A geometrical obstacle map is given. For the given obstacle map, mathematical equations are developed and the path is found from a given start node to goal node.

"""
import copy

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

if __name__ == '__main__':
    initial_state,final_state = take_inputs()
    print(initial_state,final_state)
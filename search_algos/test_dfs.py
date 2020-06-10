# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 1i:11:38 2019

@author: segonzalez

"""

import csv
from time import sleep

def findPoint(a,b):
    
    """
    Find the tracer intitial point
    """
    for i in range(0,len(a)):
        for j in range(0,len(a[i])):
            if (a[i][j] == b):
                return [i, j]

def scanCells(lab_matrix, pRow, pCol):

    """
    Scan the adyacent cells to decide weather the actual cell is a node or not
    """
    moves = []
    
    
    if( lab_matrix [pRow - 1][pCol] == ('-' or 'F') ):
        moves.append('UP')
    
    if( lab_matrix [pRow][pCol - 1] == ('-' or 'F') ):
        moves.append('LEFT')
    
    if( lab_matrix [pRow][pCol + 1] == ('-' or 'F') ):
        moves.append('RIGHT')
    
    if( lab_matrix [pRow + 1][pCol] == ('-' or 'F') ):
        moves.append('DOWN')
    
    
    if( len(moves) > 1):
        return [ True, moves[0]]
    elif( len(moves) == 1):
        return [ False, moves[0]]
    else:
        return [ False, '']

    
def moveToCell(next_move, pRow,pCol):
    
    if( next_move == 'UP' ):
        return [ pRow - 1, pCol]
    
    elif( next_move == 'LEFT' ):
        return [ pRow, pCol - 1]
    
    elif( next_move == 'RIGHT' ):
        return [ pRow, pCol + 1]
    
    elif( next_move == 'DOWN' ):
        return [ pRow + 1, pCol]
    

 
def printMat(a):
    
    for i in range(0,len(a)):
        print("".join(a[i]))

        
import csv
import numpy as np
 

lab_matrix = []
nodes = []

"""
Read the initial laberinth
"""
with open('test_dfs.csv') as csvfile:
    lab = csv.reader(csvfile, delimiter=',')
    lab_matrix = list(lab)

"""
The agent finds his coordinates and afterwards senses his environment
"""

[ pRow, pCol ] = findPoint(lab_matrix,'P')

complete = False

while(complete == False):
 
    """
    Occupy the current cell
    Explore adyacent ones
    Decide wheather the current cell is a node or not
    Decide to further explore the node of return to previous one
    Decide weather or not the requirements have be met
    Vizualice
    Decide to iterate
    """
    
    if(lab_matrix[pRow][pCol]=='F'):
        complete == True
    else:
        continue

    lab_matrix[pRow][pCol] = 'E'     
    
    [ node_state, next_move] = scanCells(lab_matrix, pRow, pCol)
      
    if (node_state):
        nodes.append([ pRow, pCol])
    
        if ( next_move):
            [ pRow, pCol] = moveToCell(next_move,pRow,pCol)
        else:
            nodes.remove(nodes[-1])
            [pRow, pCol] = nodes[-1]

    else:
        
        if ( next_move):
            [ pRow, pCol] = moveToCell(next_move,pRow,pCol)
        elif((nodes[-1]==[pRow,pCol])):
            nodes.remove(nodes[-1])
            [pRow, pCol] = nodes[-1]

        else:
            [pRow, pCol] = nodes[-1]
    
    printMat(lab_matrix)
    print(next_move)
    print(nodes)
    sleep(.5)
    
        
    
    

# -*- coding: utf-8 -*-
"""
Created on Wed Aug 12 12:21:48 2020

@author: yeapym
"""

depot = []
for ele in active_arcs:
    if ele[0] == 0:
        depot.append(ele)

def find_second_ind(first_ind):
    for ele in active_arcs:
        if ele[0] == first_ind:
            return ele[1]
        
path = []
for ele in depot:
    first_ind = ele[0]
    second_ind = ele[1]
    path = [first_ind, second_ind]
    while second_ind != 0:
        first_ind = second_ind
        second_ind = find_second_ind(first_ind)
        path.append([first_ind,second_ind])
        
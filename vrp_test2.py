# -*- coding: utf-8 -*-
"""

@author: yeapym
"""
import numpy as np
import matplotlib.pyplot as plt
from gurobipy import Model, GRB, quicksum

file = open("./data/vrp_200_16_1.txt")
input_data = file.read()
file.close()
lines = input_data.split('\n')
parts = lines[0].split()

n = int(parts[0])-1 # numbre of clients
m = int(parts[1])
N = [i for i in range(1, n+1)]
V = [0] + N
A = [(i, j) for i in V for j in V if i != j]
Q = int(parts[2])
k = [0] #depot

demand = []
points = []
for i in range(1, n+2):
    line = lines[i]
    parts = line.split()
    demand.append(int(parts[0]))
    points.append([float(parts[1]), float(parts[2])])

q = {i: demand[i] for i in range(1,len(N)+1)}
c = {(i, j): np.hypot(points[i][0]-points[j][0], points[i][1]-points[j][1]) 
      for i, j in A}
# rnd = np.random
# rnd.seed(0)

# n = 20  # numbre of clients
# xc = rnd.rand(n+1)*200
# yc = rnd.rand(n+1)*100

# N = [i for i in range(1, n+1)]
# V = [0] + N
# A = [(i, j) for i in V for j in V if i != j]
# c = {(i, j): np.hypot(xc[i]-xc[j], yc[i]-yc[j]) for i, j in A}
# Q = 20
# q = {i: rnd.randint(1, 10) for i in N}

mdl = Model('CVRP')

x = mdl.addVars(A, vtype=GRB.BINARY)
u = mdl.addVars(N, vtype=GRB.CONTINUOUS)

mdl.modelSense = GRB.MINIMIZE
mdl.setObjective(quicksum(x[i, j]*c[i, j] for i, j in A))

mdl.addConstrs(quicksum(x[i, j] for j in V if j != i) == 1 for i in N)
mdl.addConstrs(quicksum(x[i, j] for i in V if i != j) == 1 for j in N)
mdl.addConstrs(quicksum(x[i, j] for j in N) == m for i in V if i==0)
mdl.addConstrs(quicksum(x[i, j] for i in N) == m for j in V if j==0)

mdl.addConstrs((x[i, j] == 1) >> (u[i]+q[j] == u[j])
                for i, j in A if i != 0 and j != 0)
mdl.addConstrs(u[i] >= q[i] for i in N)
mdl.addConstrs(u[i] <= Q for i in N)


mdl.Params.MIPGap = 0.1
mdl.Params.TimeLimit = 60  # seconds
mdl.optimize()

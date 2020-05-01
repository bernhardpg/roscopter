import numpy as np
from drone import *

file = open("log.txt")
currentState = np.zeros(12)
thrust = np.zeros(4)
dt = 0.02

for i in range(12):
    line = file.readline().rstrip("\n")
    currentState[i] = float(line)
file.readline()
file.readline()

for i in range(4):
    line = file.readline().strip().rstrip("\n").replace(",", "")
    thrust[i] = float(line)

file.readline()

thrust_array = []

line = file.readline().rstrip("\n")
line_front = line.split(" = ")[0]

while line_front != "thrustPlan":
    line = file.readline().rstrip("\n")
    line_front = line.split(" = ")[0]

while line_front == "thrustPlan":
    thrust_array.append(np.array(line.split("=")[1].split()).astype(np.float))
    line = file.readline().rstrip("\n")
    line_front = line.split(" = ")[0]

d = Drone(currentState, 2.86, 0.07, 0.08, 0.12)
print(currentState)
print("---")
for i in range(len(thrust_array)):
    thrust_action = thrust_array[i]
    dState = d.getDrakeDelta(currentState, thrust_action[0], thrust_action[1], thrust_action[2], thrust_action[3])
    currentState += dt*dState
    print(str(currentState[0:3]) + str(currentState[3:6]) + str(currentState[6:9]) + str(currentState[9:12]))
print(thrust_array)
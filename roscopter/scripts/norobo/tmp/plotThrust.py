
def calcThrust(signal):
    return signal*signal*0.000015 + signal*-0.024451 + 9.002250

thrust = [0.2615, 1.26105, 2.565, 4.17, 6.075, 8.280, 10.785, 13.5905, 14.906, 14.96]
time_series = []
for i in range(10):
    time_series.append((i+1)*0.1)
import matplotlib.pyplot as plt
import math


# diff = []
#
#
# for i in range(len(thrust)-1):
#     diff.append(thrust[i+1]-thrust[i])
#
# plt.plot(diff)
# plt.show()
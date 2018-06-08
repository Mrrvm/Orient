import torch
import random
import math



file = open("quat_example.txt", "w")
q = [0, 0, 0, 0]
for t in range(30):
    q[0] = random.uniform(-1, 1)
    q[1] = random.uniform(-1, 1)
    q[2] = random.uniform(-1, 1)
    q[3] = random.uniform(-1, 1)
    a = math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])
    for l in range(4):
        q[l] = q[l]/a

    file.write(repr(q[0])+" "+repr(q[1])+" "+repr(q[2])+" "+repr(q[3])+"\n")

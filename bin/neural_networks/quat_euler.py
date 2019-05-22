import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import math



def non_linear_test_function(w, x, y, z):
    a1 = w*w+2*x+z+math.atan(x*y)
    a2 = 1-x-y+z+math.atan(math.cosh(w))
    a3 = math.exp(z)

    return a1, a2, a3


def quaternion_to_euler_angle(q0, q1, q2, q3):
    ysqr = q2 * q2

    t0 = +2.0 * (q0 * q1 + q2 * q3)
    t1 = +1.0 - 2.0 * (q1 * q2 + ysqr)
    alpha = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (q0 * q2 - q3 * q1)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    beta = math.degrees(math.asin(t2))

    t3 = +2.0 * (q0 * q3 + q1 * q2)
    t4 = +1.0 - 2.0 * (ysqr + q3 * q3)
    gamma = math.degrees(math.atan2(t3, t4))

    return alpha, beta, gamma


class Quat_Euler(nn.Module):

    def __init__(self):
        super(Quat_Euler, self).__init__()
        self.lin1 = nn.Linear(4, 8)
        self.lin2 = nn.Linear(8, 8)
        self.lin3 = nn.Linear(8, 3)

    def forward(self, data):
        aux1 = F.relu(self.lin1(data))
        aux2 = F.relu(self.lin2(aux1))
        aux3 = self.lin3(aux1)
        return aux3


model = Quat_Euler()
N_DATA=30
print(model)
criterion = nn.MSELoss(size_average=False)
optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
x = torch.empty(N_DATA, 4, dtype=torch.float)
y = torch.empty(N_DATA, 3, dtype=torch.float)

with open("quat_example.txt", "r") as f:
    data = f.readlines()

m = 0
for line in data:
    words = line.split()
    for j in range(4):
        x[m, j] = float(words[j])

    m = m + 1


for l in range(N_DATA):
    #[y[l, 0], y[l, 1], y[l, 2]] = quaternion_to_euler_angle(x[l, 0], x[l, 1], x[l, 2], x[l, 3])
    [y[l, 0], y[l, 1], y[l, 2]] = non_linear_test_function(x[l, 0], x[l, 1], x[l, 2], x[l, 3])

#print(x)
#print(y)

for t in range(500):
    # Forward pass: Compute predicted y by passing x to the model
    y_pred = model(x)

    # Compute and print loss
    loss = criterion(y_pred, y)
    print(t, loss.item())

    # Zero gradients, perform a backward pass, and update the weights.
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

z = torch.empty(1, 4, dtype=torch.float)
z[0, 0] = 0.5
z[0, 1] = 0.5
z[0, 2] = 0.4
z[0, 3] = 0.3
w = model(z)
#print(z)
print(w)

[a, b, c] = non_linear_test_function(0.5, 0.5, 0.4, 0.3)
print(a)
print(b)
print(c)


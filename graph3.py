# plot data1 data2 and data3 in three graphs next to each other

import matplotlib.pyplot as plt

f1 = 'gt.txt'
f2 = 'lm.txt'
f3 = 'nlm.txt'

x1 = []
y1 = []
with open(f1, 'r') as f:
    gt = f.readlines()
    for i in gt:
        x1.append(float(i.split()[0]))
        y1.append(float(i.split()[1]))

x2 = []
y2 = []
with open(f2, 'r') as f:
    lm = f.readlines()
    for i in lm:
        x2.append(float(i.split()[0]))
        y2.append(float(i.split()[1]))

x3 = []
y3 = []
with open(f3, 'r') as f:
    nlm = f.readlines()
    for i in nlm:
        x3.append(float(i.split()[0]))
        y3.append(float(i.split()[1]))

fig, axs = plt.subplots(1, 3, figsize=(15, 5))

axs[0].plot(x1, y1, 'ro-')
axs[0].set_title('Ground Truth')
axs[0].set_xlabel('X position')
axs[0].set_ylabel('Y position')

axs[1].plot(x2, y2, 'bo-')
axs[1].set_title('With LMS')
axs[1].set_xlabel('X position')
axs[1].set_ylabel('Y position')

axs[2].plot(x3, y3, 'go-')
axs[2].set_title('Without LMS')
axs[2].set_xlabel('X position')
axs[2].set_ylabel('Y position')

plt.tight_layout()
plt.show()



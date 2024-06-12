# read x, y datapoints from a file and calculate mean error between points

ground_truth = 'ground_truth.txt'
lms = 'lms.txt'
no_lms = 'no_lms.txt'

xg = []
yg = []
with open(ground_truth, 'r') as f:
    gt = f.readlines()
    # x,y coordinates
    for i in gt:
        xg.append(float(i.split()[0]))
        yg.append(float(i.split()[1]))

xl = []
yl = []
with open(lms, 'r') as f:
    lms = f.readlines()
    # x,y coordinates
    for i in lms:
        xl.append(float(i.split()[0]))
        yl.append(float(i.split()[1]))

xn = []
yn = []
with open(no_lms, 'r') as f:
    nolms = f.readlines()
    # x,y coordinates
    for i in nolms:
        xn.append(float(i.split()[0]))
        yn.append(float(i.split()[1]))

mean_error_lms = 0
mean_error_no_lms = 0

mse_lms = 0
mse_no_lms = 0

for i in range(len(xg)):
    mean_error_lms += abs(xg[i] - xl[i]) + abs(yg[i] - yl[i])
    mean_error_no_lms += abs(xg[i] - xn[i]) + abs(yg[i] - yn[i])

    mse_lms += (xg[i] - xl[i])**2 + (yg[i] - yl[i])**2
    mse_no_lms += (xg[i] - xn[i])**2 + (yg[i] - yn[i])**2

mean_error_lms /= len(xg)
mean_error_no_lms /= len(xg)

mse_lms /= len(xg)
mse_no_lms /= len(xg)

print("Mean error with LMS: ", mean_error_lms)
print("Mean error without LMS: ", mean_error_no_lms)

print("MSE with LMS: ", mse_lms)
print("MSE without LMS: ", mse_no_lms)



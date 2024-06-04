import matplotlib.pyplot as plt

f_name = 'odom.txt'

with open(f_name, 'r') as f:
    lines = f.readlines()

    xl = []
    yl = []
    # every line has a set of x and y coordinates (x, y), plot them as a line through all the points since it's a pose graph and therefore a path
    for line in lines:
        x, y, z = line.strip('()\n').split(', ')
        xl.append(float(x))
        yl.append(float(y))

    print(len(xl), len(yl))
    # for x, y in zip(xl, yl):
    #     plt.plot(float(x), float(y), 'ro')

    # plot a line
    plt.plot(xl, yl, 'ro')

    # this line is very spikey and sporadisch so we need to smooth it out
    # keep in mind that this is a pose graph, so there can be points with the same x value but different y values
    # we need to make sure that we don't remove those points

    # we can use a moving average to smooth out the line
    # we will use a window size of 5
    window_size = 200
    smoothed_x = []
    smoothed_y = []
    for i in range(len(xl) - window_size):
        smoothed_x.append(sum(xl[i:i + window_size]) / window_size)
        smoothed_y.append(sum(yl[i:i + window_size]) / window_size)

    plt.plot(smoothed_x, smoothed_y, 'b-')

    # title is "LiDAR path"
    plt.title("LiDAR path")
    # x-axis label is "x"
    plt.xlabel("x")
    # y-axis label is "y"
    plt.ylabel("y")

    plt.show()
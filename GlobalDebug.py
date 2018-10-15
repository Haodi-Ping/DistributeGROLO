from matplotlib import pyplot as plt
import numpy

times = 1
def show(points, robots):
    global times
    fig = plt.figure(times)
    print('points ',points)
    plt.xlim((-2, 10))
    plt.ylim((-2, 10))
    plt.scatter(points[:, 0], points[:, 1], c='b', label='real_Position')
    plotx = []
    ploty = []
    i = 0
    for r in robots:
        plotx.append(r.get_coord()[0])
        ploty.append(r.get_coord()[1])
        plt.annotate(s=i, xy=(r.get_coord()[0], r.get_coord()[1]), xytext=(-5, 5), textcoords='offset points')
        plt.annotate(s=i, xy=(points[i, 0], points[i, 1]), xytext=(-5, 5), textcoords='offset points')
        i = (i + 1) % len(robots)

#    plt.subplot('21' + times.__str__())
#     plt.title('num = num,NO coord , epoch 500, NO edge ')
    plt.scatter(plotx, ploty, c='r', label='estimate_Position')
    plt.plot(20, 20, 'b')
    plt.plot(0, 0, 'b')
    plt.legend()
    plt.show()

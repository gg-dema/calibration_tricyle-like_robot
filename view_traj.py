import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def analyze_original_dataset(path, ground_t=False):
    f = open(path)

    lines = f.read().splitlines()

    c = 0
    x = []
    y = []
    for l in lines:
        c += 1
        if c < 10:
            continue
        tokens = l.split(":")
        if ground_t:
            tracker_pose = tokens[-1].strip()  # sensor trajectory
        else:
            tracker_pose = tokens[-2].strip()  # the trajectory generated with the wrong parameters
        xy = tracker_pose.split(" ")
        x.append(float(xy[0]))
        y.append(float(xy[1]))

    x_np = np.asarray(x)
    y_np = np.asarray(y)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(x, y)
    ax.axis("equal")
    plt.show()


if __name__ == "__main__":
    import os
    print(os.getcwd())
    analyze_original_dataset(path='data/dataset.txt', ground_t=True)
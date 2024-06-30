import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def analyze_original_dataset(ground_t=False):
    f = open("../data/dataset.txt")

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
            tracker_pose = tokens[
                -2
            ].strip()  # the trajectory generated with the wrong parameters
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


def analyze_reconstructed_odometry(path="data/odometry_reconstructed.csv"):
    data = pd.read_csv(path, sep=";", header=0)
    print(data.columns)
    fig, axs = plt.subplots(1, 3)

    axs[0].plot(data["x"].to_numpy(), data["y"].to_numpy())
    axs[0].set_title("Position")

    axs[1].plot(data["steering_v"].to_numpy())
    axs[1].set_title("Steering Velocity")

    axs[2].plot(data["driving_v"].to_numpy())
    axs[2].set_title("Driving Velocity")

    plt.show()


if __name__ == "__main__":
    # analyze_original_dataset()
    analyze_reconstructed_odometry()

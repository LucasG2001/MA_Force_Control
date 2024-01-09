#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


def compute_rmse(hand_position, ee_position):
    difference = hand_position - ee_position
    rmse_norm = np.sqrt(np.mean(np.sum(difference ** 2, axis=1)))
    rmse_per_dimension = np.sqrt(np.mean(difference ** 2, axis=0))
    return rmse_norm, rmse_per_dimension


def compute_std_dev(hand_position, ee_position):
    std_dev = np.std(hand_position - ee_position, axis=0)
    return std_dev


def plot_data(file_path):
    data = np.loadtxt(file_path, delimiter=',')

    time = data[:, -1]
    hand_position = data[:, :3]
    ee_position = data[:, 3:6]

    plt.plot(time, hand_position, label='Hand Position')
    plt.plot(time, ee_position, label='EE Position')

    plt.xlabel('Time Count')
    plt.ylabel('Position')
    plt.legend()
    plt.title('Hand and EE Positions over Time')

    # Compute and print metrics
    rmse_norm, rmse_per_dimension = compute_rmse(hand_position, ee_position)
    std_dev = compute_std_dev(hand_position, ee_position)

    print(f"RMSE Norm: {rmse_norm:.4f}")
    print(f"RMSE per Dimension: {rmse_per_dimension}")
    print(f"Standard Deviation: {std_dev}")

    # Plot one standard deviation
    plt.fill_between(time, ee_position - std_dev, ee_position + std_dev, color='gray', alpha=0.3,
                     label='EE Position ± 1 Std Dev')

    plt.legend()
    plt.show()


if __name__ == "__main__":
    log_file_path = "data_log.txt"  # Update with your actual log file path
    plot_data(log_file_path)

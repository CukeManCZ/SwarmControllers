import sys
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import itertools
import numpy as np

# -------- CONFIGURABLE FIXED COLORS --------
DRONE_COLORS = {
    0: "green",
    1: "blue",
    2: "red",
    3: "orange",
    4: "purple",
    5: "brown",
}

# Maximum distance to show in the distance plot (meters)
MAX_DISTANCE = 3.0

def visualize_sequence(csv_path):
    # Load CSV
    df = pd.read_csv(csv_path)

    # Normalize time to start at 0
    df["time"] = df["time"] - df["time"].min()

    # Get unique drones
    drone_ids = sorted(df["uav_id"].unique())

    # Create figure with 3 plots
    fig = plt.figure(figsize=(18, 6))

    # ----------- 3D TRAJECTORY -----------
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.set_title("3D Trajectories")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_zlabel("Z")

    # ----------- FEEDBACK PLOT -----------
    ax2 = fig.add_subplot(132)
    ax2.set_title("Feedback vs Time")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Feedback")

    # ----------- DISTANCE PLOT -----------
    ax3 = fig.add_subplot(133)
    ax3.set_title("Pairwise Distances vs Time")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Distance [m]")
    ax3.set_ylim(0, MAX_DISTANCE)  # crop distances above MAX_DISTANCE

    start_handle = None
    end_handle = None
    drone_handles = []
    drone_labels = []

    # Store drone positions in dictionary for distance computation
    drone_positions = {drone_id: df[df["uav_id"] == drone_id][["px","py","pz"]].values for drone_id in drone_ids}
    time_array = df[df["uav_id"] == drone_ids[0]]["time"].values

    for drone_id in drone_ids:
        drone_data = df[df["uav_id"] == drone_id]

        color = DRONE_COLORS.get(drone_id, "black")

        # ---- 3D trajectory ----
        line, = ax1.plot(
            drone_data["px"],
            drone_data["py"],
            drone_data["pz"],
            color=color
        )
        drone_handles.append(line)
        drone_labels.append(f"Drone {drone_id}")

        # ---- Start position ----
        if start_handle is None:
            start_handle = ax1.scatter(
                drone_data["px"].iloc[0],
                drone_data["py"].iloc[0],
                drone_data["pz"].iloc[0],
                color="dimgray",
                marker="X",
                s=60,
                alpha=0.7,
                edgecolors="black",
                linewidths=0.5,
                label="Start"
            )
        else:
            ax1.scatter(
                drone_data["px"].iloc[0],
                drone_data["py"].iloc[0],
                drone_data["pz"].iloc[0],
                color="dimgray",
                marker="X",
                s=60,
                alpha=0.7,
                edgecolors="black",
                linewidths=0.5
            )

        # ---- Finish position ----
        if end_handle is None:
            end_handle = ax1.scatter(
                drone_data["px"].iloc[-1],
                drone_data["py"].iloc[-1],
                drone_data["pz"].iloc[-1],
                color="dimgray",
                marker="o",
                s=50,
                alpha=0.7,
                edgecolors="black",
                linewidths=0.5,
                label="Finish"
            )
        else:
            ax1.scatter(
                drone_data["px"].iloc[-1],
                drone_data["py"].iloc[-1],
                drone_data["pz"].iloc[-1],
                color="dimgray",
                marker="o",
                s=50,
                alpha=0.7,
                edgecolors="black",
                linewidths=0.5
            )

        # ---- Feedback vs time ----
        ax2.plot(
            drone_data["time"],
            drone_data["feedback"],
            color=color,
            label=f"Drone {drone_id}"
        )

    # Compute pairwise distances and crop them
    for (id1, id2) in itertools.combinations(drone_ids, 2):
        pos1 = drone_positions[id1]
        pos2 = drone_positions[id2]
        distances = np.linalg.norm(pos1 - pos2, axis=1)
        distances_cropped = np.clip(distances, 0, MAX_DISTANCE)
        ax3.plot(time_array, distances_cropped, label=f"Drones {id1}-{id2}")

    # Legends
    handles = [start_handle, end_handle] + drone_handles
    labels = ["Start", "Finish"] + drone_labels
    ax1.legend(handles=handles, labels=labels)
    ax2.legend()
    ax3.legend(fontsize=8)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualize_sequence.py path/to/sequence.csv")
        sys.exit(1)

    visualize_sequence(sys.argv[1])

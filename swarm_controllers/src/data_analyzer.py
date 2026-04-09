import os
import glob
import pandas as pd
import numpy as np
import itertools

# ================= CONFIG =================
FOLDER_PATH = "/home/traskos/ws_examples/src/swarm_controllers/data/run_2026-02-25_15-05-30"

# Where to save output tables
OUTPUT_FOLDER = FOLDER_PATH   # Change if you want different location

TESTS = ["sphere", "cube", "torus", "pyramid", "flat-sphere", "flat-cube", "large-torus"]
X_THRESHOLD = 0.25
START_INDEX = 2   # <-- Choose which CSV number to start from
# ==========================================


def compute_convergence_time(df, threshold):
    df = df.copy()
    df["abs_feedback"] = df["feedback"].abs()

    drone_ids = df["uav_id"].unique()
    times = sorted(df["time"].unique())

    for t in times:
        after_t = df[df["time"] >= t]

        converged = True
        for drone in drone_ids:
            drone_data = after_t[after_t["uav_id"] == drone]
            if not (drone_data["abs_feedback"] <= threshold).all():
                converged = False
                break

        if converged:
            return t

    return np.nan


def compute_min_distance(df):
    drone_ids = sorted(df["uav_id"].unique())

    positions = {
        drone_id: df[df["uav_id"] == drone_id][["px", "py", "pz"]].values
        for drone_id in drone_ids
    }

    min_distance = np.inf

    for (id1, id2) in itertools.combinations(drone_ids, 2):
        pos1 = positions[id1]
        pos2 = positions[id2]
        distances = np.linalg.norm(pos1 - pos2, axis=1)
        min_distance = min(min_distance, distances.min())

    return min_distance


def analyze_all():
    csv_files = sorted(
        glob.glob(os.path.join(FOLDER_PATH, "*.csv")),
        key=lambda x: int(os.path.basename(x).split(".")[0])
    )

    results = []

    # Filter files by START_INDEX
    filtered_files = []
    for file in csv_files:
        file_index = int(os.path.basename(file).split(".")[0])
        if file_index >= START_INDEX:
            filtered_files.append(file)

    for idx, file in enumerate(filtered_files):
        shape = TESTS[idx % len(TESTS)]

        df = pd.read_csv(file)
        df["time"] = df["time"] - df["time"].min()

        convergence_time = compute_convergence_time(df, X_THRESHOLD)
        min_distance = compute_min_distance(df)

        results.append({
            "shape": shape,
            "file": os.path.basename(file),
            "convergence_time": convergence_time,
            "min_distance": min_distance
        })

    return pd.DataFrame(results)


def summarize_results(df):
    summary = df.groupby("shape").agg(
        runs=("shape", "count"),
        convergence_mean=("convergence_time", "mean"),
        convergence_std=("convergence_time", "std"),
        convergence_success_rate=("convergence_time", lambda x: x.notna().mean()),
        min_distance_mean=("min_distance", "mean"),
        min_distance_std=("min_distance", "std"),
        min_distance_min=("min_distance", "min")
    ).reset_index()

    return summary


if __name__ == "__main__":
    all_results = analyze_all()
    summary_table = summarize_results(all_results)

    print(f"\nStarting from CSV index: {START_INDEX}")

    print("\n==== PER-RUN DATA ====")
    print(all_results)

    print("\n==== FINAL SUMMARY TABLE ====")
    print(summary_table)

    # Ensure output folder exists
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)

    summary_path = os.path.join(OUTPUT_FOLDER, "final_summary.csv")
    runs_path = os.path.join(OUTPUT_FOLDER, "per_run_results.csv")

    summary_table.to_csv(summary_path, index=False)
    all_results.to_csv(runs_path, index=False)

    print(f"\nSaved summary to: {summary_path}")
    print(f"Saved per-run results to: {runs_path}")
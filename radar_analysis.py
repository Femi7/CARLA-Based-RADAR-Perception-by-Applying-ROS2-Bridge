import struct
import re
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Step 1: Load and parse the radar_data.txt
with open("radar_data.txt", "r") as f:
    text = f.read()

pattern = re.compile(
    r"stamp:\s+sec:\s+(\d+)\s+nanosec:\s+(\d+).*?width:\s+(\d+).*?data:\s+\-([^\n]+(?:\n\s*-\s*[^\n]+)*)",
    re.DOTALL
)
matches = pattern.findall(text)

# Step 2: Extract frames
frames = []
for sec, nsec, width, data_block in matches:
    timestamp = int(sec) + int(nsec) * 1e-9
    data_list = list(map(int, re.findall(r'\d+', data_block)))
    byte_data = bytes(data_list)
    width = int(width)
    for i in range(width):
        offset = i * 16
        if offset + 16 > len(byte_data):
            continue
        x, y, z, velocity = struct.unpack('ffff', byte_data[offset:offset+16])
        frames.append({
            "timestamp": timestamp,
            "x": x,
            "y": y,
            "z": z,
            "velocity": velocity
        })

df = pd.DataFrame(frames)

# Step 3: Task 1 - 2D Motion Trajectory Plot
plt.figure(figsize=(10, 8))
sc = plt.scatter(df['x'], df['y'], c=df['timestamp'], cmap='plasma', s=10, alpha=0.7)
plt.colorbar(sc, label='Timestamp (s)')
plt.title("Task 1: 2D Object Trajectory Tracking")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.grid(True)
plt.axis("equal")
plt.tight_layout()
plt.savefig("task1_trajectory.png")
plt.close()

# Step 4: Task 2 - Velocity-Based Categorization
def categorize(v):
    if v == 0:
        return "Static"
    elif v <= 2:
        return "Low Speed"
    else:
        return "High Speed"

df["velocity_category"] = df["velocity"].apply(categorize)
summary = df["velocity_category"].value_counts().reset_index()
summary.columns = ["velocity_range", "count"]
summary["% of total"] = (summary["count"] / len(df) * 100).round(2)

# Plot bar chart
plt.figure(figsize=(8, 5))
sns.barplot(data=summary, x="velocity_range", y="count", palette="Set2")
plt.title("Task 2: Velocity-Based Motion Categorization")
plt.xlabel("Velocity Category")
plt.ylabel("Detection Count")
plt.tight_layout()
plt.savefig("task2_velocity_categorization.png")
plt.close()

# Save summary table
summary.to_csv("velocity_summary.csv", index=False)

# Step 5: Task 3 - RADAR Detection Density Heatmap
plt.figure(figsize=(10, 8))
plt.hist2d(df['x'], df['y'], bins=100, cmap="hot")
plt.colorbar(label="Detection Density")
plt.title("Task 3: RADAR Detection Density Heatmap")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.grid(True)
plt.axis("equal")
plt.tight_layout()
plt.savefig("task3_heatmap.png")
plt.close()

print("✅ Analysis complete. Output files:")
print("- task1_trajectory.png")
print("- task2_velocity_categorization.png")
print("- task3_heatmap.png")
print("- velocity_summary.csv")

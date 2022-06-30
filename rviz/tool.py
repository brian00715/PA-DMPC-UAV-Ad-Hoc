import yaml
import random
import sys

# Load the .rviz configuration from the file
with open("./crowds_path.rviz", "r") as file:
    config = yaml.safe_load(file)

# Find all the Path displays and update their topics and colors
path_displays = [display for display in config["Visualization Manager"]["Displays"] if display["Class"] == "rviz/Path"]

for i, path_display in enumerate(path_displays):
    path_display["Topic"] = f"/crowds/{i}_path"
    path_display["Color"] = f"{random.randint(0, 255)}; {random.randint(0, 255)}; {random.randint(0, 255)}"
    print(f"Updated path display {i} to topic {path_display['Topic']} with color {path_display['Color']}")

# Save the modified configuration back to the file
with open("crowds_path_updated.rviz", "w") as file:
    yaml.dump(config, file)

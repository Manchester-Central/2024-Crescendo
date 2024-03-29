import json
from pathlib import Path
from itertools import combinations
from string import Template
import csv

settings = json.loads(Path('../../.pathplanner/settings.json').read_text())
max_velocity = settings["defaultMaxVel"]
max_acceleration = settings["defaultMaxAccel"]
max_angular_velocity = settings["defaultMaxAngVel"]
max_angular_acceleration = settings["defaultMaxAngAccel"]
print(settings)

with open('./points.csv', 'r') as file:
    csv_reader = csv.DictReader(file)
    points = [row for row in csv_reader]
print(points)

all_pairs = list(combinations(points, 2))

print(all_pairs)

json_path = Template("""
{
  "version": 1.0,
  "waypoints": [
    {
      "anchor": {
        "x": ${start_pose_x},
        "y": ${start_pose_y}
      },
      "prevControl": null,
      "nextControl": {
        "x": ${start_heading_x},
        "y": ${start_heading_y}
      },
      "isLocked": false,
      "linkedName": null
    },
    {
      "anchor": {
        "x": ${end_pose_x},
        "y": ${end_pose_y}
      },
      "prevControl": {
        "x": ${end_heading_x},
        "y": ${end_heading_y}
      },
      "nextControl": null,
      "isLocked": false,
      "linkedName": "NF5"
    }
  ],
  "rotationTargets": [],
  "constraintZones": [],
  "eventMarkers": [
    {
      "name": "Start Intake",
      "waypointRelativePos": 0,
      "command": {
        "type": "parallel",
        "data": {
          "commands": [
            {
              "type": "named",
              "data": {
                "name": "intake"
              }
            }
          ]
        }
      }
    }
  ],
  "globalConstraints": {
    "maxVelocity": ${max_velocity},
    "maxAcceleration": ${max_acceleration},
    "maxAngularVelocity": ${max_angular_velocity},
    "maxAngularAcceleration": ${max_angular_acceleration}
  },
  "goalEndState": {
    "velocity": 0,
    "rotation": ${end_pose_rotation},
    "rotateFast": true
  },
  "reversed": false,
  "folder": "Templates",
  "previewStartingState": {
    "rotation": ${start_pose_rotation},
    "velocity": 0
  },
  "useDefaultConstraints": true
}
""")
print(json_path)

def createAutoPath(start_point, end_point):
    print("-----------")
    generated_path_string = json_path.substitute(
        max_velocity = max_velocity,
        max_acceleration = max_acceleration,
        max_angular_velocity = max_angular_velocity,
        max_angular_acceleration = max_angular_acceleration,
        start_pose_x = start_point["xMeters"],
        start_pose_y = start_point["yMeters"],
        start_pose_rotation = start_point["rotationDegrees"],
        start_heading_x = start_point["defaultHeadingXMeters"],
        start_heading_y = start_point["defaultHeadingYMeters"],
        end_pose_x = end_point["xMeters"],
        end_pose_y = end_point["yMeters"],
        end_pose_rotation = end_point["rotationDegrees"],
        end_heading_x = end_point["defaultHeadingXMeters"],
        end_heading_y = end_point["defaultHeadingYMeters"]
    )
    print(generated_path_string)
    generated_path = json.loads(generated_path_string)
    print(generated_path)
    with open('../../src/main/deploy/pathplanner/paths/Template ' + start_point["name"] + " - " + end_point["name"] + ".path", 'w') as f:
        json.dump(generated_path, f, ensure_ascii=False, indent=4)

for [a, b] in all_pairs:
    createAutoPath(a, b)
    createAutoPath(b, a)
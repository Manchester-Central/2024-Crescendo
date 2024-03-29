import json
from pathlib import Path
from itertools import combinations
from string import Template
import csv

SETTINGS_FILE = Path('../../.pathplanner/settings.json')
def getSettings():
    settings = json.loads(SETTINGS_FILE.read_text())
    print(settings)
    return settings

initialSettings = getSettings()
max_velocity = initialSettings["defaultMaxVel"]
max_acceleration = initialSettings["defaultMaxAccel"]
max_angular_velocity = initialSettings["defaultMaxAngVel"]
max_angular_acceleration = initialSettings["defaultMaxAngAccel"]

def getAllPoints():
    with open('./points.csv', 'r') as file:
        csv_reader = csv.DictReader(file)
        points = [row for row in csv_reader]
    print(points)
    return points

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
      "linkedName": "${start_pose_link}"
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
      "linkedName": "${end_pose_link}"
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
    "rotateFast": false
  },
  "reversed": false,
  "folder": "${folder}",
  "previewStartingState": {
    "rotation": ${start_pose_rotation},
    "velocity": 0
  },
  "useDefaultConstraints": true
}
""")
print(json_path)

json_auto = Template("""
{
  "version": 1.0,
  "startingPose": {
    "position": {
      "x": ${start_pose_x},
      "y": ${start_pose_y}
    },
    "rotation": ${start_pose_rotation}
  },
  "command": {
    "type": "sequential",
    "data": {
      "commands": ${commands}
    }
  },
  "folder": null,
  "choreoAuto": false
}
""")
print(json_auto)

def createAutoPath(auto_name, folder, start_point, end_point, previous_link = None):
    print("-----------")
    full_path_name = auto_name + " " + start_point["name"] + " - " + end_point["name"]
    start_link = previous_link if previous_link is not None else full_path_name + " - START"
    end_link = full_path_name + " - END"
    generated_path_string = json_path.substitute(
        max_velocity = max_velocity,
        max_acceleration = max_acceleration,
        max_angular_velocity = max_angular_velocity,
        max_angular_acceleration = max_angular_acceleration,
        start_pose_link = start_link,
        start_pose_x = start_point["xMeters"],
        start_pose_y = start_point["yMeters"],
        start_pose_rotation = start_point["rotationDegrees"],
        start_heading_x = start_point["defaultHeadingXMeters"],
        start_heading_y = start_point["defaultHeadingYMeters"],
        end_pose_link = end_link,
        end_pose_x = end_point["xMeters"],
        end_pose_y = end_point["yMeters"],
        end_pose_rotation = end_point["rotationDegrees"],
        end_heading_x = end_point["defaultHeadingXMeters"],
        end_heading_y = end_point["defaultHeadingYMeters"],
        folder = folder
    )
    print(generated_path_string)
    generated_path = json.loads(generated_path_string)
    print(generated_path)
    with open("../../src/main/deploy/pathplanner/paths/" + full_path_name + ".path", 'w') as f:
        json.dump(generated_path, f, ensure_ascii=False, indent=4)
    return {
        "pathName": full_path_name,
        "startLink": start_link,
        "endLink": end_link
    }

def createAuto(name, start_point, commands):
    generated_auto_string = json_auto.substitute(
        start_pose_x = start_point["xMeters"],
        start_pose_y = start_point["yMeters"],
        start_pose_rotation = start_point["rotationDegrees"],
        commands = json.dumps(commands)
    )
    generated_auto = json.loads(generated_auto_string)
    with open("../../src/main/deploy/pathplanner/autos/" + name + ".auto", 'w') as f:
        json.dump(generated_auto, f, ensure_ascii=False, indent=4)

def createPathFolder(folderName):
    settings = getSettings()
    pathFolders = settings["pathFolders"]
    pathFolders.append(folderName)
    with open(SETTINGS_FILE, 'w') as f:
        json.dump(settings, f, ensure_ascii=False, indent=4)

def createNamedCommand(commandName):
    return {
        "type": "named",
        "data": {
            "name": commandName
        }
    }

def createPathCommand(pathName):
    return {
        "type": "path",
        "data": {
            "pathName": pathName
        }
    }
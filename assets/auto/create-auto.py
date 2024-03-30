from itertools import combinations
from pathplannerutil import createAutoPath, getAllPoints, createPathFolder, createAuto, createNamedCommand, createPathCommand
from tkinter import ttk, messagebox
import tkinter as tk

COMMAND_PREFIX = "[Command] "

points = getAllPoints()
print(points)
point_names = list(map(lambda p: p["name"], points))
point_names.insert(0, COMMAND_PREFIX + "launch")
point_names.insert(0, "")
print(point_names)

main_window = tk.Tk()
main_window.config(width=300, height=800)
main_window.title("Auto Builder")

auto_name = tk.Text(height=1, width=20)
auto_name.place(x=50, y=5)

def addComboBox(index):
    box = ttk.Combobox(state="readonly", values=point_names)
    box.place(x=50, y=(25 * index) + 30)
    return box

numBoxes = 20
boxes = []
for i in range(numBoxes):
    boxes.append(addComboBox(i))

def save():
    name = auto_name.get(1.0, "end-1c")
    print(name)
    createPathFolder(name)
    if name.isspace() or len(name) is 0:
        messagebox.showerror('Unable to save', 'A name for the auto is required')
        return
    first_point = None
    last_point = None
    last_link = None
    commands = []
    for box in boxes:
        line = box.get()
        if not line:
            continue
        print(line)
        if line.startswith(COMMAND_PREFIX):
            commands.append(createNamedCommand(line.replace(COMMAND_PREFIX, "")))
            continue
        new_point = next((x for x in points if x["name"] == line), None)
        if not first_point:
            first_point = new_point
        if not new_point:
            raise "Not a valid point: " + new_point
        if last_point:
            created_auto = createAutoPath(name, name, last_point, new_point, last_link)
            last_link = created_auto["endLink"]
            commands.append(createPathCommand(created_auto["pathName"]))
        last_point = new_point
    createAuto(name, first_point, commands)
    main_window.destroy()

save_button = ttk.Button(text="Save", command=save)
save_button.place(x=50, y=(25 * numBoxes) + 30)

main_window.mainloop()
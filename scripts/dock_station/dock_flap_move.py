#!/usr/bin/env python3
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import subprocess
import tkinter as tk

OPEN = 1.2   # rad
DOCKS = ["dock", "dock_2", "dock_3", "dock_4"]

try:
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double

    node = Node()
    pubs = {d: (node.advertise(f"/{d}/left/cmd", Double),
                node.advertise(f"/{d}/right/cmd", Double)) for d in DOCKS}

    def send(dock, angle):
        l, r = Double(), Double()
        l.data, r.data = -angle, angle
        pubs[dock][0].publish(l)
        pubs[dock][1].publish(r)

except ImportError:
    def send(dock, angle):
        cmd = (f"gz topic -t /{dock}/left/cmd -m gz.msgs.Double -p 'data: {-angle}' & "
               f"gz topic -t /{dock}/right/cmd -m gz.msgs.Double -p 'data: {angle}' & wait")
        subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def send_all(angle):
    for d in DOCKS:
        send(d, angle)

root = tk.Tk()
root.title("Docks")

# ALL row
fa = tk.LabelFrame(root, text="ALL")
fa.grid(row=0, column=0, columnspan=len(DOCKS), padx=6, pady=6, sticky="ew")
tk.Button(fa, text="OPEN ALL", width=20, bg="#4caf50",
          command=lambda: send_all(OPEN)).pack(side="left", padx=8, pady=8, expand=True)
tk.Button(fa, text="CLOSE ALL", width=20, bg="#f44336",
          command=lambda: send_all(0)).pack(side="left", padx=8, pady=8, expand=True)

# Per-dock
for i, d in enumerate(DOCKS):
    f = tk.LabelFrame(root, text=d)
    f.grid(row=1, column=i, padx=6, pady=6)
    tk.Button(f, text="OPEN", width=12, bg="#4caf50",
              command=lambda d=d: send(d, OPEN)).pack(padx=8, pady=(8, 3))
    tk.Button(f, text="CLOSE", width=12, bg="#f44336",
              command=lambda d=d: send(d, 0)).pack(padx=8, pady=(3, 8))

root.mainloop()

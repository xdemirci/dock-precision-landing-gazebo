#!/usr/bin/env python3
"""
dock_model SDF uretici (v4 tasarimi: ayak + taban + sol/sag flap + on/arka panel).

    python3 gen_dock.py --h 3
    python3 gen_dock.py --h 5 --ped-w 3 --out ~/models/dock_model/model.sdf

--h      ayak yuksekligi (m). Taban ustu = h + 0.05
--ped-w  ayak genisligi (m), uzun kulede devrilmeyi azaltmak icin artir
--mark-h marker kaidesi yuksekligi (m), 0.8x0.8 beyaz plaka boyutunda, taban ustunde (varsayilan 0 = yok)
--out    cikti dosyasi (varsayilan: dock_model.sdf)

Dunyada dock pose z = 0 olmali. precland_center.py icin --dock-h = h + 0.05 + mark-h
"""
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("--h", type=float, default=1.0, help="ayak yuksekligi (m)")
ap.add_argument("--ped-w", type=float, default=1.8, help="ayak genisligi (m)")
ap.add_argument("--mark-h", type=float, default=0.0, help="marker kaidesi yuksekligi (m)")
ap.add_argument("--out", default="dock_model.sdf")
args = ap.parse_args()

H, W, MH = args.h, args.ped_w, args.mark_h
if H <= 0:
    raise SystemExit("--h > 0 olmali")


def g(v):
    return f"{v:.6g}"


BLOCK = ""
if MH > 0:
    BLOCK = f"""
      <collision name="markpad_c">
        <pose>0 0 {g(0.05 + MH / 2)} 0 0 0</pose>
        <geometry><box><size>0.8 0.8 {g(MH)}</size></box></geometry>
      </collision>
      <visual name="markpad_v">
        <pose>0 0 {g(0.05 + MH / 2)} 0 0 0</pose>
        <geometry><box><size>0.8 0.8 {g(MH)}</size></box></geometry>
        <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
      </visual>
"""


FZ = H + 0.05                      # flap / panel menteşe yuksekligi
M = 80.0 * H                       # ayak kutlesi (yukseklikle olcekli)
IXY = M * (W * W + H * H) / 12.0
IZZ = M * (2 * W * W) / 12.0
COM_Z = -0.45 * H


def panel(name, y_link, y_c, lower, upper, topic):
    return f"""
    <!-- {name}: tek parca panel, alt kenardan mentese (y={y_link}) -->
    <link name="{name}">
      <pose>0 {y_link} {g(FZ)} 0 0 0</pose>
      <inertial>
        <pose>0 {y_c} 0.45 0 0 0</pose>
        <mass>4</mass>
        <inertia><ixx>0.27</ixx><iyy>1.44</iyy><izz>1.18</izz></inertia>
      </inertial>
      <collision name="{name}_c">
        <pose>0 {y_c} 0.45 0 0 0</pose>
        <geometry><box><size>1.88 0.05 0.89</size></box></geometry>
      </collision>
      <visual name="{name}_v">
        <pose>0 {y_c} 0.45 0 0 0</pose>
        <geometry><box><size>1.88 0.05 0.89</size></box></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
      </visual>
    </link>

    <joint name="{name}_joint" type="revolute">
      <parent>body</parent>
      <child>{name}</child>
      <pose>0 0 0 0 0 0</pose>
      <axis>
        <xyz>1 0 0</xyz>
        <limit><lower>{lower}</lower><upper>{upper}</upper></limit>
        <dynamics><damping>5</damping></dynamics>
      </axis>
    </joint>

    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{name}_joint</joint_name>
      <topic>{topic}</topic>
      <p_gain>300</p_gain><i_gain>2</i_gain><d_gain>30</d_gain>
    </plugin>
"""


def side(name, jname, x_link, sx, color_wall, color_flap, lower, upper, topic):
    # sx: +1 sol (x=-1, duvar +x'e), -1 sag
    return f"""
    <link name="{name}">
      <pose>{x_link} 0 {g(FZ)} 0 0 0</pose>
      <inertial>
        <pose>{g(0.2 * sx)} 0 0.45 0 0 0</pose>
        <mass>5</mass>
        <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>
      </inertial>
      <collision name="{name}_wall_c">
        <pose>{g(0.025 * sx)} 0 0.45 0 0 0</pose>
        <geometry><box><size>0.05 2 0.9</size></box></geometry>
      </collision>
      <visual name="{name}_wall_v">
        <pose>{g(0.025 * sx)} 0 0.45 0 0 0</pose>
        <geometry><box><size>0.05 2 0.9</size></box></geometry>
        <material><ambient>{color_wall} 1</ambient><diffuse>{color_wall} 1</diffuse></material>
      </visual>
      <collision name="{name}_flap_c">
        <pose>{g(0.5 * sx)} 0 0.925 0 0 0</pose>
        <geometry><box><size>1 2 0.05</size></box></geometry>
      </collision>
      <visual name="{name}_flap_v">
        <pose>{g(0.5 * sx)} 0 0.925 0 0 0</pose>
        <geometry><box><size>1 2 0.05</size></box></geometry>
        <material><ambient>{color_flap} 1</ambient><diffuse>{color_flap} 1</diffuse></material>
      </visual>
    </link>

    <joint name="{jname}" type="revolute">
      <parent>body</parent>
      <child>{name}</child>
      <pose>0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>{lower}</lower><upper>{upper}</upper></limit>
        <dynamics><damping>5</damping></dynamics>
      </axis>
    </joint>

    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{jname}</joint_name>
      <topic>{topic}</topic>
      <p_gain>300</p_gain><i_gain>2</i_gain><d_gain>30</d_gain>
    </plugin>
"""


sdf = f"""<?xml version="1.0"?>
<!-- uretildi: gen_dock.py h={g(H)} ped_w={g(W)} mark_h={g(MH)} -->
<sdf version="1.9">
  <model name="dock_model">

    <plugin filename="gz-sim-velocity-control-system"
            name="gz::sim::systems::VelocityControl"/>

    <!-- GOVDE: ayak (link z -{g(H)}..0) + taban 2x2 + ArUco. Link orijini taban alt yuzu (dunya z={g(H)}) -->
    <link name="body">
      <pose>0 0 {g(H)} 0 0 0</pose>
      <inertial>
        <pose>0 0 {g(COM_Z)} 0 0 0</pose>
        <mass>{g(M)}</mass>
        <inertia><ixx>{g(IXY)}</ixx><iyy>{g(IXY)}</iyy><izz>{g(IZZ)}</izz></inertia>
      </inertial>

      <collision name="pedestal_c">
        <pose>0 0 {g(-H / 2)} 0 0 0</pose>
        <geometry><box><size>{g(W)} {g(W)} {g(H)}</size></box></geometry>
      </collision>
      <visual name="pedestal_v">
        <pose>0 0 {g(-H / 2)} 0 0 0</pose>
        <geometry><box><size>{g(W)} {g(W)} {g(H)}</size></box></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>

      <collision name="bottom_c">
        <pose>0 0 0.025 0 0 0</pose>
        <geometry><box><size>2 2 0.05</size></box></geometry>
      </collision>
      <visual name="bottom_v">
        <pose>0 0 0.025 0 0 0</pose>
        <geometry><box><size>2 2 0.05</size></box></geometry>
        <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>

      {BLOCK}
      <visual name="aruco_plate_v">
        <pose>0 0 {g(0.0505 + MH)} 0 0 0</pose>
        <geometry><box><size>0.8 0.8 0.001</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>

      <visual name="aruco_v">
        <pose>0 0 {g(0.052 + MH)} 0 0 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>0.5 0.5</size></plane>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://arucotag1/arucotag1.png</albedo_map>
              <roughness>1.0</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
"""

sdf += side("left_side", "left_joint", -1, +1, "0.7 0.7 0.7", "0.92 0.92 0.92", -1.6, 0, "/dock/left/cmd")
sdf += side("right_side", "right_joint", 1, -1, "0.7 0.7 0.7", "0.92 0.92 0.92", 0, 1.6, "/dock/right/cmd")
# on (y=+1): disari = +y -> negatif aci -> sol komutu (-a); arka (y=-1): pozitif aci -> sag komutu (+a)
sdf += panel("front_panel", 1.0, -0.025, -1.6, 0, "/dock/left/cmd")
sdf += panel("back_panel", -1.0, 0.025, 0, 1.6, "/dock/right/cmd")
sdf += """
  </model>
</sdf>
"""

with open(args.out, "w") as f:
    f.write(sdf)
print(f"yazildi: {args.out}  (h={g(H)}, ayak {g(W)}x{g(W)}, taban ustu z={g(FZ)}, "
      f"marker kaidesi {g(MH)} m, kutle={g(M)} kg)  precland --dock-h {g(FZ + MH)}")

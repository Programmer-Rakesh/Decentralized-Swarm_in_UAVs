# Swarm GCS — compact layout (4 code files)

    python3 gcs5.py                                                # the ground station (run this)
    python3 swarm_agent.py --master tcp:127.0.0.1:5762 --sysid 1   # one agent (the GCS starts these in AUTO MODE)

Install: `pip install -r requirements.txt`

## Tree
```
swarm_gcs/
├── gcs5.py          WHOLE GCS: MAVLink link, connection bar, layout, map, panels,
│                    telemetry loop, swarm control (bus, mission, RTL, navigation panel), DashboardApp, main()
├── gcs_ui.py        ALL looks: colours, sizes, map-style list, timings, glass panel, icons,
│                    every widget (buttons, gauges, RSSI, battery, check box), button/section helpers
├── gcs_3d.py        3D Map button (starts drone_3d.py -- put drone_3d.py in this same folder)
├── swarm_agent.py   ALL algorithms: formation geometry, formation/transit control, RTL + landing,
│                    UDP bus, MAVLink vehicle, message handlers, Agent, main()   (one copy runs per drone)
├── requirements.txt
└── README.md
```
Every file is divided by banners — search for `# SECTION:` to jump (each banner says which old file it came from).
The GCS imports `formation_slots` from `swarm_agent.py`, so formation geometry now exists in ONE place only.
Keep `swarm_agent.py` next to `gcs5.py` (the GCS spawns it from there).

## "I want to change ..." -> where
| Change | File / section |
|---|---|
| Colours, ports, timings, default paths, map styles | `gcs_ui.py` -> SECTION config.py (top) |
| Button / gauge / panel look | `gcs_ui.py` -> SECTION edge_button, glass_panel, ... , ui_helpers |
| Window layout, side tabs | `gcs5.py` -> SECTION layout.py, side_panels.py |
| Connect button, port list, SIM/TELE | `gcs5.py` -> SECTION connection.py, status_bar.py |
| Map, waypoints, mission path | `gcs5.py` -> SECTION map_panel, map_style, map_overlays, waypoints |
| Takeoff/AUTO, gauges, failsafe, params, log | `gcs5.py` -> SECTION control_panel ... log_panel |
| Telemetry -> UI | `gcs5.py` -> SECTION poll_loop.py |
| Swarm mission / RTL / formation bar / nav panel | `gcs5.py` -> SECTION mission, rtl, formation_bar, nav_* |
| MAVLink send/receive | `gcs5.py` -> SECTION mavlink, mav_commands, mav_mission |
| 3D button | `gcs_3d.py` |
| Formation shapes, control gains, RTL/landing, agent <-> autopilot | `swarm_agent.py` (config at top, then geometry, control, rtl, vehicle) |

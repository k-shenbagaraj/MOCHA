☕ MOCHA: Multi-robot Opportunistic Communication for Heterogeneous Collaboration
---------------------------------------------------------------------------------

[![build](https://github.com/KumarRobotics/MOCHA/actions/workflows/build.yaml/badge.svg?branch=ros2)](https://github.com/KumarRobotics/MOCHA/actions/workflows/build.yaml)

![MOCHA gif](mocha.gif)

This repository contains the distributed and opportunistic communication stack used for multi-robot experiments at KumarRobotics.

## Directories

 - `mocha_launch/`: launch files the different robots in MOCHA. The launch file
   sets up the `robot_name` argument,
 - `mocha_core/`: MOCHA's main components (source code, config files).
 - `interface_rajant/`: interface for Rajant breadrumb radios

## Dependencies:

MOCHA requires `rospkg`, `defusedxml`, and `python3-zmq`. You may install these
packages with:

```
sudo apt update
pip3 install rospkg
pip3 install defusedxml
sudo apt install python3-zmq
```

## Rajant to WiFi fallback

MOCHA can use WiFi as a backup transport for the same ZMQ sync channel used over
the Rajant mesh. Add each robot's WiFi address to
`mocha_core/config/robot_configs.yaml`:

By default, MOCHA falls back to WiFi when Rajant RSSI is below the threshold or
when RSSI messages stop for `rajant_signal_timeout` seconds. To also keep a
periodic WiFi backup sync running while Rajant is healthy:

```bash
ros2 launch mocha_launch titan.launch.py wifi_backup_always_on:=true
```

Useful knobs are `wifi_fallback_enabled`, `wifi_backup_always_on`,
`wifi_backup_period`, and `rajant_signal_timeout`.

## Contribution - Questions

Please [fill-out an issue](https://github.com/KumarRobotics/MOCHA/issues) if you have any questions.
Do not hesitate to [send your pull request](https://github.com/KumarRobotics/MOCHA/pulls).

## Citation

If you find MOCHA useful, please cite:

```
@INPROCEEDINGS{cladera2024enabling,
  author={Cladera, Fernando and Ravichandran, Zachary and Miller, Ian D. and Ani Hsieh, M. and Taylor, C. J. and Kumar, Vijay},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  title={{Enabling Large-scale Heterogeneous Collaboration with Opportunistic Communications}},
  year={2024},
  pages={2610-2616},
  doi={10.1109/ICRA57147.2024.10611469}
}
```

# constraint_to_gazebo

## Overview

`constraint_to_gazebo` is a small utility package that uses the `<constraint>` elements defined in a URDF file into **Gazebo detachable joint plugins** and create a valid kinematic chain. This is only used to start a Gezbeo simulation with an ARDF in a launch file to proove, that it can be used. An import function in Gazebo might be able to use this process at startup, if the ARDF extension is accepted as pull request. 
It parses the URDF, generates the appropriate `<plugin>` XML elements and inserts them back into the robot description, enabling the reassembling of the constraints in Gazebo simulations. Additionally it uses a numerical solver to calculate the correct joint offsets to ensure a valid closed loop. This is necessary because the offsets are not defined in the URDF file and would cause the robot to not be able to close the loop.
The defined structure must be able to get a valid configuration in the zero position of the actuated joints marked with a `<transmission>`. The offset values are used to correct the detachable joint frames to ensure that the kinematic chain closes correctly. To ensure a good simulation, all virtual joints without an inertial element can be generated with a new inertial element to satisfy the simulation requirements.

## Features

- Supports the most common constraint types (`fixed`, `revolute`, `prismatic`, `spherical`, `universal`).
- Generates a `gz::sim::systems::DetachableJoint` plugin for each constraint.
- Simple Python API that adds the new Elements to a etree description and returns the adapted etree.
- Use a numerical solver to create a valid configuration of the closed loop in the zero state of the actuated joints.
- add a world link to the URDF
- add inertia values to links without any inertial values

## Installation

The package depends on:

- `ament_cmake`
- `ament_cmake_python`
- `python3-lxml`
- `pinocchio` - self compiled with our urdfdom fork (for urdf parsing)
- `robot_model` - our solver for the constraint elements

## Usage (Python API)

```python
from constraint_to_gazebo.constraint_to_detachable_joint import addDetachableJoints
from constraint_to_gazebo.add_masses import addMassesToDummyLinks
from constraint_to_gazebo.add_world_link import addWorldLink
from constraint_to_gazebo.close_loop import close_loop
from robot_model import RobotModel

# Provide the absolute path to your URDF file
urdf_path = "/path/to/robot.urdf"

# process the URDF by reading it and apply the changes to the tree    
robot_model = RobotModel(path)
urdf_tree = etree.parse(path)
# close th loops to a valid configuration and save the remaining transfomrations beween the constraint frames to add the cut links correctly
urdf_tree, violations = close_loop(urdf_tree, robot_model)
# create the detachable joinst and use the violations to add the cut links correctly
urdf_tree = addDetachableJoints(urdf_tree, violations)
# add inertia values to the virtual joints
urdf_tree = addMassesToDummyLinks(urdf_tree)
# create a world link and attach it to the robot
urdf_tree = addWorldLink(urdf_tree, offset=[0, 0, 1.5])
# create string representation of the urdf
processed_urdf_xml = etree.tostring(urdf_tree, encoding='unicode')
# You can write it back to a file if you wish
with open('processed_robot.urdf', 'w') as f:
    f.write(processed_urdf_xml)
```

The function will raise `FileNotFoundError` if the supplied path does not exist.

## Integration with ROS 2 launch

The `bringup` package provides a launch file (`sim.launch.py`) that demonstrates how to:

1. Pass the URDF path as a launch argument.
2. Prepare the urdef for gazebo simulation
3. Spawn the robot in Gazebo.

```bash
ros2 launch bringup sim.launch.py urdf_file:=/absolute/path/to/robot.urdf
```

## roadmap

- test with more robots
- improve documentation

## Contributing

Feel free to open issues or pull requests. When adding support for new constraint types, extend the `createDetachableJointElements` dispatch in `constraint_to_detachable_joint.py`.

## License

This package is released under the BSD license (see the `LICENSE` file in the repository).

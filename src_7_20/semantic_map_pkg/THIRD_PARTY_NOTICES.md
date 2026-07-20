# Third-Party Notices

## Cartographer configuration files

The files under `config/cartographer/` are copied or adapted from the local
Cartographer configuration used by this project. Upstream Cartographer is
licensed under the Apache License 2.0.

Upstream project: <https://github.com/cartographer-project/cartographer>

## Robot calibration URDF

`urdf/rosbag_robot.urdf` is copied from the project's `robot_calibration`
package so that the recorded `base_link -> laser -> camera_link` calibration
can be replayed without machine-specific paths. The source package declares
the MIT license.

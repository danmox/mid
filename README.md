# mid

A collection of ROS packages for mobile (wireless) infrastructure on demand.

## Installation

Clone this repo in the `src` folder of a catkin workspace. ROS dependencies can be installed using rosdep in the usual way:
```bash
rosdep install --from-paths . --ignore-src -y
```
`robust_routing` and `connectivity_planner` depend on [cvxpy](https://www.cvxpy.org/) which is not distributed through apt and must be installed using pip (or pip3 depending on your system):
```bash
pip install --user cvxpy
```

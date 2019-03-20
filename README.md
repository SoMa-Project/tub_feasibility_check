# tub_feasibility_check

## Changes in this branch
### Surface grasp call and wall grasp call
This branch introduces a combined surface grasp call and a combined wall grasp call and pregrasp manifolds for those grasps.
They are documented in the ROS .srv and .msg definitions:

- [combined surface grasp call](srv/CheckSurfaceGrasp.srv)
- [pregrasp manifolds for surface grasp](msg/SurfaceGraspPregraspManifold.msg)
- [combined wall grasp call](srv/CheckWallGrasp.srv)
- [pregrasp manifold for wall grasp](msg/WallGraspPregraspManifold.msg)

### Currently only for the IFCO scene
The combined calls currently only work for the IFCO scene. Extending them to work for the tabletop scene would be straightforward.

### Notebooks for prototyping the manifolds
There is a notebook for every pregrasp manifold type: [circular manifold for surface grasp](notebooks/surface-grasp-circular.ipynb),
[elongated manifold for surface grasp](notebooks/surface-grasp-elongated.ipynb) and [wall grasp manifold](notebooks/wall-grasp-manifold.ipynb). However, they have not been updated, and there are some differences in the text description to what is currently implemented. The up-to-date description resides in the .srv and .msg files. Still, they can be used to visualize what is going on or test ideas.

### Examples
There is a number of examples demonstrating how the combined calls work. They can be found in [examples/combined_surface_grasp](examples/combined_surface_grasp) and [examples/combined_wall_grasp](examples/combined_wall_grasp). To run an example, you need to start the feasibility checker for the IFCO scene.
```bash
roslaunch tub_feasibility_check tub_feasibility_check.launch
```

And then pipe the provided YAML file to the `rosservice` call:
```bash
rosservice call /check_surface_grasp "`cat examples/combined_surface_grasp/success_without_sampling.yaml`"
```

### Visualization
The pregrasp manifolds are visualized with grey boxes. 

**Note that** for the circular manifold, the visualization always shows a circle, but the positions are actually sampled on a ring defined by `min_radius` and `max_radius`.

### How do I modify existing manifolds?
If you only wish to change how a specific manifold works, you only need to edit the `generate` method of the respective manifold. For example, if you wish to tweak the logic for the circular manifold, you would edit `CircularManifold::generate` in [src/manifolds/circular_manifold.cpp](src/manifolds/circular_manifold.cpp).

If your modification does not require passing new parameters, that is it. If it does, then you need to:

- add the parameters to the .msg file that defines the manifold
- add a field to the corresponding `Description` struct of the manifold, for example `CircularManifold::Description` in [src/manifolds/circular_manifold.h](src/manifolds/circular_manifold.h)
- parse the ROS message field and write it into the `Description` struct in `processCheckSurfaceGraspParameters` or `processCheckWallGraspParameters` in [src/check_kinematics_parameters.cpp](src/check_kinematics_parameters.cpp)

## Installation:
### Requirements
In order to use the feasibility checker you have to fulfill the requirements of the contact-motion-planning submodule (https://gitlab.tubit.tu-berlin.de/rbo-lab/contact-motion-planning).

On ubuntu those dependencies can be resolved by installing:
```
sudo apt-get install libboost-all-dev libbullet-dev libcoin80-dev libdc1394-22-dev libeigen3-dev libnlopt-dev libode-dev libqt4-dev libqt4-opengl-dev libsimage-dev libsolid3d-dev libsoqt4-dev libxml2-dev libxslt1-dev
```
The feasibility checker does not introduce additional dependencies itself.

### Setup
To install the feasibility checker you can either use the convenenience script git_clone.sh from soma_utils (https://github.com/SoMa-Project/soma_utils/blob/master/scripts/soma_git_clone.sh) or build it manually using catkin:

```
cd <your catkin workspace>
git clone https://github.com/SoMa-Project/tub_feasibility_check.git
cd tub_feasibility_check
git submodule init
git submodule update
catkin build tub_feasibility_check
```


## Usage:
To start the ros service you can call ``` roslaunch tub_feasibility_check tub_feasibility_check.launch ``` or use the convenenience script clean_start.sh from soma_utils (https://github.com/SoMa-Project/soma_utils/blob/master/scripts/clean_start.sh)

The service request parameters and response are documented within the srv file https://github.com/SoMa-Project/tub_feasibility_check/blob/master/srv/CheckKinematics.srv

For example calls to the service (using the comand line) check out the scripts in the `example` folder.
The simplest example is `surface_grasp_sampling_simple.sh`, which encodes a straight go down movement without any unallowed collision. (**TODO** the other examples still use the wrong ifco frames (see https://github.com/SoMa-Project/tub_feasibility_check/issues/4). fix that

### Service call parameters
#### AllowedCollisions
##### Objects
You can allow collisions with arbitrary objects in the ifco. **TODO**: Describe attributes etc.
##### Environmental Constraints
You can allow collisions with parts of the ifco. Therefore you have to set the attribute `constraint_name`. Possible values are: `bottom, north, east, south, west`. 
Since the ifco_pose can be set arbitrarily it makes no sense to refer to them after applying the current ifco transform. Instead the initial placement of the ifco and the robot is used to determine the naming.
You can see the naming in this image:

![ifco wall naming](https://github.com/SoMa-Project/tub_feasibility_check/blob/master/examples/ifco_tub_feasibility_naming.png)

# tub_feasibility_check

## Changes in this branch
### Combined surface grasp call
This branch introduces a combined surface grasp call. Inputs to that call are pregrasp goal pose and a go down goal pose. The combined service call will try to execute initial grasp defined by those poses. 

If that fails, pregrasp poses will be sampled from a manifold. Then a trajectory will be tried that first goes to sampled pose, and from there downward to a modified go down pose preserving the orientation of the sampled pregrasp pose.

### A pregrasp manifold for surface grasping circular objects
A new type of manifold is also introduced. It is designed for circular objects.
In the following, initial pregrasp goal pose is referred to as initial frame. 
The sampled frame origin is sampled in the plane defined by the X and Y axes of the initial frame. It is sampled uniformly from a circle lying in that plane around the goal frame position with a certain radius. The sampled frame orientation is rotated such that the X axis of the frame points towards the origin of the initial frame.

Check the [jupyter notebook](notebooks/surface-grasp-circular.ipynb) for visualizations and example code. 

### A pregrasp manifold for surface grasping elongated objects
The sampled frame origin is sampled in the plane defined by the X and Y axes of the initial frame. It is sampled uniformly from two stripes aligned with the X axis of the initial frame. The placement and sizes of stripes can be controlled via their width, height and offset. The sampled frame orientation is rotated such that the X axis of the frame is perpendicular to the X and Z axes of the initial frame and goes in the direction of the initial frame.

Check the [jupyter notebook](notebooks/surface-grasp-elongated.ipynb) for visualizations and example code. 

### Wall grasp pregrasp manifold
[Jupyter notebook](notebooks/wall-grasp-manifold.ipynb).

### Examples
The following [video](https://drive.google.com/open?id=1rifkSblYNoxWirS3yiKDp53Ti947qAfm) demonstrates how the new combined call works. To try examples used to produced this video yourself, first start the feasibility checker and then call the `/check_surface_grasp` service using the provided example yaml files:

```bash
roslaunch tub_feasibility_check tub_feasibility_check.launch
rosservice call /check_surface_grasp "`cat examples/combined_surface_grasp/success_without_sampling.yaml`"
rosservice call /check_surface_grasp "`cat examples/combined_surface_grasp/sampling_near_wall.yaml`"
rosservice call /check_surface_grasp "`cat examples/combined_surface_grasp/sampling_in_center.yaml`"
```

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

# tub_feasibility_check

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

### Troubleshooting
If there are issues building tub_feasibility_check, these steps might help:
* grep -rl 'dSINGLE' ./ | xargs sed -i 's/dSINGLE/dDOUBLE/g'
* Qt4 + CGAL - Parse error at “BOOST_JOIN”: edit the files usr/include/boost/type_traits/detail/has_binary_operator.hp and has_prefix_operator.hp: add `#ifndef Q_MOC_RUN` and `#endif` around the BOOST_JOIN namespace implementation

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

**TODO**: more paramters, more details in general?

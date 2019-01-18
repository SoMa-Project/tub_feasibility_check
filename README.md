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


## Usage:
To start the ros service you can call ``` roslaunch tub_feasibility_check tub_feasibility_check.launch ``` or use the convenenience script clean_start.sh from soma_utils (https://github.com/SoMa-Project/soma_utils/blob/master/scripts/clean_start.sh)

The service request parameters and response are documented within the srv file https://github.com/SoMa-Project/tub_feasibility_check/blob/master/srv/CheckKinematics.srv

**TODO**: more details??

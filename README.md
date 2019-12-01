# SwarmTouch

An interaction strategy for a human-swarm communication, which combines impedance control and vibrotactile feedback. The  approach takes into account the human hand velocity and changes drones formation shape and dynamics accordingly using impedance interlinks simulated between quadrotors. Several tactile patterns representing static and dynamic parameters of the swarm are proposed. The user feels the state of the swarm at the fingertips and receives valuable information to improve the controllability of the complex formation.

Take a look at the videos to get a better sense about the project:
* [Video](https://www.youtube.com/watch?v=CX2mOAT3anM&feature=youtu.be) from IEEE Spectrum featured video section
* [Full Video](https://www.youtube.com/watch?v=GWKGQKcj-XA&feature=youtu.be)

## Data
Flight experiments and tactile patterns recognition data is available on [Zenodo](https://doi.org/10.5281/zenodo.3256614).

## Tutorial
This code section is aimed to explain how to create impedance interlinks
between human operator and drones in the swarm with the help of [swarmlib](https://github.com/negvet/swarmtouch/blob/master/flight_code/swarmlib.py).

Import neccessary packages.
```python
import rospy
import crazyflie
import swarmlib
import time
import numpy as np
```

Introduce flight parameters and initialize impedance correction terms.
```python
TAKEOFFHEIGHT    = 1.45 # meters
TakeoffTime      = 4    # seconds
l                = 0.4  # distance between drones, meters

initialized = False
imp_pose_prev = np.array( [0,0,0] )
imp_vel_prev = np.array( [0,0,0] )
imp_time_prev = time.time()

pos_koef          = 2.0 # this parameter defines the scale on how human movements are maped to the swarm displacements 
```
Define motion capture objects that you use in [Vicon Tracker](https://www.vicon.com/software/tracker/). In our case these are 3 drones and a human operator.
```python
cf1_name         = 'cf1'
cf2_name         = 'cf2'
cf3_name         = 'cf3'
human_name       = 'human'

rospy.init_node('follow_multiple', anonymous=True)
# Objects init
human = swarmlib.mocap_object(human_name)
drone1 = swarmlib.drone(cf1_name)
drone2 = swarmlib.drone(cf2_name)
drone3 = swarmlib.drone(cf3_name)
```

Takeoff!
```python
cf1 = crazyflie.Crazyflie(cf1_name, '/vicon/'+cf1_name+'/'+cf1_name)
cf1.setParam("commander/enHighLevel", 1)
cf1.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 3.0)
cf2 = crazyflie.Crazyflie(cf2_name, '/vicon/'+cf2_name+'/'+cf2_name)
cf2.setParam("commander/enHighLevel", 1)
cf2.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 3.0)
cf3 = crazyflie.Crazyflie(cf3_name, '/vicon/'+cf3_name+'/'+cf3_name)
cf3.setParam("commander/enHighLevel", 1)
cf3.takeoff(targetHeight = TAKEOFFHEIGHT, duration = 3.0)
# time to takeoff and select position for human
time.sleep(TakeoffTime)
```

The next thing to do is to specify the commands to the swarm of drones from a human operator.
This is done interactively and in real-time.
```python
rate = rospy.Rate(60) # specify here the objects poses update rate from Vicon Tracker
while not rospy.is_shutdown():
  # update current position of all objects every loop once in the beginning
  drone1.position()
  drone2.position()
  drone3.position()
  human.position()
  
  # compute impedance update term from a human operator position
  hum_vel = swarmlib.hum_vel(human.pose)
  imp_pose, imp_vel, imp_time_prev = swarmlib.impedance_human(hum_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
  imp_pose_prev = imp_pose
  imp_vel_prev = imp_vel
  
  # find initial human and leader drone positions
  if not initialized:
    human_pose_init = human.pose
    drone1_pose_init = drone1.pose
    initialized = True
  
  # compute position update terms relative to human initial position at each iteration
  dx, dy = (human.position() - human_pose_init)[:2]
  drone1.sp = np.array([  drone1_pose_init[0] + pos_koef*dx,
                          drone1_pose_init[1] + pos_koef*dy,
                          TAKEOFFHEIGHT        ])
                          
  # define swarm default geometry configuration
  # triangel with equal sides
  drone2.sp = drone1.sp + np.array([-l*0.86 , l/2, 0])
  drone3.sp = drone1.sp + np.array([-l*0.86 ,-l/2, 0])
  
  # and finaly update drones positions with impedance terms
  drone1.sp = drone1.sp + imp_pose
  drone2.sp = drone2.sp + imp_pose
  drone3.sp = drone3.sp + imp_pose
  drone2.sp[1] = drone2.sp[1] - imp_pose[0]*0.15
  drone3.sp[1] = drone3.sp[1] + imp_pose[0]*0.15
```

Note, that you also need to perform a save landing, ones the flight experiment is done.
For more inforamtion, please, take a look at the [source code](https://github.com/negvet/swarmtouch/tree/master/flight_code).

## References
* [SwarmTouch: Guiding a Swarm of Micro-Quadrotors With Impedance Control Using a Wearable Tactile Interface](https://ieeexplore.ieee.org/document/8758191), ([arxiv](https://arxiv.org/abs/1909.02298))
* [SwarmTouch: Tactile Interaction of Human with Impedance Controlled Swarm of Nano-Quadrotors](https://ieeexplore.ieee.org/document/8594424/), ([arxiv](https://arxiv.org/abs/1909.03491))
* [Tactile Interaction of Human with Swarm of Nano-Quadrotors augmented with Adaptive Obstacle Avoidance](https://hal.archives-ouvertes.fr/hal-02128383/)

## Citation
Feel free to cite the articles, if you use the repository.
```
@article{tsykunov2019swarmtouch,
  title={SwarmTouch: Guiding a Swarm of Micro-Quadrotors With Impedance Control Using a Wearable Tactile Interface},
  author={Tsykunov, Evgeny and Agishev, Ruslan and Ibrahimov, Roman and Labazanova, Luiza and Tleugazy, Akerke and Tsetserukou, Dzmitry},
  journal={IEEE transactions on haptics},
  volume={12},
  number={3},
  pages={363--374},
  year={2019},
  publisher={IEEE}
}
```

```
@inproceedings{tsykunov2018swarmtouch,
  title={SwarmTouch: tactile interaction of human with impedance controlled swarm of nano-quadrotors},
  author={Tsykunov, Evgeny and Labazanova, Luiza and Tleugazy, Akerke and Tsetserukou, Dzmitry},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4204--4209},
  year={2018},
  organization={IEEE}
}
```

```
@inproceedings{agishev2019tactile,
  title={Tactile Interaction of Human with Swarm of Nano-Quadrotors augmented with Adaptive Obstacle Avoidance},
  author={Agishev, Ruslan and Tsykunov, Evgeny and Labazanova, Luiza and Tleugazy, Akerke and Tsetserukou, Dzmitry},
  year={2019}
}
```

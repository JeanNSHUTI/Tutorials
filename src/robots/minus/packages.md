# Packages

## Differential-drive

N'ayant pas le temps d'implémenter nous-même, une librairie pour gérer la navigation différentielle, nous avons chercher une librairie existante et compatible avec ROS.

This package provides some basic tools for interfacing a differential-drive robot with the ROS navigation stack.

![Schema](http://wiki.ros.org/differential_drive?action=AttachFile&do=get&target=differential_drive_overview.png)

La librairie avait donc besoin de seulement 2 informations d'entrée, le retour des encodeurs de la roue gauche (**lwheel**) et droite (**rwheel**).
En sortie, nous recevions les vitesses des 2 roues, **lmotor** et **rmotor**.

#### Setting up the differential-drive package PID controller

La configuration du package se fait via un launchfile contenant tous les paramètres nécessaires tel que *Kp*, *Ki*, *Kd*, etc.

Cette configuration se trouve dans le fichier `ros_packages/differential_driver/launch/minus.launch`

```xml
<launch>
  <rosparam param="ticks_meter">16400</rosparam>	
  <node pkg="differential_drive" type="diff_tf.py" name="diff_tf">
      <rosparam param="base_width">0.353</rosparam>
      <rosparam param="encoder_min">-1000000</rosparam>
      <rosparam param="encoder_max">1000000</rosparam>
  </node>
  <node pkg="differential_drive" type="pid_velocity" name="lpid_velocity">
      <remap from="wheel" to="lwheel"/>
      <remap from="motor_cmd" to="lmotor_cmd"/>
      <remap from="wheel_vtarget" to="lwheel_vtarget"/>
      <remap from="wheel_vel" to="lwheel_vel"/>
      <rosparam param="Kp">25</rosparam>
      <rosparam param="Ki">10</rosparam>
      <rosparam param="Kd">0</rosparam>
      <rosparam param="out_min">-254</rosparam>
      <rosparam param="out_max">254</rosparam>
      <rosparam param="rate">20</rosparam>
      <rosparam param="timeout_ticks">10</rosparam>
      <rosparam param="rolling_pts">5</rosparam>
      <rosparam param="encoder_min">-1000000</rosparam>
      <rosparam param="encoder_max">1000000</rosparam>
  </node>
  <node pkg="differential_drive" type="pid_velocity" name="rpid_velocity">
      <remap from="wheel" to="rwheel"/>
      <remap from="motor_cmd" to="rmotor_cmd"/>
      <remap from="wheel_vtarget" to="rwheel_vtarget"/>
      <remap from="wheel_vel" to="rwheel_vel"/>
      <rosparam param="Kp">25</rosparam>
      <rosparam param="Ki">10</rosparam>
      <rosparam param="Kd">0</rosparam>
      <rosparam param="out_min">-254</rosparam>
      <rosparam param="out_max">254</rosparam>
      <rosparam param="rate">20</rosparam>
      <rosparam param="timeout_ticks">10</rosparam>
      <rosparam param="rolling_pts">5</rosparam>
      <rosparam param="encoder_min">-1000000</rosparam>
      <rosparam param="encoder_max">1000000</rosparam>
  </node>

  <node pkg="differential_drive" type="twist_to_motors.py" name="twist_to_motors" output="screen">
    <remap from="twist" to="cmd_vel"/>
    <rosparam param="base_width">0.353</rosparam>
    <rosparam param="rate">20</rosparam>
  </node>
</launch>
```

This launch file assumes your robot:

* Takes commands for the wheel power on `/lmotor_cmd` and `/rmotor_cmd`

* Publishes the wheel encoder on `/lwheel` and `/rwheel`

* The base_width parameter should be set to the wheel spacing of your robot.

If this is not the case, you can change the mapping in the launch file.

As long as you meet these conditions, and the PID values are sane for your robot, you should be able to drive the robot.

> For more information about what is a [launchfile](software/ros/basics/launch.html).
A method for optimizing the PID parameters and calibrating the tick per meter is avalaible [there](http://wiki.ros.org/differential_drive/tutorials/setup).

## Sensors

Ce package nous permet de gérer la détection des obstacles grâce aux données envoyés par les capteurs ultrasons branchés à l'arduino et l'action à effectuer.

L'arduino publie sur différents topics (`ultrasound_x`), les données qu'elle reçoit des capteurs.

Le package lit les valeurs et en fonction de la position du robot (`move_base/feedback`) et des valeurs envoie un message d'arrêt sur un topic (`obstacle/stop`).

 > For more information about what is a [publisher](software/ros/basics/pub.html), a [subscriber](software/ros/basics/sub.html) and [arduino with ros](software/ros/arduino/publisher.html).

 ## Strategy


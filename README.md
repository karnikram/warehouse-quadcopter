<h1 align="center">Automated Stock Counting Using a Quadcopter</h1>
This repo contains an implementation of an optic flow based odometry module for the localization of a multirotor, and a marker based package tracking module, for stock counting within warehouse environments. The system has been implemented using the PX4 framework and ROS.

[Project report](http://karnikram.info/papers/syp-report.pdf)


<table border="0" cellspacing="10">
  <tr>
    <td width="33%"><img src="https://github.com/karnikram/warehouse-quadcopter/blob/master/images/FrontView.jpg"/></td>
    <td width="33%"><img src="https://github.com/karnikram/warehouse-quadcopter/blob/master/images/MarkerDetSetup.jpg"</td>
    <td width="33%" align="center"><img src="https://github.com/karnikram/warehouse-quadcopter/blob/master/images/Traj.jpg" width="90%"/></td>
  </tr>
</table>

### Libraries Used
* [OpenCV](https://github.com/opencv/opencv)
* [Eigen](http://eigen.tuxfamily.org/)
* [ArUco](https://www.uco.es/investiga/grupos/ava/node/26)
* [NewPing](https://github.com/PaulStoffregen/NewPing)
* [Flyt](http://docs.flytbase.com/)


ROS Packages : 
* [rosserial](http://wiki.ros.org/rosserial)
* [mavros](http://wiki.ros.org/mavros)
* [cv_bridge](http://wiki.ros.org/cv_bridge)

### Collaborators:
* [Harish S](https://github.com/harish1696)

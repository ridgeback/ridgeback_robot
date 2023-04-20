^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ridgeback_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.3 (2023-04-20)
------------------
* Add microstrain_inertial_driver to support current Microstrain IMU models (`#37 <https://github.com/ridgeback/ridgeback_robot/issues/37>`_)
  * Add microstrain_inertial_driver for supporting current Microstrain IMU models.
  * Remove accidental copy
  * Clearer comments for RIDGEBACK_IMU_MICROSTRAIN.
* Contributors: Joey Yang

0.4.2 (2022-05-17)
------------------

0.4.1 (2022-03-21)
------------------
* [ridgeback_bringup] Updated install script to explicitly use Python3.
* Change the can-udp-bringup to use ip instead of ifconfig (`#35 <https://github.com/ridgeback/ridgeback_robot/issues/35>`_)
  * Change the can-udp-bringup to use ip instead of ifconfig
  * Fix the txqueue length
  * For consistency stick with "link set can0" and not "link set dev can0"
* Contributors: Chris I-B, Tony Baltovski

0.4.0 (2022-01-15)
------------------

0.3.1 (2021-06-15)
------------------
* [ridgeback_bringup] Made ros service start after can-udp-bridge service.
* [ridgeback_bringup] Removed connman config.
* Contributors: Tony Baltovski

0.3.0 (2020-11-12)
------------------
* [ridgeback_bringup] Increased CAN TX queue size.
* Contributors: Tony Baltovski

0.2.6 (2020-11-12)
------------------
* Bump CMake version to avoid CMP0048 warning.
* Contributors: Tony Baltovski

0.2.5 (2020-10-19)
------------------

0.2.4 (2019-11-22)
------------------

0.2.3 (2019-03-23)
------------------

0.2.1 (2018-08-02)
------------------
* Updated default IPs for Kinetic
* [ridgeback_bringup] Fixed test dep.
* Contributors: Dave Niewinski, Tony Baltovski

0.2.0 (2018-05-23)
------------------
* Updated to package format 2.
* [ridgeback_base] Switched to rosserial_server_udp.
* Updated bringup for kinetic
* Added Sick S300 laser and Microstrain IMU upgrade accessories.
* Updated maintainer.
* Contributors: Dave Niewinski, Tony Baltovski

0.1.7 (2016-10-03)
------------------

0.1.6 (2016-04-22)
------------------
* Specified max/min angles for Hokuyo UST-10LX.
* Added support for Hokuyo URG-10LX.
* Contributors: Tony Baltovski

0.1.5 (2016-03-02)
------------------
* Removed br0 interface from upstart job to default to hostname.
* Contributors: Tony Baltovski

0.1.4 (2015-12-01)
------------------
* Added manufacturer to laser environment variables.
* Installed robot_upstart install script with connman and can-udp-bridge.conf.
* Contributors: Mike Purvis, Tony Baltovski

0.1.3 (2015-11-20)
------------------

0.1.2 (2015-11-20)
------------------

0.1.1 (2015-11-20)
------------------

0.1.0 (2015-11-19)
------------------
* Initial Ridgeback release.
* Contributors: Mike Purvis, Tony Baltovski

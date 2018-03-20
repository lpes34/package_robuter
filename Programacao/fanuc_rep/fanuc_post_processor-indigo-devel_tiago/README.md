 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Fanuc post processor
=============================

# Travis CI

[![Travis-CI](https://api.travis-ci.org/InstitutMaupertuis/fanuc_post_processor.svg?branch=indigo-devel)](https://travis-ci.org/InstitutMaupertuis/fanuc_post_processor/branches) 

General information
-------------------
This project has been developed by the [Institut Maupertuis](http://www.institutmaupertuis.fr), a French research institute that is working on robotic industrial processes.
This project goal is to allow to create Fanuc LS/TP programs from ROS trajectories.

Directories in the project
--------------------------

| Directory  | Description
------------ | -----------
`fanuc_post_processor` | Meta-package
`fanuc_post_processor_application` | Example usage of the `fanuc_post_processor`.
`fanuc_post_processor_library` | Library to transform 

Dependencies
------------
- [Robot Operating System](http://wiki.ros.org/ROS/Installation)
- [`industrial-core`](http://wiki.ros.org/industrial_core)
- [`libcurl`](https://curl.haxx.se/libcurl/) is used to upload your programs directly to the robot using ASCII Upload Fanuc option.

This package has been tested with Ubuntu 14.04 and ROS Indigo.

Documentation
-------------
Please read [fanuc_post_processor_library/doc/README.md](fanuc_post_processor_library/doc/README.md).

Quick help
----------

**Build**

Clone this repository into your catkin source folder and build the workspace:
```bash
cd $(catkin_workspace)/src
git clone https://github.com/InstitutMaupertuis/fanuc_post_processor.git
```

**Launch**
```bash
roslaunch fanuc_post_processor_application test.launch
```

How to use
----------
See the `fanuc_post_processor_application` package for an example on how to include/use the library.

Troubleshooting upload
----------------------
If you have trouble uploading programs to your robots, here are simple tests to help you find what's wrong:

- Make sure you can ping the robot controller in a terminal: `ping 192.168.1.1`
- Make sure you have ASCII Upload; in your favorite web browser go to the IP address of the robot and check for the Ascii Upload option (R507) on the `Summary Configuration/Status` page
- Make sure you can connect to the FTP (using your credentials if applicable): 
```
$ ftp
ftp> open 192.168.1.1
Connected to 192.168.1.1.
220 R-30iB FTP server ready. [ArcTool V8.20P/24]
Name (192.168.100.200:dell): 
230 User logged in [NORM].
Remote system type is UNKNOWN.
```
- Make sure you can upload a simple program, `ROS_TP_PROGRAM.ls`
```
/PROG ROS_TP_PROGRAM
/ATTR
COMMENT = "ROS generated";
PROTECT = READ_WRITE;
DEFAULT_GROUP = 1,*,*,*,*;
/MN
:  !This is a ROS generated TP program;
:  DO[5]=ON;
:  WAIT  0.5(sec);
:  DO[5]=OFF;
/POS
/END
```

Upload it on with the FTP:
```
ftp> put ./ROS_TP_PROGRAM.ls 
local: ./ROS_TP_PROGRAM.ls remote: ./ROS_TP_PROGRAM.ls
200 PORT command successful.
150 ASCII data connection.
226 ASCII Transfer complete.
215 bytes sent in 0.00 secs (7240.0 kB/s)
```

If the upload loops/fails, check the Teach Pendant alarm history to get details on the error and [open an issue](https://github.com/InstitutMaupertuis/fanuc_post_processor/issues/new).

# APF_Swarm


# Note: Copy all Ardupilot parameter files to default param folder in ArduPilot directory and add the custom vehicle.

add all custom vehicle names to be used to ardupilot/Tools/autotest/pysim/vehicleinfo.py as shown bellow.

	    "gazebo-skywalkerx8-red": {							
                "waf_target": "bin/arduplane",
                "default_params_filename":  "default_params/gazebo-skywalkerx8-red.parm",  
		"external": True,
            },
	    "gazebo-skywalkerx8-green": {								
                "waf_target": "bin/arduplane",
                "default_params_filename":  "default_params/gazebo-skywalkerx8-green.parm", 
		"external": True,
            },
	    "gazebo-skywalkerx8-blue": {			#this is gazebo skywalker3 added....					
                "waf_target": "bin/arduplane",
                "default_params_filename":  "default_params/gazebo-skywalkerx8-blue.parm", 
		"external": True,
            },

# Use src folder to creat catkin workspace as follow

  1.Create the root workspace directory (let say catkin_ws ) cd ~/ mkdir --parents catkin_ws/src cd catkin_ws.
  
  2.Initialize the catkin workspace. ...
  
  3.Build the workspace. ...

# To launch swarm for n=3

Use seperate tabs to execute following commands.

ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-red -L PAF_Kiet  --console  -I0 --out=tcpin:0.0.0.0:8100 --out=127.0.0.1:14552 --out=127.0.0.1:14553

ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-green -L PAF_Kiet  --console  -I1 --out=tcpin:0.0.0.0:8200 --out=127.0.0.1:14562 --out=127.0.0.1:14563

ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-blue -L PAF_Kiet  --console  -I2 –out=tcpin:0.0.0.0:8300 --out=127.0.0.1:14572 --out=127.0.0.1:14573

ubuntu-pc:~$ roslaunch ahfaz_sim skywalkerx8_swarm.launch

execute it at the same location with Python code

ubuntu-pc:~/Drone_Kit$ python Drone_swarm3D.py


# To launch swarm for n=10 

execute following commands in seperate tabs.

 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-red -L PAF_Kiet  --console  -I0

 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-green -L PAF_Kiet  --console  -I1
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-blue -L PAF_Kiet  --console  -I2
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-red2 -L PAF_Kiet  --console  -I3
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-green2 -L PAF_Kiet  --console  -I4
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-blue2 -L PAF_Kiet  --console  -I5
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-red3 -L PAF_Kiet  --console  -I6
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-green3 -L PAF_Kiet  --console  -I7
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-blue3 -L PAF_Kiet  --console  -I8
 
 ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-red4 -L PAF_Kiet  --console  -I9
 
 ubuntu-pc:~$ roslaunch ahfaz_sim skywalkerx8_swarm10_obstacles.launch

 execute it at the same location with Python code
 
 ubuntu-pc:~/Drone_Kit$ python Drone_swarm_10.py


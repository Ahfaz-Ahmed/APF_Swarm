# APF_Swarm


Note# copy all Ardupilot parameter files to default param folder in ArduPilot directory.

To launch swarm for n=3

ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-red -L PAF_Kiet  --console  -I0 --out=tcpin:0.0.0.0:8100 --out=127.0.0.1:14552 --out=127.0.0.1:14553

ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-green -L PAF_Kiet  --console  -I1 --out=tcpin:0.0.0.0:8200 --out=127.0.0.1:14562 --out=127.0.0.1:14563

ubuntu-pc:~$ sim_vehicle.py -v ArduPlane -f gazebo-skywalkerx8-blue -L PAF_Kiet  --console  -I2 –out=tcpin:0.0.0.0:8300 --out=127.0.0.1:14572 --out=127.0.0.1:14573

ubuntu-pc:~$ roslaunch ahfaz_sim skywalkerx8_swarm.launch

ubuntu-pc:~/Drone_Kit$ python Drone_swarm3D.py

To launch swarm for n=10

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
 
 ubuntu-pc:~/Drone_Kit$ python Drone_swarm_10.py


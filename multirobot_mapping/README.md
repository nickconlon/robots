# multirobotmapping
A multirobot occupancy grid mapping and fusion simulation environment for the University of Massachusetts Lowell COMP 5490 fall 2017 final project. 

# To Run:

(1) edit /launch/robot.launch by commenting/uncommenting the required number of robots.

(2) edit /maps/cave.world by commenting/uncommenting lines 59-68 and 72-81 to match the number of robots in step (1)

(3) in terminal ~/catkin_ws run roslaunch multirobotmapping robot.launch

(4) Stage and RVIZ should popup and robots should begin mapping right away

(5) Pause State with 'p'. In RVIZ bottom left goto 'add' -> 'By topic' -> '/fused_map' -> 'OK'. In RVIZ top right change Type to 'TopDownOrtho'. Resume Stae with 'p'. Notice fused map coming tegether in RVIZ!



# To Compare:

(1) whenever the fused map is to your liking, pause Stage with 'p'.

(2) in terminal rosrun map_server map_saver -f mympa /map:=/fused_map

(3) unpause and pause stage. Map is saved by the map server in the current directory.

(4) copy mymap into /scripts. in terminal run python compare.py mymap.png.

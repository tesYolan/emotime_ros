# emotime_ros
this package is ros based emotion detector using emotion detector library called emotime.

#building emotime-ros
1 Install prerequisites

-clone and build emotime from the following repository https://github.com/luca-m/emotime.git 

2. modify LD_LIBRARY_PATH, CPLUS_INCLUDE_PATH to include appropriate emotime library and include files respectively.

3. clone this repository in to your catkin ws

           cd ~/catkin_ws/src
          
           git clone https://github.com/Linaf/emotime_ros.git
           
           cd .. && catkin_make
          
           source ./devel/setup.bash 

#running emotime ros using emotime launch file


          roslaunch emotime emotime.launch
 
         

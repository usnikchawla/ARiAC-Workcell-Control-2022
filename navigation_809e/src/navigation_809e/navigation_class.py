#!/usr/bin/env python
"""
This scripts performs the navigation task for the turtlebot.
This script reads the aruco marker and forms the meassage to be published on /part_infos
"""
import rospy
import sys
from fiducial_msgs.msg import FiducialTransformArray
from enpm809e_msgs.msg import PartInfo, PartInfos
from geometry_msgs.msg import Pose
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Navigation(object):
    """
    A controller class to drive a mobile base in Gazebo.
    """

    def __init__(self, rate=4):
        rospy.init_node('bot_controller')
        rospy.loginfo('Press Ctrl c to exit')
        self._rate = rospy.Rate(rate)
        self._robot_name = 'waffle'
        self.fed_id=[]
        self.sub=None
        self.parts_info_pub=rospy.Publisher('part_infos', PartInfos, queue_size=10)
        
        
        

    def fiducial_callback(self,msg):
        """
        Callback function for the topic /fiducial_transforms
        After the id is read the subscriber for the Fiducial transform is unregisterd.

        Args:
            msg (fiducial_msgs.msg): Fiducial Tranform Array
        """
        if len(msg.transforms)!=0:
            self.fed_id.append(str(msg.transforms[0].fiducial_id))
            self.sub.unregister()
            
    def movebase_client(self, po):
        """
        This function recives the pose of the robot to navigate to and uses the move_base to navigate to it.
        Once it reaches the said pose it creates a subscriber to listen to Fiducual transforms.

        Args:
            po (Pose): pose of the robot to be at.

        
        """
        client = actionlib.SimpleActionClient('/waffle/move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = po[0]
        goal.target_pose.pose.position.y = po[1]
        goal.target_pose.pose.orientation.w = po[2]
        goal.target_pose.pose.orientation.x = po[3]
        goal.target_pose.pose.orientation.y = po[4]
        goal.target_pose.pose.orientation.z = po[5]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            self.sub=rospy.Subscriber("fiducial_transforms",FiducialTransformArray,self.fiducial_callback)
            rospy.sleep(2)
            return client.get_result()
        
    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")

    def handle_inputs(self):
        """
        This function gets the pose of the turlebot to navigate to.
        These poses are read from the parameter server.
        After it gets the poses, it navigates to the poses and reads the aruco marker to gets it's id.
        This function also forms the PartsInfos message and publishes it to the topic /part_infos.
        
        """
        
        for i in range(1,4):
            position_x=rospy.get_param(f"/aruco_lookup_locations/target_{i}/position_x")
            position_y=rospy.get_param(f"/aruco_lookup_locations/target_{i}/position_y")
            orientation_w=rospy.get_param(f"/aruco_lookup_locations/target_{i}/orientation_w")
            orientation_x=rospy.get_param(f"/aruco_lookup_locations/target_{i}/orientation_x")
            orientation_y=rospy.get_param(f"/aruco_lookup_locations/target_{i}/orientation_y")
            orientation_z=rospy.get_param(f"/aruco_lookup_locations/target_{i}/orientation_z")
             
            po=[position_x,position_y,orientation_w,orientation_x,orientation_y,orientation_z]
            
            if None not in po:
                
                self.movebase_client(po)
                
                if i==2:
                    base=f"/kits/aruco_{self.fed_id[0]}"
                    part1_info_msg=PartInfo()
                    part1_info_msg.bin=str(rospy.get_param(base + "/bin"))
                    part1_info_msg.color=rospy.get_param(base + "/part/color")
                    pose1=Pose()
                    pose1.position.x=rospy.get_param(base + "/part/location/position_x")
                    pose1.position.y=rospy.get_param(base + "/part/location/position_y")
                    pose1.position.z=rospy.get_param(base + "/part/location/position_z")
                    part1_info_msg.pose_in_bin=pose1
                    
                    base=f"/kits/aruco_{self.fed_id[1]}"
                    part2_info_msg=PartInfo()
                    part2_info_msg.bin=str(rospy.get_param(base + "/bin"))
                    part2_info_msg.color=rospy.get_param(base + "/part/color")
                    pose2=Pose()
                    pose2.position.x=rospy.get_param(base + "/part/location/position_x")
                    pose2.position.y=rospy.get_param(base + "/part/location/position_y")
                    pose2.position.z=rospy.get_param(base + "/part/location/position_z")
                    part2_info_msg.pose_in_bin=pose2
                    
                    parts_infos_msg=PartInfos()
                    
                    parts_info_list=[part1_info_msg,part2_info_msg]
                    parts_infos_msg.part_infos=parts_info_list
                    self.parts_info_pub.publish(parts_infos_msg)
                    
            
            else:
                rospy.logerr("x or y is missing")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)

    

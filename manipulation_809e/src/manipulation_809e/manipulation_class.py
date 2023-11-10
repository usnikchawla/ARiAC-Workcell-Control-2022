#!/usr/bin/env python

"""
This Script performs the pick and place operation of the project.
This Scipt recieves data from the navigation node on which parts to pick and where they are to be placed. 
"""
# python
from math import prod
import sys
import copy
import tf
import rospy
from geometry_msgs.msg import Pose
from enpm809e_msgs.srv import VacuumGripperControl
from enpm809e_msgs.msg import VacuumGripperState
from enpm809e_msgs.msg import LogicalCameraImage
from enpm809e_msgs.msg import PartInfo, PartInfos

# moveit
import moveit_commander as mc

class Manipulation(object):

    def __init__(self, node_name='manipulation_809e', ns='',robot_description='robot_description'):
        
        """
        Here we initalize the subscriber to the topic /part_infos.
        We also initalize the moveit from kitting arm group.
        """
        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)
        
        self.parts=[]
        #self.part_camera=[]
        self.bins=[]
        rospy.Subscriber("part_infos", PartInfos, self.part_infos_callback)

        self._tf_listener = tf.TransformListener()
        self._parent_frame = 'world'
        self._child_frame = None

        self.br=tf.TransformBroadcaster()
        self.robot = mc.RobotCommander(ns + '/' + robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)

        moveit_group = mc.MoveGroupCommander('kitting_arm', robot_description=ns + '/' + robot_description, ns=ns)
        self.groups = {}
        self.groups['kitting_arm'] = moveit_group
        self._arm_group = self.groups['kitting_arm']
        self._arm_group.set_goal_orientation_tolerance = 0.001
        self._arm_group.set_goal_position_tolerance = 0.001

        

    def main(self):
        """
        Main function to start the Node core.
        We wait until we recive the message from turtlebot.
        """
        #
        rospy.spin()
        
        
    def part_infos_callback(self,msg):
        """
        This is the callback for the topic /part_infos.
        Once we receive the message we retrieve the information on the parts. This include the color of the part and the location it is to be plced at.
        Once we have the location for the part in the bin we do a broadcast for the part to get the location of the part in world frame.
        After we get the pose of the part in the world frame we send this along with current pose of the part to pickandplace function.
        We get the current pose of the part from the logical camera.
        
        """
        
        part_1=msg.part_infos[0]
        bin=part_1.bin
        color=part_1.color
        po=part_1.pose_in_bin
        
        trans_dest=self.get_transform_final(bin,po)
        self.parts_from_camera()
        
        
        try:
            index=self.bins.index(color)
            trans_source=self.parts[index]
            self.pickandplace(trans_source,trans_dest)
        
        except:
            print("Part not in bin")
                    
        part_2=msg.part_infos[1]
        bin=part_2.bin
        color=part_2.color
        po=part_2.pose_in_bin
        
        trans_dest=self.get_transform_final(bin,po)
        self.parts_from_camera()
        
        try:
            index=self.bins.index(color)
            trans_source=self.parts[index]
            self.pickandplace(trans_source,trans_dest)
        
        except:
            print("Part not in bin")
        
        
        
    def get_transform_final(self,bin,po):
        
        """
        This function retrieves the location of the part in the world frame from the current bin frame.
        We first perform broadcast of the location of the part in the bin frame. The we get the location of the part in the world
        frame by reading the transform.
        
        Returns:
            List[int]: The location of the part in the world frame.
        """
        
        try:
            print('Sending Transform')
            now = rospy.Time.now()
            frame="sample" + bin[-1]
            trans=(po.position.x,po.position.y,po.position.z)
            rot=(0,0,0,1)
            self.br.sendTransform(trans,rot,rospy.Time.now(),frame,bin)
            self._tf_listener.waitForTransform(self._parent_frame,frame,now,rospy.Duration(5))
            trans,rot = self._tf_listener.lookupTransform(self._parent_frame, frame, now)
            return trans
            
            
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logfatal("TF Exception")
        
        
    def parts_from_camera(self):
        
        """
        This function reads the pose of the part in the camera frame and stores the location of the part the world frame.
        This fiction does that for both the cameras.
        
        """
        
        msg1=rospy.wait_for_message("/logical_camera/logical_camera_1",LogicalCameraImage)
        msg2=rospy.wait_for_message("/logical_camera/logical_camera_2",LogicalCameraImage)
        
        self.parts.clear()
        self.bins.clear()
        #self.part_index.clear()
        
        # if len(msg1.models)!=0:
            
        #     for index,model in enumerate(msg1.models):
        #         self.bins.append(model.type)
        #         self.part_camera.append("logical_camera_1_")
                
        # if len(msg2.models)!=0:
            
        #     for index,model in enumerate(msg1.models):
        #         self.bins.append(model.type)
        #         self.part_camera.append("logical_camera_2_")
        
        # id=0
        # for ind,part in enumerate(self.bins):
        #     it=self.bins.count(part)
        #     if it>1:
        #         self._child_frame=self.part_camera[ind] + part + "_" + str(id) + "_" + "frame"
        #         self.get_transform()
        #         id+=1
        #     else:
        #         id=0
        #         self._child_frame=self.part_index[ind][0] + part + "_" + str(id) + "_" + "frame"
        #         self.get_transform()
                
                
                
                
            
        
        if len(msg1.models)==2:
            id=0
            type1=msg1.models[0].type
            self.bins.append(type1.split("_")[-1])
            self._child_frame="logical_camera_1_" + type1 + "_" + str(id) + "_" + "frame"
            self.get_transform(type1,id,1)
            
            type2=msg1.models[1].type
            self.bins.append(type2.split("_")[-1])
            if type1==type2:
                id+=1
            self._child_frame="logical_camera_1_" + type2 + "_" + str(id) + "_" + "frame"
            self.get_transform(type2,id,1)
            
        elif len(msg1.models)==1:
            id=0
            type=msg1.models[0].type
            self.bins.append(type.split("_")[-1])
            self._child_frame="logical_camera_1_" + type + "_" + str(id) + "_" + "frame"
            self.get_transform(type,id,1)
        
        else:
            pass
        
        if len(msg2.models)==2:
            id=0
            type1=msg2.models[0].type
            self.bins.append(type1.split("_")[-1])
            self._child_frame="logical_camera_2_" + type1 + "_" + str(id) +"_"+ "frame"
            self.get_transform(type1,id,2)
            
            type2=msg2.models[1].type
            self.bins.append(type2.split("_")[-1])
            if type1==type2:
                id+=1
            self._child_frame="logical_camera_2_" + type2 + "_" + str(id) +"_"+ "frame"
            self.get_transform(type2,id,2)
            
        elif len(msg2.models)==1:
            id=0
            type=msg2.models[0].type
            self.bins.append(type.split("_")[-1])
            self._child_frame="logical_camera_2_" + type + "_" + str(id) +"_"+ "frame"
            self.get_transform(type,id,2)
        
        else:
            pass
        
        print(self.bins)
        print(self.parts)
            
     
        
    def get_transform(self,type,id,camera):
        """
        Get the current pose of the part in the world frame.
        """
        
        try:
            now = rospy.Time.now()
            self._tf_listener.waitForTransform(self._parent_frame,self._child_frame,now,rospy.Duration(5))
            
            trans, rot = self._tf_listener.lookupTransform(self._parent_frame, self._child_frame, now)
            self.parts.append(trans)
            
            
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logfatal("TF Exception")
            
    def reach_goal(self):
        """
        Give a goal to the end effector to reach
        """
        pose_to_reach = copy.deepcopy(self._arm_group.get_current_pose())
        #pose_to_reach.pose.position.x -= 1
        pose_to_reach.pose.position.z += 0.1
        self.cartesian_move([pose_to_reach.pose])
        

    def activate_gripper(self):
        """
        Activate a robot's gripper to grasp objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        try:
            control = rospy.ServiceProxy(
                '/ariac/kitting/arm/gripper/control', VacuumGripperControl)
            result = control(True)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def deactivate_gripper(self):
        """
        Deactivate a robot's gripper to release objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        try:
            control = rospy.ServiceProxy(
                '/ariac/kitting/arm/gripper/control', VacuumGripperControl)
            result = control(False)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def is_object_attached(self):
        """
        Check whether an object is attached to the gripper

        Returns:
            bool: True if an object is attached, otherwise false
        """
        status = rospy.wait_for_message(
            '/ariac/kitting/arm/gripper/state', VacuumGripperState)
        return status.attached

    def move_arm_base(self, x):
        """
        Only move the joint linear_arm_actuator_joint to the x coordinate

        Args:
            x (float): x position in the world frame
        """
        x = -1.5 - x
        arm_joints = [x, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self._arm_group.go(arm_joints, wait=True)

    def pickandplace(self,trans_source, trans_dest):
        """
        trans_source and trans_dest are the pose of the poses of the part.
        """
        pickup_pose = Pose()
        pickup_pose.position.x = trans_source[0]
        pickup_pose.position.y = trans_source[1]
        pickup_pose.position.z = trans_source[2]

        place_pose = Pose()
        place_pose.position.x = trans_dest[0]
        place_pose.position.y = trans_dest[1]
        place_pose.position.z = trans_dest[2]

        self.move_part(pickup_pose, place_pose)

    

    def pickandplace(self,trans_source, trans_dest):
        """
        Hard coded poses for pick and place
        """
        pickup_pose = Pose()
        pickup_pose.position.x = trans_source[0]
        pickup_pose.position.y = trans_source[1]
        pickup_pose.position.z = trans_source[2]

        place_pose = Pose()
        place_pose.position.x = trans_dest[0]
        place_pose.position.y = trans_dest[1]
        place_pose.position.z = trans_dest[2]

        self.move_part(pickup_pose, place_pose)


    def pick_up_part(self, pickup_pose):
        """
        Pick up a part given its pose

        Args:
            pickup_pose (geometry_msgs.Pose): Pose of the part in the 
            world frame
        """

        self.reach_goal()
        # First: get the arm closer to the part
        self.move_arm_base(pickup_pose.position.x)
        
        # This configuration keeps the gripper flat (facing down)
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # position to reach = position of the part
        gripper_position = Pose().position
        gripper_position.x = pickup_pose.position.x
        gripper_position.y = pickup_pose.position.y
        gripper_position.z = pickup_pose.position.z + 0.10

        # combine position + orientation
        above_part_pose = Pose()
        above_part_pose.position = gripper_position
        above_part_pose.orientation = flat_gripper
        self.cartesian_move([above_part_pose])


        # activate gripper
        self.activate_gripper()

        # slowly move down until the part is attached to the gripper
        part_is_attached = self.is_object_attached()
        while not part_is_attached:
            pickup_pose = copy.deepcopy(self._arm_group.get_current_pose())
            pickup_pose.pose.position.z -= 0.001
            self.cartesian_move([pickup_pose.pose])
            part_is_attached = self.is_object_attached()
            rospy.sleep(0.2)

        above_part_pose.position.z+=0.1
        self.cartesian_move([above_part_pose])

    def place_part(self, place_pose):
        """
        Place a part to the given pose

        Args:
            place_pose (geometry_msgs.Pose): Pose of the part in the
            world frame
        """
        # move the arm closer to the drop pose
        self.move_arm_base(place_pose.position.x)

        # ensure the gripper is facing down
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # set the position to reach
        gripper_position = Pose().position
        gripper_position.x = place_pose.position.x
        gripper_position.y = place_pose.position.y
        gripper_position.z = place_pose.position.z + 0.20

        # set the pose = position + orientation
        above_bin_pose = Pose()
        above_bin_pose.position = gripper_position
        above_bin_pose.orientation = flat_gripper
        self.cartesian_move([above_bin_pose])

        # get the pose of the gripper and make it move a bit lower
        # before releasing the part
        current_arm_pose = copy.deepcopy(self._arm_group.get_current_pose())
        current_arm_pose.pose.position.z -= 0.02
        self.cartesian_move([current_arm_pose.pose])

        # deactivate gripper
        self.deactivate_gripper()
        


    def move_part(self, pickup_pose, place_pose):
        """
        Move a part from one pose to another pose

        Args:
            pickup_pose (geometry_msgs.Pose): Current pose of the part in world frame
            place_pose (geometry_msgs.Pose): Pose of the part in the bin in the world frame

        Returns:
            bool: True
        """

        self.pick_up_part(pickup_pose)
        self.place_part(place_pose)

        return True

    def cartesian_move(self, waypoints):
        """
        Move the robotic arm through waypoints

        Args:
            waypoints (List(geometry_msgs.Pose)): List of waypoints
        """
        self._arm_group.set_pose_reference_frame("world")
        (plan, fraction) = self._arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        self._arm_group.execute(plan, wait=True)

    def go_home(self):
        """
        Move the robotic arm to the 'home' preset
        location
        """
        self.goto_preset_location('home')

    def goto_preset_location(self, location_name):
        """
        Move the robotic arm to a pre-set location

        Args:
            location_name (str): Pre-set location
        """
        arm = self.locations[location_name]
        location_pose = self._arm_group.get_current_joint_values()
        location_pose[:] = arm
        self._arm_group.go(location_pose, wait=True)

    

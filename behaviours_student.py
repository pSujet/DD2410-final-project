import actionlib
import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

from geometry_msgs.msg import PoseStamped, Vector3Stamped, PoseWithCovarianceStamped
from rospy.msg import AnyMsg
from std_srvs.srv import Empty, SetBool, SetBoolRequest
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionFeedback, MoveBaseGoal, MoveBaseResult
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
from colorama import Fore, Back, Style
from trajectory_msgs.msg import JointTrajectoryPoint

import threading


# Check Localization
LOCALIZED = False; TRIED_LOCALIZATION = False
# checkpick, checekplace
PICKED = False; PLACED = False

# Move head
HEADUP = False; HEADDOWN = False

# Status of move base behavior, Check if it tried to move
MOVEBASESTATUS = 0
TRIED_MOVE = False


# Gripper, cube detected after place action, cube reinitialization
GRIPPER_THRESHOLD = 0.014
DETECTED = False
CUBESTATE = True
CUBEPLACED = False
TRIED_PICK = False
TRIED_PLACE = False
DETECTPLACEDCUBE = True

TUCK_ARM = False
TUCK_STATUS = 0

CANCELMOVE = False
ROTATE_CCW = False
ROTATE_CW = False

#############################################################################################################################
#                                                                                                                           #
#                                                                                                                           #
#                                                      LOCALIZATION                                                         #
#                                                                                                                           #
#                                                                                                                           #
#############################################################################################################################
#####################################################################################################
#                                            rotate_cw Class                                           #
#####################################################################################################
class rotate_cw(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, angle):

        rospy.loginfo("Initialising rotate behaviour.")

        # action space
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.angular.z = - 0.5
        self.angle = angle

        # become a behaviour
        super(rotate_cw, self).__init__('Rotate robot cw')

    def update(self):
        global ROTATE_CW
        
        if ROTATE_CW:
            print('ROTATE_CW : SUCCESS')
            return pt.common.Status.SUCCESS

        rate = rospy.Rate(10)
        # send the message        
        for _ in range(20*int(self.angle)):            
            self.cmd_vel_pub.publish(self.move_msg)
            rospy.loginfo("I am SPINNINGGGGG CW!!!!!!!")
            rate.sleep()
        ROTATE_CW = True
        self.move_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_msg)
        # tell the tree that you're running
        return pt.common.Status.SUCCESS
#####################################################################################################
#                                            rotate_ccw Class                                           #
#####################################################################################################
class rotate_ccw(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, angle):

        rospy.loginfo("Initialising rotate behaviour.")

        # action space
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.angular.z = 0.5
        self.angle = angle

        # become a behaviour
        super(rotate_ccw, self).__init__('Rotate robot ccw')

    def update(self):
        global ROTATE_CCW
        
        if ROTATE_CCW:
            print('ROTATE_CW : SUCCESS')
            return pt.common.Status.SUCCESS

        rate = rospy.Rate(10)
        # send the message        
        for _ in range(20*int(self.angle)):            
            self.cmd_vel_pub.publish(self.move_msg)
            rospy.loginfo("I am SPINNINGGGGG CCW!!!!!!!")
            rate.sleep()
        ROTATE_CCW = True
        self.move_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_msg)
        # tell the tree that you're running
        return pt.common.Status.SUCCESS

#####################################################################################################
#                                           GO Class                                           #
#####################################################################################################
class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):
        global TUCK_ARM

        if not TUCK_ARM:
            # increment i
            self.i += 1

            # succeed after count is done
            if self.i <= self.n:
                print("Count FAILURE")
                return pt.common.Status.FAILURE 
            else:
                self.i = 0
                print("Count SUCCESS")
                return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.loginfo("I am SPINNINGGGGG!!!!!!!")
        rate.sleep()
        

        # tell the tree that you're running
        return pt.common.Status.RUNNING


#####################################################################################################
#                                           tuckarm Class                                           #
#####################################################################################################
class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.status = 0
        # self.finished = False

        # add go
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = 0

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        global TUCK_ARM, TUCK_STATUS
        # already tucked the arm
        # if self.finished: 
        #     return pt.common.Status.SUCCESS
        # command to tuck arm if haven't already
        # elif not self.sent_goal:
        rospy.loginfo(Fore.CYAN + 'Tuck arm = in update')
        print("TUCK_STATUS" + str(TUCK_STATUS))
        if TUCK_ARM:
            self.sent_goal = False
            rospy.loginfo(Fore.CYAN + 'Tuck arm = in 1 if')
            return pt.common.Status.SUCCESS
        else:
            if not self.sent_goal:
                self.play_motion_ac.send_goal(self.goal,active_cb=None, feedback_cb=self.tuckarm_fb_cb, done_cb=self.tuckarm_res_cb)
                self.sent_goal = True
                rospy.loginfo(Fore.RED + 'Tuck arm = send goal')
                # self.play_motion_ac.wait_for_result()                
                # rospy.loginfo(Fore.GREEN + 'Tuck arm = Running')
            
            if self.play_motion_ac.get_result():
                TUCK_STATUS == 0
                TUCK_ARM = True
                self.sent_goal = False
                rospy.loginfo(Fore.RED + 'Tuck arm = SUCCESS!!!')
                return pt.common.Status.SUCCESS
            elif TUCK_STATUS == 0:
                rate = rospy.Rate(10)
                self.cmd_vel_pub.publish(self.move_msg)
                rospy.loginfo(Fore.CYAN + 'Tuck arm = Running')
                return pt.common.Status.RUNNING
            else:
                rospy.loginfo(Fore.CYAN + 'Tuck arm = Failure')
                return pt.common.Status.FAILURE

    def tuckarm_fb_cb(self):
        rospy.loginfo(Fore.CYAN + 'Tuck arm = in fb')
        TUCK_STATUS = 0
    def tuckarm_res_cb(self, state, msg):
        if actionlib.TerminalState.SUCCEEDED == state:
            # self.status = 1
            TUCK_STATUS = 1
        else:
            # self.status = 2
            TUCK_STATUS = 2

#####################################################################################################
#                                    checklocalization Class                                        #
#####################################################################################################
class checklocalization(pt.behaviour.Behaviour):
    def __init__(self): 
        # rospy.loginfo('CHECK LOCALIZATION INIT')
        rospy.loginfo("Initializing check for localization")
        amcl_estimate = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)
        # print("1111111111111111111111111111111111111111111111111")

        super(checklocalization, self).__init__("Check if localized")
         
    def update(self):
        # print("2222222222222222222222222222222222222222222222222")
        #rospy.loginfo('CHECK LOCALIZATION UPDATE')
        global LOCALIZED

        if LOCALIZED:
            # rospy.loginfo(Fore.GREEN + 'CHECK LOCALIZATION UPDATE = Success')
            return pt.common.Status.SUCCESS
        else:
            # rospy.loginfo(Fore.GREEN + 'CHECK LOCALIZATION UPDATE = Fail')
            return pt.common.Status.FAILURE 
    def amcl_estimate_cb(self, msg):
        global LOCALIZED
        # if np.linalg.norm(msg.pose.covariance) > 0.04:
        if np.linalg.norm(msg.pose.covariance) > 0.032:
            self.reached_position = False
            LOCALIZED = False

#####################################################################################################
#                                      localization Class                                           #
#####################################################################################################
class localization(pt.behaviour.Behaviour):
    def __init__(self):
        #rospy.loginfo('LOCALIZATION INIT')
        rospy.loginfo('Initializing localization Behavior')

        # Parameters
        global_loc_srv = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        cmd_vel_topic = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        amcl_estimate = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Services
        self.global_loc_srv_client = rospy.ServiceProxy(global_loc_srv, Empty)
        rospy.wait_for_service(global_loc_srv, timeout = 30)

        # Topics (Pubs & Subs)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
        rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)

        rospy.sleep(3)

        # execution checker
        self.tried = False
        self.done = False
        
        super(localization, self).__init__("Localize Robot!")

    def amcl_estimate_cb(self, msg):
        self.amcl_estimation = msg
    
    def update(self):

        # global DICT
        global LOCALIZED, TRIED_LOCALIZATION
        global TRIED_MOVE

        if LOCALIZED:
            #TRIED_LOCALIZATION = False
            return pt.common.Status.SUCCESS

        if not TRIED_LOCALIZATION:
            TRIED_LOCALIZATION = True
            self.global_loc_srv_client()

        rate = rospy.Rate(10) 
        rate.sleep()        ################################
        vel = Twist()

        # if np.linalg.norm(self.amcl_estimation.pose.covariance) > 0.025:
        if np.linalg.norm(self.amcl_estimation.pose.covariance) > 0.02:
            vel.angular.z = 0.5
            self.cmd_vel_pub.publish(vel)
            #rospy.loginfo(Fore.GREEN + 'LOCALIZATION UPDATE = Running')
            return pt.common.Status.RUNNING
        else:
            vel.angular.z = 0
            self.cmd_vel_pub.publish(vel)
            LOCALIZED = True
            TRIED_LOCALIZATION = False
            rospy.loginfo(Fore.GREEN + 'LOCALIZATION UPDATE = Success')

            # TRIED_MOVE = False
            return pt.common.Status.SUCCESS
      
        # global LOCALIZED, TRIED_LOCALIZATION
        
        # # Success if done
        # if LOCALIZED:
        #     return pt.composites.Status.SUCCESS
        # # Try if not tried
        # if not TRIED_LOCALIZATION:
        #     # Command
        #     self.localize_req = self.global_loc_srv_client()
        #     TRIED_LOCALIZATION = True

        # rospy.sleep(3)
        
        # vel = Twist()
        # if np.linalg.norm(self.amcl_estimation.pose.covariance) > 0.025:
        #     vel.angular.z = 1.0
        #     self.cmd_vel_pub.publish(vel)
        #     return pt.common.Status.RUNNING
        # else:
        #     vel.angular.z = 0.0
        #     self.cmd_vel_pub.publish()
        #     LOCALIZED = True
        #     TRIED_LOCALIZATION = False
        #     self.cnt = 0
        #     return pt.common.Status.SUCCESS

#####################################################################################################
#                                      Clear Costmap Class                                          #
#####################################################################################################
class clearcostmaps(pt.behaviour.Behaviour):
    def __init__(self):
        clear_costmaps_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        # rospy.loginfo('CLEAR COSTMAPS INIT')
        # Services
        self.clear_costmaps_srv_client = rospy.ServiceProxy(clear_costmaps_srv, Empty)
        rospy.wait_for_service(clear_costmaps_srv, timeout = 30)

        super(clearcostmaps, self).__init__('Clear costmaps')
        # execution checker
        self.tried = False
        self.done = False
    
    def update(self):
        self.clear_costmaps_srv_client()
        rospy.loginfo(Fore.GREEN + 'CLEAR COSTMAPS = Success')
        rospy.sleep(1)
        return pt.common.Status.SUCCESS
        # if self.done:
        #     rospy.loginfo(Fore.GREEN + 'CLEAR COSTMAPS = Success')
        #     return pt.common.Status.SUCCESS
        # else: #not self.tried:
        #     self.req = self.clear_costmaps_srv_client()
        #     rospy.loginfo(Fore.GREEN + 'CLEAR COSTMAPS = Running')
        #     self.tried = True
        #     #rospy.sleep(1)
        #     return pt.common.Status.RUNNING
        # elif self.req:
        #     self.done = True
        #     self.tried = False
        #     return pt.common.Status.SUCCESS
        # else:
        #     return pt.common.Status.FAILURE

#############################################################################################################################
#                                                                                                                           #
#                                                                                                                           #
#                                                        PICK CUBE                                                          #
#                                                                                                                           #
#                                                                                                                           #
#############################################################################################################################

#####################################################################################################
#                                       cube state Class                                            #
#####################################################################################################
class cubestate(pt.behaviour.Behaviour):

    def __init__(self):
        super(cubestate, self).__init__()

    def update(self):
        global CUBESTATE
        if CUBESTATE == True:
            print("CUBESTATE = SUCCESS")
            return pt.common.Status.SUCCESS
        else:
            print("CUBESTATE = FAIL")
            return pt.common.Status.FAILURE

#####################################################################################################
#                                      checkgripper Class                                          #
#####################################################################################################
class checkgripper(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.Subscriber('/gripper_controller/state', JointTrajectoryControllerState, self.check_gripper_cb)
        super(checkgripper, self).__init__('Check Gripper')

    def update(self):
        global GRIPPER_THRESHOLD, CUBESTATE, PICKED,DETECTPLACEDCUBE
    
        if abs(self.gripper_state.actual.positions[0]) < GRIPPER_THRESHOLD and abs(self.gripper_state.actual.positions[1]) < GRIPPER_THRESHOLD :
            rospy.loginfo(Fore.GREEN + 'CHECK GRIPPER = Fail')
            # rospy.loginfo('gripper state = {}' .format(self.gripper_state.actual.positions))
            DETECTPLACEDCUBE = False
            return pt.common.Status.FAILURE
          
        else:
            rospy.loginfo(Fore.GREEN + 'CHECK GRIPPER = Success')
            return pt.common.Status.SUCCESS


    def check_gripper_cb(self, msg):
        self.gripper_state = msg

#####################################################################################################
#                                       setcubestate Class                                          #
#####################################################################################################
class setcubestate(pt.behaviour.Behaviour):
    def __init__(self):
        super(setcubestate, self).__init__('Set cube state')

    def update(self):
        global CUBESTATE, DETECTPLACEDCUBE
        CUBESTATE = True
        DETECTPLACEDCUBE = True
        return pt.common.Status.SUCCESS

#####################################################################################################
#                                           reinitcube Class                                       #
#####################################################################################################
class reinitcube(pt.behaviour.Behaviour):
    def __init__(self):
        self.set_cube_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('/gazebo/set_model_state', timeout = 30)
        self.model_state = ModelState()
        self.model_state.model_name = 'aruco_cube'
        self.model_state.pose.position.x = -1.130530
        self.model_state.pose.position.y = -6.653650
        self.model_state.pose.position.z = 0.86250
        self.model_state.pose.orientation.x = 0
        self.model_state.pose.orientation.y = 0
        self.model_state.pose.orientation.z = 0
        self.model_state.pose.orientation.w = 1
        self.model_state.twist.linear.x = 0
        self.model_state.twist.linear.y = 0
        self.model_state.twist.linear.z = 0
        self.model_state.twist.angular.x = 0
        self.model_state.twist.angular.y = 0
        self.model_state.twist.angular.z = 0
        self.model_state.reference_frame = 'map'

        super(reinitcube, self).__init__('Reinitialize cube')

    def update(self):
        
        # Over here I might need to add conditions to check for the state of the cube (using the data from topic /robotics_intro/aruco_single/position)
        global CUBESTATE, PICKED, TRIED_MOVE, TRIED_PICK, TRIED_PLACE, TUCK_ARM, CANCELMOVE, MOVEBASESTATUS, ROTATE_CCW, ROTATE_CW
        if TRIED_PICK:
            self.move_head_req = self.set_cube_state_srv(self.model_state)
            PICKED = False
            TRIED_MOVE =False
            TRIED_PICK = False
            TRIED_PLACE = False
            # CUBESTATE = True
            TUCK_ARM = False
            CANCELMOVE = True
            MOVEBASESTATUS = 0
            ROTATE_CCW = False
            ROTATE_CW = False
            rospy.loginfo('does it work???????????????????????????????')

        rospy.loginfo(Fore.GREEN + 'REINITCUBE = Success')
        return pt.common.Status.SUCCESS

#####################################################################################################
#                                     checkpickcube Class                                           #
#####################################################################################################
class checkpickcube(pt.behaviour.Behaviour):
    def __init__(self):
        # rospy.loginfo('CHECK PICK CUBE INIT')
        super(checkpickcube, self).__init__('Check if cube has been picked')

    def update(self):
        
        global PICKED, MOVEBASESTATUS, TRIED_PICK
        
        if PICKED:
            rospy.loginfo(Fore.GREEN + 'CHECK PICK CUBE = Success')
            return pt.common.Status.SUCCESS
            
        else:
            rospy.loginfo(Fore.GREEN + 'CHECK PICK CUBE = Fail')
            if TRIED_PICK:
                MOVEBASESTATUS = 0
            return pt.common.Status.FAILURE

#####################################################################################################
#                                         pickcube Class                                            #
#####################################################################################################
class pickcube(pt.behaviour.Behaviour):

    # def __init__(self, cube_pose_): # Use this for E grade
    def __init__(self):
        # rospy.loginfo('PICK CUBE INIT')

        rospy.loginfo("Initialising pick cube behaviour.")
        
        # print('Picking cube')
        '''
        Use this for E grade

        self.cube_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.cube_pose_pub = rospy.Publisher(self.cube_pose_topic, PoseStamped, queue_size = 10)                
        self.pose = PoseStamped()

        cube_pose = list(cube_pose_.split(', '))
        for i in range(len(cube_pose)):
            cube_pose[i] = float(cube_pose[i])

        self.pose.pose.position.x = cube_pose[0]
        self.pose.pose.position.y = cube_pose[1]
        self.pose.pose.position.z = cube_pose[2]
        self.pose.pose.orientation.x = cube_pose[3]
        self.pose.pose.orientation.y = cube_pose[4]
        self.pose.pose.orientation.z = cube_pose[5]
        self.pose.pose.orientation.w = cube_pose[6]
        self.pose.header.frame_id = "/base_footprint"
        '''

        # server
        my_pick_cube_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_cube_srv = rospy.ServiceProxy(my_pick_cube_srv_nm, SetBool)
        rospy.wait_for_service(my_pick_cube_srv_nm, timeout=30)

        # rospy.sleep(3)

        # execution checker
        self.tried = False
        # self.done = False

        # become a behaviour
        super(pickcube, self).__init__("Pick Cube!")
    

    def update(self):
        global PICKED, TRIED_PICK

        if not self.tried:

            # command
            self.pick_cube_req = self.pick_cube_srv()
            self.tried = True
            TRIED_PICK = True

            # tell the tree you're running
            # rospy.loginfo(Fore.GREEN + 'PICK CUBE = Running')
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_cube_req.success:
            self.tried = False
            PICKED = True
            rospy.loginfo(Fore.GREEN + 'PICK CUBE = Success')
            # rospy.sleep(1)
            rospy.loginfo(Fore.GREEN + 'Woke up!!!!!!!!!!!')
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_cube_req.success:
            # rospy.loginfo(Fore.GREEN + 'PICK CUBE = Fail')
            return pt.common.Status.FAILURE

        # if still trying
        else:
            # rospy.loginfo(Fore.GREEN + 'PICK CUBE = Running')
            return pt.common.Status.RUNNING

#############################################################################################################################
#                                                                                                                           #
#                                                                                                                           #
#                                                        PLACE CUBE                                                         #
#                                                                                                                           #
#                                                                                                                           #
#############################################################################################################################

#####################################################################################################
#                                checkdetectedplacecube Class                                       #
#####################################################################################################
class checkdetectedplacecube(pt.behaviour.Behaviour):
    def __init__(self):
        # rospy.loginfo('CHECK PICK CUBE INIT')
        super(checkdetectedplacecube, self).__init__('Check...what???')

    def update(self):
        
        global DETECTPLACEDCUBE
        
        if DETECTPLACEDCUBE:
            rospy.loginfo(Fore.GREEN + 'DETECTPLACEDCUBE = Success')
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo(Fore.GREEN + 'DETECTPLACEDCUBE = Fail')
            return pt.common.Status.FAILURE

#####################################################################################################
#                                   detectplacedcube Class                                          #
#####################################################################################################
class detectplacedcube(pt.behaviour.Behaviour):
    def __init__(self, task):
        rospy.loginfo('Initliazing Detect Placed Cube Behavior!')
        self.task = task

        # Subscriber: /robotics_intro/aruco_single/position
        rospy.Subscriber('/robotics_intro/aruco_single/position', Vector3Stamped, self.detect_cube_cb)

        super(detectplacedcube, self).__init__('Detect placed cube')

    def update(self):
        global PLACED, PICKED, DETECTED, CUBEPLACED, CUBESTATE, DETECTPLACEDCUBE
        # try:
        #     rospy.wait_for_message('/robotics_intro/aruco_single/position', Vector3Stamped, timeout=5)
        #     DETECTED = True
        #     if self.task == 'pick':
        #         PICKED = False
        #         CUBESTATE = False
        #         return pt.common.Status.FAILURE
        #     if self.task == 'place':
        #         PLACED = True
        #         return pt.common.Status.SUCCESS
        # except:
        #     DETECTED = False        
        #     if self.task == 'pick':
        #         PICKED = True
        #         return pt.common.Status.SUCCESS
        #     if self.task == 'place':
        #         PLACED = False
        #         CUBESTATE = False
        #         return pt.common.Status.FAILURE    
        try:
            rospy.wait_for_message('/robotics_intro/aruco_single/position', Vector3Stamped, timeout=5)
            DETECTED = True
            if self.task == 'pick':
                PICKED = False
                CUBESTATE = False
            if self.task == 'place':
                PLACED = True
            return pt.common.Status.SUCCESS
        except:
            DETECTED = False        
            if self.task == 'pick':
                PICKED = True
            if self.task == 'place':
                PLACED = False
                CUBESTATE = False
                DETECTPLACEDCUBE = False
                print("detectplacedcube = FAIL")
            return pt.common.Status.FAILURE

        # if PLACED:
        #     if DETECTED:
        #         # CUBEPLACED = True
        #         return pt.common.Status.SUCCESS
        #     else:
        #         PICKED = False
        #         PLACED = False
        #         # CUBEPLACED = False
        #         return pt.common.Status.FAILURE

        return super().update()

    def detect_cube_cb(self, msg):
        pass
        # rospy.loginfo('CUBE DETECTED FOR TASK' + self.task)
        # global PLACED, DETECTED, PICKED
        # # rospy.loginfo('DETECT_ARUCO_CB FUNCTION')
        # if PLACED == True and PICKED == True:
        #     rospy.loginfo('DETECT_ARUCO_CB FUNCTION IF STATEMENT TRUE!!')
        #     DETECTED = True
            
#####################################################################################################
#                                     checkplacecube Class                                          #
#####################################################################################################
class checkplacecube(pt.behaviour.Behaviour):
    def __init__(self):
        # rospy.loginfo('CHECK PLACE CUBE INIT')
        super(checkplacecube, self).__init__('Check if cube has been placed')

    def update(self):
        
        global PLACED, TRIED_MOVE, MOVEBASESTATUS, TRIED_PLACE
        if PLACED:
            rospy.loginfo(Fore.GREEN + 'CHECK PLACE CUBE = Success')
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo(Fore.RED + 'CHECK PLACE CUBE = Fail')
            if TRIED_PLACE:
                MOVEBASESTATUS = 0
            return pt.common.Status.FAILURE

#####################################################################################################
#                                        placecube Class                                            #
#####################################################################################################
class placecube(pt.behaviour.Behaviour):

    def __init__(self):
        # rospy.loginfo('PALCE CUBE INIT')
        rospy.loginfo("Initialising place cube behaviour.")
        
        # print('Placing cube')

        # server
        my_place_cube_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_cube_srv = rospy.ServiceProxy(my_place_cube_srv_nm, SetBool)
        rospy.wait_for_service(my_place_cube_srv_nm, timeout=30)

        rospy.sleep(3)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(placecube, self).__init__("Place Cube!")
    

    def update(self):
        global PLACED, TRIED_PLACE
        # success if done
        # if self.done:
        #     rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Success')
        #     PLACED = True
        #     return pt.common.Status.SUCCESS

        # try if not tried
        if not self.tried:

            # command
            self.place_cube_req = self.place_cube_srv()
            self.tried = True
            TRIED_PLACE = True

            # tell the tree you're running
            rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Running')
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_cube_req.success:
            self.tried = False
            rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Success')
            PLACED = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_cube_req.success:
            rospy.loginfo(Fore.RED + 'PLACE CUBE = Fail')
            PLACED = False
            return pt.common.Status.FAILURE

        # if still trying
        else:
            # rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Running')
            return pt.common.Status.RUNNING

#############################################################################################################################
#                                                                                                                           #
#                                                                                                                           #
#                                                       OTHERS CUBE                                                         #
#                                                                                                                           #
#                                                                                                                           #
#############################################################################################################################

#####################################################################################################
#                                     check cube Class                                              #
#####################################################################################################
class checkcube(pt.behaviour.Behaviour):
    def __init__(self):
        super(checkcube, self).__init__('Check cube has been placed in the position')

    def update(self):
        global CUBEPLACED, CUBESTATE , DETECTED

        
        if DETECTED == True:
            rospy.loginfo(Fore.GREEN + 'CHECK CUBESTATE PLACED = Success')
            return pt.common.Status.SUCCESS
            
        else:
            rospy.loginfo(Fore.GREEN + 'CHECK CUBESTATE PLACED = Fail')
            CUBESTATE = False
            return pt.common.Status.FAILURE

#####################################################################################################
#                                           movehead Class                                          #
#####################################################################################################
class movehead(pt.behaviour.Behaviour):

    def __init__(self, direction):
        global HEAD
        #rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction
        rospy.sleep(3)

        # become a behaviour
        super(movehead, self).__init__("Move head!")


    def update(self):
        global HEADUP, HEADDOWN
        # print('moving the head {}' .format(self.direction))
        if self.direction == 'up':
            if HEADUP:
                rospy.loginfo(Fore.GREEN + 'MOVE HEAD UP = Success')
                return pt.common.Status.SUCCESS
            else:
                move_head_srv_req = self.move_head_srv('up')

                if move_head_srv_req:
                    HEADUP = True
                    HEADDOWN = False
                    rospy.loginfo(Fore.GREEN + 'MOVE HEAD UP = Success')
                    return pt.common.Status.SUCCESS
                rospy.loginfo(Fore.GREEN + 'MOVE HEAD UP = Running')

                return pt.common.Status.RUNNING
        elif self.direction == 'down':
            if HEADDOWN:
                rospy.loginfo(Fore.GREEN + 'MOVE HEAD DOWN = Success')
                return pt.common.Status.SUCCESS
            else:
                move_head_srv_req = self.move_head_srv('down')

                if move_head_srv_req:
                    HEADDOWN = True
                    HEADUP = False
                    rospy.loginfo(Fore.GREEN + 'MOVE HEAD DOWN = Success')
                    return pt.common.Status.SUCCESS
                rospy.loginfo(Fore.GREEN + 'MOVE HEAD DOWN = Running')
                return pt.common.Status.RUNNING            
        else:
            rospy.loginfo('Invalid heaed motion was given. The motion \'{}\' was given whereas the valid inputs are \'up\' annd \'down\' ', format({self.direction}))
            rospy.loginfo(Fore.GREEN + 'MOVE HEAD = Failure')
            return pt.common.Status.FAILURE

#####################################################################################################
#                                           move Class                                              #
#####################################################################################################
class move(pt.behaviour.Behaviour):
    def __init__(self, goal_pose):
        # rospy.loginfo('MOVE INIT')
        
        rospy.loginfo('Initializing move process')

        self.goal_pose = goal_pose
        self.reached_position = False
        self.in_progress = False

        #self.status = {"MOVEBASESTATUS":"PENDING"}
        #self.status = 0

        # Parameters

        pick_pose_topic = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        place_pose_topic = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        # clear_costmaps_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        amcl_estimate = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Action Client
        # self.move_client = SimpleActionClient('/move_base', MoveBaseAction)


        # Topics (Pubs & Subs)
        rospy.Subscriber(pick_pose_topic, PoseStamped, self.pick_pose_cb)
        rospy.Subscriber(place_pose_topic, PoseStamped, self.place_pose_cb)
        rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)


        self.move_client = SimpleActionClient('/move_base', MoveBaseAction)
        if not self.move_client.wait_for_server(rospy.Duration(20)):
            rospy.loginfo('Failed to connect with /move_base action server!')
            exit()
        

        # try:
        #     self.move_client.wait_for_server(rospy.Duration(30))
        # except:
        #     rospy.loginfo('Failed to connect with /move_base action server!')
        #     exit()


        # # Topics (Pubs & Subs)
        # rospy.Subscriber(pick_pose_topic, PoseStamped, self.pick_pose_cb)
        # rospy.Subscriber(place_pose_topic, PoseStamped, self.place_pose_cb)
        # rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)

        super(move, self).__init__('Move robot')



    def update(self):
        global MOVEBASESTATUS, TRIED_MOVE, CANCELMOVE
        self.move_goal = MoveBaseGoal()
        print(Fore.YELLOW + 'MOVEBASESTATUS = {}' .format(MOVEBASESTATUS))

        # Check what the goal pose is
        if self.goal_pose == 'pick':
            target = "pick"
            self.move_goal.target_pose = self.pick_pose
        if self.goal_pose == 'place':
            target = "place"
            self.move_goal.target_pose = self.place_pose

        if CANCELMOVE:
            self.move_client.cancel_all_goals()
            CANCELMOVE = False
            return pt.common.Status.FAILURE

        # If it has already succeeded in moving then no need to send goal again
        if MOVEBASESTATUS == 1:
            return pt.common.Status.SUCCESS

        # If it hasn't already tried moving then move (send goal)
        if not TRIED_MOVE:
            TRIED_MOVE = True
            print(Fore.BLUE + 'SEND GOAL' + target)
            try:
                self.move_client.cancel_all_goals()
            except:
                pass
            self.move_client.send_goal(goal = self.move_goal, active_cb = self.move_active_cb, feedback_cb = self.move_fb_cb, done_cb=self.move_res_cb)
           
        if MOVEBASESTATUS == 0:
            rospy.loginfo(Fore.BLUE + 'MOVE UPDATE = Running')
            return pt.common.Status.RUNNING
        elif MOVEBASESTATUS == 1:
            rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Success')  
            MOVEBASESTATUS = 0

            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo(Fore.RED + 'MOVE UPDATE = Fail')
            # MOVEBASESTATUS = 0
            return pt.common.Status.FAILURE
                    

    def move_active_cb(self):
        pass

    # Action callbacks
    def move_fb_cb(self, msg):
        global MOVEBASESTATUS
        MOVEBASESTATUS = 0
        return pt.common.Status.RUNNING
        

    def move_res_cb(self, state, msg):
        global MOVEBASESTATUS, TRIED_MOVE

        TRIED_MOVE = False
        if actionlib.TerminalState.SUCCEEDED == state:
            # self.status = 1
            MOVEBASESTATUS = 1
            
            rospy.loginfo('MOVEBASESTATUS IN RESULT SUCCESS = {}' .format(MOVEBASESTATUS))
        else:
            # self.status = 2
            MOVEBASESTATUS = 2
            rospy.loginfo('MOVEBASESTATUS IN RESULT FAIL = {}' .format(MOVEBASESTATUS))            

    # Subscriber callbacks
    def pick_pose_cb(self, msg):
        self.pick_pose = msg

    def place_pose_cb(self, msg):
        self.place_pose = msg

    def amcl_estimate_cb(self, msg):
        global LOCALIZED
        # if np.linalg.norm(msg.pose.covariance) > 0.04:
        if np.linalg.norm(msg.pose.covariance) > 0.032:
            self.reached_position = False
            # LOCALIZED = False
            # Cancel action goals & clear costmap
            self.move_client.cancel_all_goals()
            # self.clear_costmaps_srv_client()
            LOCALIZED = False




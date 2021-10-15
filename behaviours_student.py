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

#############################################################################################################################
#                                                                                                                           #
#                                                                                                                           #
#                                                      LOCALIZATION                                                         #
#                                                                                                                           #
#                                                                                                                           #
#############################################################################################################################


#####################################################################################################
#                                             GO Class                                              #
#####################################################################################################
class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        print("Initialising counter behaviour.")

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
                return pt.common.Status.FAILURE 
            else:
                self.i = 0
                return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.SUCCESS

class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        print("Initialising go behaviour.")

        # Publisher init
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # Become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # Send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
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

        print("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # Personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # Execution checker
        self.sent_goal = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        global TUCK_ARM, TUCK_STATUS

        if TUCK_ARM:
            self.sent_goal = False
            return pt.common.Status.SUCCESS
        else:
            if not self.sent_goal:
                self.play_motion_ac.send_goal(self.goal,active_cb=None, feedback_cb=self.tuckarm_fb_cb, done_cb=self.tuckarm_res_cb)
                self.sent_goal = True
                print(Fore.WHITE + 'Tucking arm ...')
            
            if self.play_motion_ac.get_result():
                TUCK_STATUS == 0
                TUCK_ARM = True
                self.sent_goal = False
                return pt.common.Status.SUCCESS
            elif TUCK_STATUS == 0:
                return pt.common.Status.RUNNING
            else:
                print(Fore.CYAN + 'Failed to tuck arm')
                return pt.common.Status.FAILURE

    def tuckarm_fb_cb(self):
        TUCK_STATUS = 0
    def tuckarm_res_cb(self, state, msg):
        if actionlib.TerminalState.SUCCEEDED == state:
            TUCK_STATUS = 1
        else:
            TUCK_STATUS = 2

#####################################################################################################
#                                    checklocalization Class                                        #
#####################################################################################################
class checklocalization(pt.behaviour.Behaviour):

    def __init__(self): 
        print("Initializing check for localization behavior")
        amcl_estimate = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)

        super(checklocalization, self).__init__("Check if localized")
         
    def update(self):
        global LOCALIZED

        if LOCALIZED:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def amcl_estimate_cb(self, msg):
        global LOCALIZED
        if np.linalg.norm(msg.pose.covariance) > 0.032:
            self.reached_position = False
            LOCALIZED = False

#####################################################################################################
#                                      localization Class                                           #
#####################################################################################################
class localization(pt.behaviour.Behaviour):
    def __init__(self):
        print('Initializing localization Behavior')

        # Parameters
        global_loc_srv = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        cmd_vel_topic = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        amcl_estimate = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Services
        self.global_loc_srv_client = rospy.ServiceProxy(global_loc_srv, Empty)
        rospy.wait_for_service(global_loc_srv, timeout = 30)

        # Pubs & Subs Init
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
        rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)

        rospy.sleep(3)

        # execution checker
        self.tried = False
        self.done = False

        self.counter = 0
        
        super(localization, self).__init__("Localize Robot!")

    def amcl_estimate_cb(self, msg):
        self.amcl_estimation = msg
    
    def update(self):

        global LOCALIZED, TRIED_LOCALIZATION, TRIED_MOVE
        rate = rospy.Rate(10) 

        if LOCALIZED:
            print('Tiago has been localized')
            return pt.common.Status.SUCCESS

        if not TRIED_LOCALIZATION:
            TRIED_LOCALIZATION = True
            self.global_loc_srv_client()

        rate.sleep()
        vel = Twist()

        if self.counter > 200:
            LOCALIZED = False
            print(Fore.RED + 'Timeout: Failed to localize')
            print(Fore.YELLOW + 'WARN: The particle filter failed to converge.')
            print(Fore.YELLOW + 'Clear costmaps and retry localization')
            self.counter = 0
            TRIED_LOCALIZATION = False
            return pt.common.Status.FAILURE

        elif np.linalg.norm(self.amcl_estimation.pose.covariance) > 0.02:
            vel.angular.z = 0.5
            self.cmd_vel_pub.publish(vel)
            self.counter += 1
            return pt.common.Status.RUNNING
        else:
            vel.angular.z = 0
            self.cmd_vel_pub.publish(vel)
            LOCALIZED = True
            TRIED_LOCALIZATION = False
            self.counter = 0
            print("Successfully localized")

            return pt.common.Status.SUCCESS
      
#####################################################################################################
#                                      clearcostmaps Class                                          #
#####################################################################################################
class clearcostmaps(pt.behaviour.Behaviour):
    def __init__(self):
        print('Initializing clear costmaps behavior')
        # Parameters
        clear_costmaps_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        # Services
        self.clear_costmaps_srv_client = rospy.ServiceProxy(clear_costmaps_srv, Empty)
        rospy.wait_for_service(clear_costmaps_srv, timeout = 30)

        super(clearcostmaps, self).__init__('Clear costmaps')

        # execution checker
        self.tried = False
        self.done = False
    
    def update(self):
        self.clear_costmaps_srv_client()
        print(Fore.WHITE + 'Cleared costmaps successfully!')
        rospy.sleep(1)
        return pt.common.Status.SUCCESS

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
            return pt.common.Status.SUCCESS
        else:
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
            DETECTPLACEDCUBE = False
            return pt.common.Status.FAILURE
        else:
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
        
        global CUBESTATE, PICKED, TRIED_MOVE, TRIED_PICK, TRIED_PLACE, TUCK_ARM, CANCELMOVE, MOVEBASESTATUS
        if TRIED_PICK:
            self.move_head_req = self.set_cube_state_srv(self.model_state)
            PICKED = False
            TRIED_MOVE =False
            TRIED_PICK = False
            TRIED_PLACE = False
            TUCK_ARM = False
            CANCELMOVE = True
            MOVEBASESTATUS = 0

        return pt.common.Status.SUCCESS

#####################################################################################################
#                                     checkpickcube Class                                           #
#####################################################################################################
class checkpickcube(pt.behaviour.Behaviour):
    def __init__(self):
        super(checkpickcube, self).__init__('Check if cube has been picked')

    def update(self):
        global PICKED, MOVEBASESTATUS, TRIED_PICK
        
        if PICKED:
            return pt.common.Status.SUCCESS
        else:
            if TRIED_PICK:
                MOVEBASESTATUS = 0
            return pt.common.Status.FAILURE

#####################################################################################################
#                                         pickcube Class                                            #
#####################################################################################################
class pickcube(pt.behaviour.Behaviour):

    # def __init__(self, cube_pose_): # Use this for E grade
    def __init__(self):
        print("Initialising pick cube behaviour.")
        
        # Server
        my_pick_cube_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_cube_srv = rospy.ServiceProxy(my_pick_cube_srv_nm, SetBool)
        rospy.wait_for_service(my_pick_cube_srv_nm, timeout=30)

        # execution checker
        self.tried = False

        # become a behaviour
        super(pickcube, self).__init__("Pick Cube!")
    

    def update(self):
        global PICKED, TRIED_PICK

        if not self.tried:
            # Command
            self.pick_cube_req = self.pick_cube_srv()
            self.tried = True
            TRIED_PICK = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_cube_req.success:
            self.tried = False
            PICKED = True
            print(Fore.WHITE + 'Successfully picked up the cube')
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_cube_req.success:
            print(Fore.RED + 'Failed to pick up the cube')
            print(Fore.YELLOW + 'Tuck arm and try again to pick up the cube')
            
            return pt.common.Status.FAILURE

        # if still trying
        else:
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
        super(checkdetectedplacecube, self).__init__('Check...what???')

    def update(self):
        global DETECTPLACEDCUBE
        
        if DETECTPLACEDCUBE:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

#####################################################################################################
#                                   detectplacedcube Class                                          #
#####################################################################################################
class detectplacedcube(pt.behaviour.Behaviour):
    def __init__(self, task):
        self.task = task
        rospy.Subscriber('/robotics_intro/aruco_single/position', Vector3Stamped, self.detect_cube_cb)

        super(detectplacedcube, self).__init__('Detect placed cube')

    def update(self):
        global PLACED, PICKED, DETECTED, CUBEPLACED, CUBESTATE, DETECTPLACEDCUBE

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
            return pt.common.Status.FAILURE

    def detect_cube_cb(self, msg):
        pass
            
#####################################################################################################
#                                     checkplacecube Class                                          #
#####################################################################################################
class checkplacecube(pt.behaviour.Behaviour):
    def __init__(self):
        super(checkplacecube, self).__init__('Check if cube has been placed')

    def update(self):
        
        global PLACED, TRIED_MOVE, MOVEBASESTATUS, TRIED_PLACE
        if PLACED:
            return pt.common.Status.SUCCESS
        else:
            if TRIED_PLACE:
                MOVEBASESTATUS = 0
            return pt.common.Status.FAILURE

#####################################################################################################
#                                        placecube Class                                            #
#####################################################################################################
class placecube(pt.behaviour.Behaviour):

    def __init__(self):
        print("Initialising place cube behaviour.")
        
        # Server
        my_place_cube_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_cube_srv = rospy.ServiceProxy(my_place_cube_srv_nm, SetBool)
        rospy.wait_for_service(my_place_cube_srv_nm, timeout=30)

        rospy.sleep(3)
        # Execution checker
        self.tried = False
        self.done = False
        # Become a behaviour
        super(placecube, self).__init__("Place Cube!")

    def update(self):
        global PLACED, TRIED_PLACE

        # try if not tried
        if not self.tried:

            # command
            self.place_cube_req = self.place_cube_srv()
            self.tried = True
            TRIED_PLACE = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_cube_req.success:
            self.tried = False
            print(Fore.WHITE + 'Successfully placed cube')
            PLACED = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_cube_req.success:
            print(Fore.RED + 'Failed to place cube')
            PLACED = False
            return pt.common.Status.FAILURE

        # if still trying
        else:
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
            return pt.common.Status.SUCCESS
            
        else:
            CUBESTATE = False
            return pt.common.Status.FAILURE

#####################################################################################################
#                                           movehead Class                                          #
#####################################################################################################
class movehead(pt.behaviour.Behaviour):

    def __init__(self, direction):
        global HEAD

        # Server
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
        if self.direction == 'up':
            if HEADUP:
                return pt.common.Status.SUCCESS
            else:
                move_head_srv_req = self.move_head_srv('up')

                if move_head_srv_req:
                    HEADUP = True
                    HEADDOWN = False
                    return pt.common.Status.SUCCESS

                return pt.common.Status.RUNNING
        elif self.direction == 'down':
            if HEADDOWN:
                return pt.common.Status.SUCCESS
            else:
                move_head_srv_req = self.move_head_srv('down')

                if move_head_srv_req:
                    HEADDOWN = True
                    HEADUP = False
                    return pt.common.Status.SUCCESS
                return pt.common.Status.RUNNING            
        else:
            print(Fore.RED + 'Invalid heaed motion was given. The motion \'{}\' was given whereas the valid inputs are \'up\' annd \'down\' ', format({self.direction}))
            return pt.common.Status.FAILURE

#####################################################################################################
#                                           move Class                                              #
#####################################################################################################
class move(pt.behaviour.Behaviour):
    def __init__(self, goal_pose):      
        print('Initializing move process')

        self.goal_pose = goal_pose
        self.reached_position = False
        self.in_progress = False

        # Parameters
        pick_pose_topic = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        place_pose_topic = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        amcl_estimate = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Topics (Pubs & Subs)
        rospy.Subscriber(pick_pose_topic, PoseStamped, self.pick_pose_cb)
        rospy.Subscriber(place_pose_topic, PoseStamped, self.place_pose_cb)
        rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)


        # Action Client
        self.move_client = SimpleActionClient('/move_base', MoveBaseAction)
        if not self.move_client.wait_for_server(rospy.Duration(20)):
            print('Failed to connect with /move_base action server!')
            exit()
        
        super(move, self).__init__('Move robot')


    def update(self):
        global MOVEBASESTATUS, TRIED_MOVE, CANCELMOVE
        self.move_goal = MoveBaseGoal()

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
            print(Fore.WHITE + 'Tiago is in {} pose' .format(self.goal_pose))
            return pt.common.Status.SUCCESS

        # If it hasn't already tried moving then move (send goal)
        if not TRIED_MOVE:
            TRIED_MOVE = True
            print(Fore.WHITE + 'Sending move to {} pose goal' .format(self.goal_pose))
            try:
                self.move_client.cancel_all_goals()
            except:
                pass
            self.move_client.send_goal(goal = self.move_goal, active_cb = self.move_active_cb, feedback_cb = self.move_fb_cb, done_cb=self.move_res_cb)
           
        if MOVEBASESTATUS == 0:
            return pt.common.Status.RUNNING
        elif MOVEBASESTATUS == 1:
            MOVEBASESTATUS = 0
            return pt.common.Status.SUCCESS
        else:
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
            print(Fore.WHITE + 'Successfully moved to {} pose' .format(self.goal_pose))
            MOVEBASESTATUS = 1
        else:
            print(Fore.RED + 'Failed to move to {} pose' .format(self.goal_pose))
            MOVEBASESTATUS = 2

    # Subscriber callbacks
    def pick_pose_cb(self, msg):
        self.pick_pose = msg

    def place_pose_cb(self, msg):
        self.place_pose = msg

    def amcl_estimate_cb(self, msg):
        global LOCALIZED
        if np.linalg.norm(msg.pose.covariance) > 0.032:
            self.reached_position = False
            # Cancel action goals & clear costmap
            self.move_client.cancel_all_goals()
            LOCALIZED = False




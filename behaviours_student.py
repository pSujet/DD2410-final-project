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
from control_msgs.msg import JointTrajectoryControllerState
from colorama import Fore, Back, Style

# Check Localization
LOCALIZED = False; TRIED_LOCALIZATION = False
PICKED = False; PLACED = False; CUBE = True
HEAD = 'up' # It is by default up
HEADUP = False; HEADDOWN = False

GRIPPER_THRESHOLD = 0.013

STATUS = 0

# Check tuck arm??



#####################################################################################################
#                                      checkgripper Class                                          #
#####################################################################################################
class checkgripper(pt.behaviour.Behaviour):
    def __init__(self):
        # rospy.loginfo('CHECK GRIPPER INIT')
        rospy.Subscriber('/gripper_controller/state', JointTrajectoryControllerState, self.check_gripper_cb)
        super(checkgripper, self).__init__('Check Gripper')

    def update(self):
        global GRIPPER_THRESHOLD, CUBE, PICKED
        rospy.loginfo(Fore.GREEN + 'CHECK GRIPPER UPDATE')
        if abs(self.gripper_state.actual.positions[0]) < GRIPPER_THRESHOLD and abs(self.gripper_state.actual.positions[1]) < GRIPPER_THRESHOLD :
            rospy.loginfo(Fore.GREEN + 'CHECK GRIPPER = Fail')
            CUBE = False
            #PICKED = False
            return pt.common.Status.FAILURE
        else:
            # CUBE = True
            # PICKED = True
            rospy.loginfo(Fore.GREEN + 'CHECK GRIPPER = Success')
            return pt.common.Status.SUCCESS


    def check_gripper_cb(self, msg):
        self.gripper_state = msg

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
        global CUBE, PICKED
        if CUBE == False:
            PICKED = False
            self.move_head_req = self.set_cube_state_srv(self.model_state)
        CUBE = True
        rospy.loginfo(Fore.GREEN + 'REINITCUBE = Success')
        return pt.common.Status.SUCCESS

#####################################################################################################
#                                           movehead Class                                          #
#####################################################################################################
class moveheadup(pt.behaviour.Behaviour):
    def __init__(self):
        #rospy.loginfo('MOVE HEAD UP INIT')
        mv_head_up_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_up_srv = rospy.ServiceProxy(mv_head_up_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_up_srv_nm, timeout=  30)

        super(moveheadup, self).__init__('move head up')

    def update(self):
        global HEADUP, HEADDOWN
        
        if HEADUP:
            rospy.loginfo(Fore.GREEN + 'MOVE HEAD UP UPDATE = Success')
            return pt.common.Status.SUCCESS
        else:
            move_head_up_srv_req = self.move_head_up_srv('up')

            if move_head_up_srv_req:
                HEADUP = True
                HEADDOWN = False
                rospy.loginfo(Fore.GREEN + 'MOVE HEAD UP UPDATE = Success')
                return pt.common.Status.SUCCESS
            rospy.loginfo(Fore.GREEN + 'MOVE HEAD UP UPDATE = Running')
            return pt.common.Status.RUNNING 

class moveheaddown(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo('MOVE HEAD DOWN INIT')
        mv_head_down_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_down_srv = rospy.ServiceProxy(mv_head_down_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_down_srv_nm, timeout=  30)

        super(moveheaddown, self).__init__('move head down')

    def update(self):
        global HEADUP, HEADDOWN
        
        if HEADDOWN:
            rospy.loginfo(Fore.GREEN + 'MOVE HEAD DOWN = Success')
            return pt.common.Status.SUCCESS
        else:
            move_head_down_srv_req = self.move_head_down_srv('down')

            if move_head_down_srv_req:
                HEADUP = False
                HEADDOWN = True
                rospy.loginfo(Fore.GREEN + 'MOVE HEAD DOWN = Success')
                return pt.common.Status.SUCCESS
            rospy.loginfo(Fore.GREEN + 'MOVE HEAD DOWN = Running')
            return pt.common.Status.RUNNING 

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
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        # command to tuck arm if haven't already
        # elif not self.sent_goal:

        # send the goal
        elif not self.sent_goal:
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True
            rospy.loginfo(Fore.GREEN + 'Tuck arm = Running')
            return pt.common.Status.RUNNING

        if self.play_motion_ac.get_result():
            # rospy.sleep(10)
            self.finished = True
            rospy.loginfo(Fore.GREEN + 'Tuck arm = Success')
            return pt.common.Status.SUCCESS


        # # if I was succesful! :)))))))))
        # elif self.play_motion_ac.get_result():

        #     # than I'm finished!
        #     self.finished = True
        #     rospy.loginfo('Tuck arm = Success')
        #     return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            rospy.loginfo(Fore.GREEN + 'Tuck arm = Fail')
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            rospy.loginfo(Fore.GREEN + 'Tuck arm = Running')
            return pt.common.Status.RUNNING
        # # already tucked the arm
        # if self.finished: 
        #     return pt.common.Status.SUCCESS
        
        # # command to tuck arm if haven't already
        # elif not self.sent_goal:

        #     # send the goal
        #     self.play_motion_ac.send_goal(self.goal)
        #     self.sent_goal = True

        #     # tell the tree you're running
        #     return pt.common.Status.RUNNING

        # # if I was succesful! :)))))))))
        # elif self.play_motion_ac.get_result():

        #     # than I'm finished!
        #     self.finished = True
        #     return pt.common.Status.SUCCESS

        # # if failed
        # elif not self.play_motion_ac.get_result():
        #     return pt.common.Status.FAILURE

        # # if I'm still trying :|
        # else:
        #     return pt.common.Status.RUNNING

#####################################################################################################
#                                    checklocalization Class                                        #
#####################################################################################################
class checklocalization(pt.behaviour.Behaviour):
    def __init__(self): 
        # rospy.loginfo('CHECK LOCALIZATION INIT')
        rospy.loginfo("Initializing check for localization")
        amcl_estimate = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        rospy.Subscriber(amcl_estimate, PoseWithCovarianceStamped, self.amcl_estimate_cb)
        print("1111111111111111111111111111111111111111111111111")

        super(checklocalization, self).__init__("Check if localized")
         
    def update(self):
        print("2222222222222222222222222222222222222222222222222")
        #rospy.loginfo('CHECK LOCALIZATION UPDATE')
        global LOCALIZED

        if LOCALIZED:
            rospy.loginfo(Fore.GREEN + 'CHECK LOCALIZATION UPDATE = Success')
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo(Fore.GREEN + 'CHECK LOCALIZATION UPDATE = Fail')
            return pt.common.Status.FAILURE 
    def amcl_estimate_cb(self, msg):
        global LOCALIZED
        if np.linalg.norm(msg.pose.covariance) > 0.04:
            self.reached_position = False
            LOCALIZED = False

#####################################################################################################
#                                      localization Class                                           #
#####################################################################################################
class localization(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo('LOCALIZATION INIT')
        rospy.loginfo('Initializing localization')

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

        if LOCALIZED:
            #TRIED_LOCALIZATION = False
            return pt.common.Status.SUCCESS

        if not TRIED_LOCALIZATION:
            TRIED_LOCALIZATION = True
            self.global_loc_srv_client()

        rate = rospy.Rate(10) 
        rate.sleep()        ################################
        vel = Twist()

        if np.linalg.norm(self.amcl_estimation.pose.covariance) > 0.025:
            vel.angular.z = 0.5
            self.cmd_vel_pub.publish(vel)
            rospy.loginfo(Fore.GREEN + 'LOCALIZATION UPDATE = Running')
            return pt.common.Status.RUNNING
        else:
            vel.angular.z = 0
            self.cmd_vel_pub.publish(vel)
            LOCALIZED = True
            TRIED_LOCALIZATION = False
            rospy.loginfo(Fore.GREEN + 'LOCALIZATION UPDATE = Success')
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
#                                     checkpickcube Class                                           #
#####################################################################################################
class checkpickcube(pt.behaviour.Behaviour):
    def __init__(self):
        # rospy.loginfo('CHECK PICK CUBE INIT')
        super(checkpickcube, self).__init__('Check if cube has been picked')

    def update(self):
        
        global PICKED
        
        if PICKED:
            rospy.loginfo(Fore.GREEN + 'CHECK PICK CUBE = Success')
            return pt.common.Status.SUCCESS
            
        else:
            rospy.loginfo(Fore.GREEN + 'CHECK PICK CUBE = Fail')
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

        rospy.sleep(3)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(pickcube, self).__init__("Pick Cube!")
    

    def update(self):
        global PICKED

        # success if done
        if self.done:
            PICKED = True
            rospy.loginfo(Fore.GREEN + 'PICK CUBE = Success')
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.pick_cube_req = self.pick_cube_srv()
            self.tried = True

            # tell the tree you're running
            rospy.loginfo(Fore.GREEN + 'PICK CUBE = Running')
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_cube_req.success:
            self.done = True
            PICKED = True
            rospy.loginfo(Fore.GREEN + 'PICK CUBE = Success')
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_cube_req.success:
            rospy.loginfo(Fore.GREEN + 'PICK CUBE = Fail')
            return pt.common.Status.FAILURE

        # if still trying
        else:
            rospy.loginfo(Fore.GREEN + 'PICK CUBE = Running')
            return pt.common.Status.RUNNING

#####################################################################################################
#                                     checkplacecube Class                                          #
#####################################################################################################
class checkplacecube(pt.behaviour.Behaviour):
    def __init__(self):
        # rospy.loginfo('CHECK PLACE CUBE INIT')
        super(checkplacecube, self).__init__('Check if cube has been placed')

    def update(self):
        
        global PLACED
        if PLACED:
            rospy.loginfo(Fore.GREEN + 'CHECK PLACE CUBE = Success')
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo(Fore.GREEN + 'CHECK PLACE CUBE = Fail')
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
        # success if done
        if self.done:
            rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Success')
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.place_cube_req = self.place_cube_srv()
            self.tried = True

            # tell the tree you're running
            rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Running')
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_cube_req.success:
            self.done = True
            rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Success')
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_cube_req.success:
            rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Fail')
            return pt.common.Status.FAILURE

        # if still trying
        else:
            rospy.loginfo(Fore.GREEN + 'PLACE CUBE = Running')
            return pt.common.Status.RUNNING

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

        #self.status = {"STATUS":"PENDING"}
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
        global STATUS
        move_goal = MoveBaseGoal()

        if self.goal_pose == 'pick':
            move_goal.target_pose = self.pick_pose
        if self.goal_pose == 'place':
            move_goal.target_pose = self.place_pose

        self.move_client.send_goal(goal = move_goal, active_cb = self.move_active_cb, feedback_cb = self.move_fb_cb, done_cb=self.move_res_cb)
        self.move_client.wait_for_result()
        rospy.loginfo("============================0")

        # status = self.move_client.wait_for_result()
        # print('44444444444444444444444444444444444444444444444444444444444444')
        # print('status = {}' .format(status))
        # print('type of status = {}' .format(type(status)))
        # self.reached_position = True

        # if not self.reached_position:
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Fail')
        #     return pt.common.Status.FAILURE
        # if status:
        #     self.reached_position = True
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Success')
        #     return pt.common.Status.SUCCESS

        # if status:
        #     self.reached_position = True
        #     return pt.common.Status.SUCCESS
        # if not self.reached_position:
        #     return pt.common.Status.FAILURE
        
        # if self.status['STATUS'] == "RUNNING":
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Running')
        #     return pt.common.Status.RUNNING
        # elif self.status['STATUS'] == "SUCCESS":
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Success')  
        #     return pt.common.Status.SUCCESS
        # else:
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Fail')
        #     return pt.common.Status.FAILURE

        # if self.status == 0:
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Running')
        #     return pt.common.Status.RUNNING
        # elif self.status == 1:
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Success')  
        #     return pt.common.Status.SUCCESS
        # else:
        #     rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Fail')
        #     return pt.common.Status.FAILURE

        if STATUS == 0:
            rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Running')
            
            return pt.common.Status.RUNNING
        elif STATUS == 1:
            rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Success')  
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo(Fore.GREEN + 'MOVE UPDATE = Fail')
            return pt.common.Status.FAILURE

    def move_active_cb(self):
        # self.move_client.wait_for_result()
        rospy.loginfo('LETS SEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!')

    # Action callbacks
    def move_fb_cb(self, msg):
        global STATUS
        #status = self.move_client.wait_for_result()
        # rospy.loginfo('move_fb_cb function!!')
        # self.status = 0
        STATUS = 0
        return pt.common.Status.RUNNING
        

    def move_res_cb(self, state, msg):
        global STATUS
        rospy.loginfo('move_res_cb function!!')
        if actionlib.TerminalState.SUCCEEDED == state:
            # self.status = 1
            STATUS = 1
        else:
            # self.status = 2
            STATUS = 2
            

    # Subscriber callbacks
    def pick_pose_cb(self, msg):
        self.pick_pose = msg

    def place_pose_cb(self, msg):
        self.place_pose = msg

    def amcl_estimate_cb(self, msg):
        global LOCALIZED
        if np.linalg.norm(msg.pose.covariance) > 0.04:
            self.reached_position = False
            # LOCALIZED = False
            # Cancel action goals & clear costmap
            self.move_client.cancel_all_goals()
            # self.clear_costmaps_srv_client()
            LOCALIZED = False






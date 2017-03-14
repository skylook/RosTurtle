'''
game_known_markers


14/03/2017
Johan Osinga
'''

from detect_markers import DetectMarkers
import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point

#Known markers in the map (0 - 11)
#MARKERS_IN_ARENA = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
MARKERS_IN_ARENA = [0, 1, 9]
#MARKER_SORT = [0, 2, 3, 4, 5, 6, 7, 8, 9, 1, 11]
MARKER_SORT = MARKERS_IN_ARENA

class GamePlan(object):
    '''
    GamePlan classmethod

    Executes the gameplan for the Adaptive Robotics ROS challenge
    This gameplan assumes the (wall)marker positions are already known and saved
    '''

    def __init__(self, debug_logging=False):
        #Constructor

        #if debug logging is True then perform prints to console
        self.debug = debug_logging

        #log
        self._console_log("GamePlan started")

        #vars
        self._detect_markers = DetectMarkers(False)
        self._markers_reached = []
        self._marker_pos_time = 5.0

    def play_game(self):
        #Play the game!

        #Iterate over the known markers in the defined order
        for marker_id in MARKER_SORT:
            self._console_log("ar_marker_" + str(marker_id))

            self.move_to_marker(marker_id)

            #Sleep at the marker pos
            rospy.sleep(self._marker_pos_time)

        #Succes?!
        return True

    def move_to_marker(self, marker_id):
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")
        
        #Get marker information
        marker_name = "ar_marker_" + str(marker_id)
        marker = self._detect_markers.get_markers()[marker_name]
        goal_x = marker['transform'].translation.x
        goal_y = marker['transform'].translation.y
        if marker['derived']:
            goal_z = marker['yaw']

        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(goal_x,goal_y,0)

        #Transform orientation
        if marker['derived']:
            quaterinion = tf.transformations.quaternion_from_euler(0, 0, goal_z)
            goal.target_pose.pose.orientation.x = quaterinion[0]
            goal.target_pose.pose.orientation.y = quaterinion[1]
            goal.target_pose.pose.orientation.z = quaterinion[2]
            goal.target_pose.pose.orientation.w = quaterinion[3]
        else:
            goal.target_pose.pose.orientation = marker['transform'].rotation

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            self._console_log("You have reached the destination")
            self._markers_reached.append(marker_name)
            return True

        else:
            self._console_log("The robot failed to reach the destination")
            return False
    
    def _console_log(self, message):
        if self.debug:
            #rospy.loginfo(message)
            print(message)


#Run if file is executed
if __name__ == '__main__':
    #Create ROS Node
    ros_node_name = "einstein_gameplan"
    rospy.init_node(ros_node_name)
    print("Node started: " + ros_node_name)

    game_plan = GamePlan(False)

    input_var = raw_input("Enter r to run, anything else to cancel")
    if input_var == "r":
        #Run game
        game_plan.play_game()
    

    rospy.spin()

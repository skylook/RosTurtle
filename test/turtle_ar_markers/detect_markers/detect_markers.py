'''
detect_markers


09/03/2017
Johan Osinga
'''
import ar_track_alvar_msgs.msg
import rospy
import tf2_ros
import geometry_msgs.msg

class DetectMarkers(object):
    '''
    DetectMarkers Class

    Listens to the ar_pose_marker topic of the ar_track_alvar node
    Broadcasts the pose of every detected marker to the tf2 to be transformed to the map

    Listens to the tf2 and saves the transformed locations of the markers
    '''
    def __init__(self, debug_logging=False):
        #Constructor

        #If debug is True then perform prints to console
        self.debug = debug_logging

        #log
        self._console_log("MarkerBroadcast started")


        #vars
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_list = {}




        #Subscribe to ar_track_alvar output topic
        rospy.Subscriber("ar_pose_marker",
                         ar_track_alvar_msgs.msg.AlvarMarkers,
                         self._ar_marker_callback)
    
    def get_markers_amount(self):
        #Return amount of known markers
        return len(self.marker_list)

    def get_markers(self):
        #Return list of known markers
        return self.marker_list


    def _ar_marker_callback(self, msg):
        #Check amount of markers detected
        msg_markers = msg.markers

        #log
        self._console_log(str(len(msg_markers)) + "markers detected")

        if len(msg_markers) > 0:
            #Iterate over every marker
            for marker in msg_markers:
                self._console_log(marker)
                markername = "ar_marker_" + str(marker.id)
                markerpose = marker.pose
                markerpose.header = marker.header

                #log
                self._console_log(markername + "detected")

                #Broadcast to tf2
                self._broadcast_to_tf2(markerpose, markername)

                #Save known markers
                if markername not in self.marker_list:
                    self.marker_list[markername] = None
                    self._console_log("Added " + markername + " to marker_list")

        #Listen to tf2 for transformed markers        
        self._listen_to_tf2()

    def _listen_to_tf2(self):
        self._console_log("Listening to tf2 for transformed markers")

        #Try to find known markers in tf2
        for markername in self.marker_list:
            try:
                #Request trans
                trans = self.tf_buffer.lookup_transform('map', markername, rospy.Time())

                self._console_log("Trans for " + markername + "found")

                #Differentiate between floor and wall markers
                markertype = 'floor'
                if len(markername[10:]) > 2:
                    markertype = 'wall'

                #Save trans information
                self.marker_list[markername] = {'name':markername, 
                                                'time':rospy.Time(), 
                                                'transform':trans.transform,
                                                'type':markertype}
            except Exception as e:
                self._console_log("Error: " + str(e))
                continue

        file = open("markers.txt", "w")
        file.write(str(self.marker_list))



    def _broadcast_to_tf2(self, msg, markername):
        msg = msg.pose
        #Broadcasts a pose to the tf2
        trans_broadcaster = tf2_ros.TransformBroadcaster()
        trans_stamped = geometry_msgs.msg.TransformStamped()

        trans_stamped.header.stamp = rospy.Time.now()
        trans_stamped.header.frame_id = "map"
        trans_stamped.child_frame_id = markername
        trans_stamped.transform.translation.x = msg.position.x
        trans_stamped.transform.translation.y = msg.position.y
        trans_stamped.transform.translation.z = 0.0
        trans_stamped.transform.rotation.x = msg.orientation.x
        trans_stamped.transform.rotation.y = msg.orientation.y
        trans_stamped.transform.rotation.z = msg.orientation.z
        trans_stamped.transform.rotation.w = msg.orientation.w

        trans_broadcaster.sendTransform(trans_stamped)

        #log
        self._console_log(markername + "broadcasted to tf2")

    def _console_log(self, message):
        if self.debug:
            print(message)

#Do if python file is run and not included
if __name__ == '__main__':
    #Create ROS Node
    ros_node_name = "ar_detect_markers"
    rospy.init_node(ros_node_name)
    print("Node started: " + ros_node_name)

    detect_markers = DetectMarkers(False)
    #while not rospy.is_shutdown():
        #print(detect_markers.get_markers())
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

'''
detect_markers


09/03/2017
Johan Osinga
'''
import pickle
import ar_track_alvar_msgs.msg
import rospy
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

class DetectMarkers(object):
    '''
    DetectMarkers Class

    Listens to the ar_pose_marker topic of the ar_track_alvar node
    Broadcasts the pose of every detected marker to the tf2 to be transformed to the map

    Listens to the tf2 and saves the transformed locations of the markers
    '''
    def __init__(self, marker_file_location="", marker_file_name="markers", scan_for_markers=True, debug_logging=False):
        #Constructor

        #If debug is True then perform prints to console
        self.debug = debug_logging

        #log
        self._console_log("MarkerBroadcast started")


        #vars
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.marker_list = {}
        self.marker_file_location = marker_file_location

        #If scanning for markers is set, then scan for markers
        #If not set then then this class is only for loading and supplying known markers
        if scan_for_markers:
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

    def save_to_file(self, name):
        with open(self.marker_file_location + name + '.pkl', 'wb') as f:
            pickle.dump(self.marker_list, f, pickle.HIGHEST_PROTOCOL)

    def load_from_file(self, name):
        with open(self.marker_file_location + name + '.pkl', 'rb') as f:
            self.marker_list = pickle.load(f)

    def _ar_marker_callback(self, msg):
        #Check amount of markers detected
        msg_markers = msg.markers

        #log
        self._console_log(str(len(msg_markers)) + "markers detected")

        if len(msg_markers) > 0:
            #Iterate over every marker
            for marker in msg_markers:
                self._console_log(marker)
                marker_name = "ar_marker_" + str(marker.id)
                marker_pose = marker.pose
                marker_pose.header = marker.header

                #log
                self._console_log(marker_name + "detected")

                #Broadcast to tf2
                self._broadcast_to_tf2(marker_pose, marker_name)


                #If the marker is a wall marker, check if the floor marker is known
                if marker.id >= 100:
                    #Wall marker
                    update_derived_marker = True
                    derived_marker_name = "ar_marker_" + str(marker.id - 100)
                    if derived_marker_name in self.marker_list:
                        #Check if floor marker is already known
                        if not self.marker_list[derived_marker_name]['derived']:
                            #Marker is derived, update
                            update_derived_marker = False

                    if update_derived_marker:
                        #Update marker
                        #Get Z-axis of wall marker
                        derived_marker_z = self._marker_euler_z(marker_pose.pose.orientation.x,
                                                                marker_pose.pose.orientation.y,
                                                                marker_pose.pose.orientation.z,
                                                                marker_pose.pose.orientation.w)

                        #Create 'carrot'
                        t = geometry_msgs.msg.TransformStamped()
                        t.header.frame_id = marker_name
                        #t.header.stamp = rospy.Time.now()
                        t.header.stamp = marker_pose.header.stamp
                        t.child_frame_id = derived_marker_name
                        t.transform.translation.x = 0.0
                        t.transform.translation.y = 0.0
                        t.transform.translation.z = 1.0

                        t.transform.rotation.x = 0.0
                        t.transform.rotation.y = 0.0
                        t.transform.rotation.z = 0.0
                        t.transform.rotation.w = 1.0

                        tfm = tf2_msgs.msg.TFMessage([t])
                        self.pub_tf.publish(tfm)

                        #Save derived marker
                        self.marker_list[derived_marker_name] = {}
                        self.marker_list[derived_marker_name]['derived'] = True
                        self.marker_list[derived_marker_name]['yaw'] = derived_marker_z

                #Save known markers
                if marker_name not in self.marker_list:
                    self.marker_list[marker_name] = {}
                    self._console_log("Added " + marker_name + " to marker_list")
                #Markers is not derived
                self.marker_list[marker_name]['derived'] = False

        #Listen to tf2 for transformed markers
        self._listen_to_tf2()

    def _listen_to_tf2(self):
        self._console_log("Listening to tf2 for transformed markers")

        #Try to find known markers in tf2
        for marker_name in self.marker_list:
            try:
                #Request trans

                lookup_time = rospy.Time()# - rospy.Duration(5.0)
                trans = self.tf_buffer.lookup_transform('map', marker_name, lookup_time)

                self._console_log("Trans for " + marker_name + " found")

                #Differentiate between floor and wall markers
                markertype = 'floor'
                if len(marker_name[10:]) > 2:
                    markertype = 'wall'

                #Set marker information
                self.marker_list[marker_name]['name'] = marker_name
                self.marker_list[marker_name]['time'] = rospy.Time()
                self.marker_list[marker_name]['transform'] = trans.transform
                self.marker_list[marker_name]['type'] = markertype

            except Exception as e:
                self._console_log("Error: " + str(e))
                continue

        file = open("markers.txt", "w")
        file.write(str(self.marker_list))



    def _broadcast_to_tf2(self, msg, marker_name):
        msg = msg.pose
        #Broadcasts a pose to the tf2
        trans_broadcaster = tf2_ros.TransformBroadcaster()
        trans_stamped = geometry_msgs.msg.TransformStamped()

        trans_stamped.header.stamp = rospy.Time.now()
        trans_stamped.header.frame_id = "map"
        trans_stamped.child_frame_id = marker_name
        trans_stamped.transform.translation.x = msg.position.x
        trans_stamped.transform.translation.y = msg.position.y
        trans_stamped.transform.translation.z = 0.0
        trans_stamped.transform.rotation.x = msg.orientation.x
        trans_stamped.transform.rotation.y = msg.orientation.y
        trans_stamped.transform.rotation.z = msg.orientation.z
        trans_stamped.transform.rotation.w = msg.orientation.w

        trans_broadcaster.sendTransform(trans_stamped)

        #log
        self._console_log(marker_name + "broadcasted to tf2")

    def _console_log(self, message):
        if self.debug:
            print(message)

    def _marker_euler_z(self, x, y, z, w):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([x, y, z, w])
        return y

#Do if python file is run and not included
if __name__ == '__main__':
    #Create ROS Node
    ros_node_name = "ar_detect_markers"
    rospy.init_node(ros_node_name)
    print("Node started: " + ros_node_name)

    detect_markers = DetectMarkers(False)

    loop = True
    while(loop):
        input_var = raw_input("Enter s to save, o to open, d to display, q to quit")
        if input_var == "s":
            detect_markers.save_to_file("testfile")
        elif input_var == "o":
            detect_markers.load_from_file("testfile")
        elif input_var == "d":
            print(detect_markers.get_markers())
        elif input_var == "q":
            loop = False

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

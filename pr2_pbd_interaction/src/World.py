'''Everything related to perception of the world'''
import roslib
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import os
import time
import threading
from numpy.linalg import norm
from numpy import array
from random import uniform
from math import sqrt

# ROS libraries
#from actionlib_msgs.msg import
import rospy
import tf
from tf import TransformListener, TransformBroadcaster
from manipulation_msgs.msg import GraspableObjectList
#from object_manipulation_msgs.msg import GraspableObjectList
from object_manipulation_msgs.srv import FindClusterBoundingBox
from pr2_interactive_object_detection.msg import UserCommandAction
from pr2_interactive_object_detection.msg import UserCommandGoal
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from actionlib_msgs.msg import GoalStatus
import actionlib

# Local stuff
from pr2_pbd_interaction.msg import Object, ArmState
from pr2_social_gaze.msg import GazeGoal
from Response import Response


class WorldObject:
    '''Class for representing objects'''

    def __init__(self, pose, index, dimensions, is_recognized):
        ''' Initialization of objects'''
        self.index = index
        self.assigned_name = None
        self.is_recognized = is_recognized
        self.object = Object(Object.TABLE_TOP, self.get_name(),
                             pose, dimensions)
        self.menu_handler = MenuHandler()
        self.int_marker = None
        self.is_removed = False
        self.menu_handler.insert('Remove from scene', callback=self.remove)

    def remove(self, dummy):
        '''Function for removing object from the world'''
        rospy.loginfo('Will remove object' + self.get_name())
        self.is_removed = True

    def assign_name(self, name):
        '''Function for assigning a different name'''
        self.assigned_name = name
        self.object.name = name

    def get_name(self):
        '''Function to get the object name'''
        if (self.assigned_name == None):
            if (self.is_recognized):
                return 'object' + str(self.index)
            else:
                return 'thing' + str(self.index)
        else:
            return self.assigned_name

    def decrease_index(self):
        '''Function to decrese object index'''
        self.index -= 1

class Legend:
    '''Key for showing what various Rviz GUI objects and colors represent, such
    as the different colors for gripper markers.'''

    def __init__(self):
        self.marker_pub = rospy.Publisher('visualization_marker', Marker)
        marker = Marker(type=Marker.SPHERE, id=42424242,
            #lifetime=rospy.Duration(1), # Assuming nothing makes infinite
            scale=Vector3(1,1,1),
            header=Header(frame_id='base_link'), # Seems like this will change
            color=ColorRGBA(1.0, 0.0, 0.0, 0.5),
            pose=Pose(Point(1,0,0), Quaternion(0,0,0,1)),
            text='Legend')
        # NOTE(max): Publishing once doesn't work, but publishing infinitely
        # or sleeping before publishing does. Strange.
        # self.marker_pub.publish(marker)
        rospy.loginfo("Published legend")

class World:
    '''Object recognition and localization related stuff'''

    tf_listener = None
    objects = []
    # Separator for reading / writing mocked objects to / from files.
    _sep = ','

    def __init__(self):

        if World.tf_listener == None:
            World.tf_listener = TransformListener()
        self._lock = threading.Lock()
        self.surface = None
        self._tf_broadcaster = TransformBroadcaster()
        self._im_server = InteractiveMarkerServer('world_objects')

        # NOTE(max): The following is for detecting "real" objects, so we're
        # disabling it for now while we mock them.
        # bb_service_name = 'find_cluster_bounding_box'
        # rospy.wait_for_service(bb_service_name)
        # self._bb_service = rospy.ServiceProxy(bb_service_name,
        #                                     FindClusterBoundingBox)
        # rospy.Subscriber('interactive_object_recognition_result',
        #     GraspableObjectList, self.receive_object_info)
        # self._object_action_client = actionlib.SimpleActionClient(
        #     'object_detection_user_command', UserCommandAction)
        # self._object_action_client.wait_for_server()
        # rospy.loginfo('Interactive object detection action ' +
        #               'server has responded.')

        # NOTE(max): I made this method not do anything, but am keeping the call
        # here as a reminder.
        self.clear_all_objects()

        # NOTE(max): We remove the table-getting subscription as well.
        # The following is to get the table information
        # rospy.Subscriber('tabletop_segmentation_markers',
        #                 Marker, self.receive_table_marker)

        # Mock objects and table
        # set this constant based on how many objects we are mocking
        self._max_mocked_action_idx = 30

        # this stores the current saved mocked objects so we don't re-mock
        # unnecessarily. start with an invalid one
        self._cur_action_idx = 0

    def _mock_objects_for_action(self, action_index):
        '''Add fake objects to the interaction / rviz. Should eventually do this
        smartly, i.e. w.r.t current test.'''
        # Clear current objects /table
        self._reset_objects()

        # Avoid races while swapping out underlying array
        self._lock.acquire()

        # Either: load the sampled objects...
        #self._load_mock_objects(action_index)

        # OR do sampling...
        self._sample_objects()

        # OR use hardcoded data.
        # if action_index == 1:
        #     position = Point(0.407702162213, 0.448435729551, 0.596100389957)
        #     orientation = Quaternion(0.0, 0.0, 0.621998629232, 0.783018330075)
        #     pose = Pose(position, orientation)
        #     dimensions = Vector3(0.0885568181956, 0.0448812849838,
        #         0.030891418457)
        #     self._add_new_object(pose, dimensions, False)
        # elif action_index == 2:
        #     position = Point(0.829287886892, -0.46793514682, 0.618727564812)
        #     orientation = Quaternion(0.0, 0.0, 0.0455530206745, 0.998961922351)
        #     pose = Pose(position, orientation)
        #     dimensions = Vector3(0.102971868856, 0.0529677164712,
        #         0.0323750972748)
        #     self._add_new_object(pose, dimensions, False)

        # Release
        self._lock.release()

        # Create the standard table
        self._mock_table()

    def _load_mock_objects(self, action_index):
        ''' Loads saved mocked objects for the given action index. Assumes they
        exist, so you won't get any if they don't!'''
        objects_dir = rospy.get_param('/pr2_pbd_interaction/dataRoot') + \
            '/data/objects/'
        obj_file = objects_dir + 'Action' + str(action_index) + '.txt'
        mocked_objs = World.read_mocked_worldobjs_fom_file(obj_file)
        for mocked_obj in mocked_objs:
            self._add_new_object_internal(mocked_obj)

    def _sample_objects(self):
        '''Generate sample positions of objects.'''

        # Position settings
        min_x = 0.40 # lowest observed x value is 0.404...
        max_x = 1.0 #0.81 # highest observed x value is 0.809...
        min_y = -0.48 # lowest observed y value is -0.47...
        max_y = 0.57 # highest observed y value is 0.56...

        # Z values will depend on the object ...
        #z = 0.638032227755 # fake-iron
        z = 0.607358753681 # red-plate
        #z = 0.666624039412 # brown-box
        #z = 0.642685890198 # white-box
        #z = 0.615162938833 # lava-moss

        # Dimension settings
        # fake-iron 
        #dimensions = Vector3(0.140946324722, 0.0966388155749, 0.0660033226013)
        # red-plate
        dimensions = Vector3(0.208253721282, 0.153458412609, 0.025651037693)
        # brown-box
        #dimensions = Vector3(0.250331583552, 0.250164705599, 0.148873627186)
        # white-box
        #dimensions = Vector3(0.21396259923, 0.0538839277603, 0.107205629349)
        # lava-moss
        #dimensions = Vector3(0.0967770918593, 0.0522750997274, 0.0276364684105)

        # Generation settings
        n_to_mock = 1
        filename = time.strftime('%y.%m.%d_%H.%M.%S') + '.txt'
        samples_dir = rospy.get_param('/pr2_pbd_interaction/dataRoot') + \
            '/data/samples/'
        if (not os.path.exists(samples_dir)):
            os.makedirs(samples_dir)
        data_filename = samples_dir + filename
        fh = open(data_filename, 'a') # append if it exists, for safety
        rospy.loginfo("Sampling " + str(n_to_mock) + " objects; saving into " +
            data_filename)

        # Actually do the generation
        # First clear the other objects and add the table back
        self._mock_table()
        while len(World.objects) < n_to_mock:
            # Sample for position
            x = uniform(min_x, max_x)
            y = uniform(min_y, max_y)
            # We don't want to sample z; object should be on table.
            position = Point(x,y,z)

            # Orientation we assume only 1 DOF, so qx == qy == 0.0
            qz = uniform(0, 1)
            qw = sqrt(1.0 - qz**2)
            orientation = Quaternion(0.0, 0.0, qz, qw)

            # Construct the candidate object
            pose = Pose(position, orientation)
            n_objects = len(World.objects)
            candidate = WorldObject(pose, n_objects, dimensions, False)

            # Ensure it's reachable. Currently just doing with either arm but
            # will likely have to change to a specific arm depending on what
            # the action is.
            if not World.is_object_within_reach(candidate):
                continue

            # Actually add it
            self._add_new_object_internal(candidate)
            rospy.loginfo('Generated ' + str(len(World.objects)) + '/' +
                str(n_to_mock) + ' objects.')

            # Save to file for bookkeeping
            World.write_mocked_worldobj_to_file(candidate, fh)

        # Cleanup
        fh.close()

    @staticmethod
    def get_planar_distance_from_arm(ref_object, arm_index):
        '''Gets how far an objet is away from the robot's arm'''
        if arm_index == 0:
            arm_name = 'r_upper_arm_roll_link'
        else:
            arm_name = 'l_upper_arm_roll_link'
        rel_obj_pose = World.transform(ref_object.object.pose, 'base_link',
            arm_name)
        #print rel_obj_pose
        return norm(array([rel_obj_pose.position.x, rel_obj_pose.position.y]))

    @staticmethod
    def is_object_within_reach(ref_object):
        '''Sees if the robot can reach the object.'''
        for arm_index in range(2):
            if not World.is_object_within_reach_of_arm(ref_object, arm_index):
                return False
        return True

    @staticmethod
    def is_object_within_reach_of_arm(ref_object, arm_index):
        '''Sees whether a robot's particular arm can reach the object. The magic
        number 0.881 is from the following code:

        # finger_name = 'l_gripper_l_finger_tip_frame'
        # arm_name = 'l_upper_arm_roll_link'
        # rel_pose = World.get_pose_in_ref_frame(finger_name, arm_name)
        # horizontal_ee_distance = norm(array([rel_pose.position.x,
            rel_pose.position.y]))

        So 0.881 is of course PR2-specific.
        '''
        allowed_distance = 0.881 + norm(array([ref_object.object.dimensions.x,
            ref_object.object.dimensions.y, ref_object.object.dimensions.z])) / 2
        planar_dist = World.get_planar_distance_from_arm(ref_object, arm_index)
        # print 'allowed_distance', allowed_distance
        # print 'planar_dist', planar_dist
        return planar_dist < allowed_distance

    @staticmethod
    def get_pose_in_ref_frame(frame_name, ref_frame):
        '''Used for computation of reachability constant (0.881). See comment in
        is_object_within_reach_of_arm() for how this was used.'''
        try:
            time = World.tf_listener.getLatestCommonTime(ref_frame,
                frame_name)
            position, orientation = World.tf_listener.lookupTransform(
                ref_frame, frame_name, time)
            frame_pose = Pose()
            frame_pose.position = Point(position[0], position[1], position[2])
            frame_pose.orientation = Quaternion(orientation[0], orientation[1],
               orientation[2], orientation[3])
            return frame_pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn('Something wrong with transform request.')
            return None

    @staticmethod
    def write_mocked_worldobj_to_file(worldObject, fileHandle):
        '''Writes a WorldObject's pose (position and orentation) and dimensions
        to the provided filehandle. Use with read_mockied_worldobjs_from_file()
        to save and restore objects. Writes as csv, so should be readable from
        text editors as well.

        Note: Doesn't open or close fileHandle, so you must manage that
        externally.'''
        # Settings
        sep = World._sep

        # extract
        p = worldObject.object.pose.position
        o = worldObject.object.pose.orientation
        d = worldObject.object.dimensions
        data = [p.x, p.y, p.z, o.x, o.y, o.z, o.w, d.x, d.y, d.z]
        data = [str(datum) for datum in data]
        line = sep.join(data)

        # write
        fileHandle.write(line + '\n')

    @staticmethod
    def read_mocked_worldobjs_fom_file(fileName):
        '''Read's the lines from the file specified by fileName and attempts to
        turn them into WorldObjects. Only restores the pose (position and
        orentation) and dimensions. Use with write_mocked_worldobj_to_file()
        to save and restore objects.

        Returns a list of WorldObjects.

        Note that this assumes these are the only objects in the scene, so it
        auto-numbers them this way (starting at 0).

        Note: Creates file handle from the provided fileName, so this DOES
        manage opening/closing of file for you.'''
        # Settings
        sep = World._sep

        objects = []
        lines = [line.strip() for line in open(fileName)]
        for line in lines:
            pieces = [float(piece) for piece in line.split(sep)]
            position = Point(pieces[0], pieces[1], pieces[2])
            orientation = Quaternion(pieces[3], pieces[4], pieces[5], pieces[6])
            pose = Pose(position, orientation)
            dimensions = Vector3(pieces[7], pieces[8], pieces[9])
            objects.append(WorldObject(pose, len(objects), dimensions, False))
        return objects

    def _mock_table(self):
        '''Testing for right now, but should add a fake object to the
        interaction / rviz.'''
        # This is mocked from the first data point collected
        position = Point(0.681273249847, 0.0500235402375, 0.555035150217)
        orientation = Quaternion(-0.000624796068025, -0.0230785640599,
            1.44232405509e-05, 0.999733459129)
        pose = Pose(position, orientation)
        dimensions = Vector3(0.635490983725, 1.12122958899, 0.01)

        # This is straight-up copy-pasted from receive_table_marker().
        self.surface = World._get_surface_marker(pose, dimensions)
        self._im_server.insert(self.surface, self.marker_feedback_cb)
        self._im_server.applyChanges()

        # NOTE(max): Testing to get a legend... who knows whether this will
        # work...
        legend = Legend()

    def _reset_objects(self):
        '''Function that removes all objects'''
        self._lock.acquire()
        for i in range(len(World.objects)):
            self._im_server.erase(World.objects[i].int_marker.name)
            self._im_server.applyChanges()
        if self.surface != None:
            self._remove_surface()
        self._im_server.clear()
        self._im_server.applyChanges()
        World.objects = []
        self._lock.release()

    def receive_table_marker(self, marker):
        '''Callback function for markers to determine table'''
        if (marker.type == Marker.LINE_STRIP):
            if (len(marker.points) == 6):
                rospy.loginfo('Received a TABLE marker.')
                xmin = marker.points[0].x
                ymin = marker.points[0].y
                xmax = marker.points[2].x
                ymax = marker.points[2].y
                depth = xmax - xmin
                width = ymax - ymin

                pose = Pose(marker.pose.position, marker.pose.orientation)
                pose.position.x = pose.position.x + xmin + depth / 2
                pose.position.y = pose.position.y + ymin + width / 2
                dimensions = Vector3(depth, width, 0.01)
                self.surface = World._get_surface_marker(pose, dimensions)
                self._im_server.insert(self.surface,
                                     self.marker_feedback_cb)
                self._im_server.applyChanges()

    def receive_object_info(self, object_list):
        '''Callback function to receive object info'''
        self._lock.acquire()
        rospy.loginfo('Received recognized object list.')
        if (len(object_list.graspable_objects) > 0):
            for i in range(len(object_list.graspable_objects)):
                models = object_list.graspable_objects[i].potential_models
                if (len(models) > 0):
                    object_pose = None
                    best_confidence = 0.0
                    for j in range(len(models)):
                        if (best_confidence < models[j].confidence):
                            object_pose = models[j].pose.pose
                            best_confidence = models[j].confidence
                    if (object_pose != None):
                        rospy.logwarn('Adding the recognized object ' +
                                      'with most confident model.')
                        self._add_new_object(object_pose, # pose
                            Vector3(0.2, 0.2, 0.2), # dimensions
                            True, # is_recognized
                            object_list.meshes[i]) # mesh
                else:
                    rospy.logwarn('... this is not a recognition result, ' +
                                  'it is probably just segmentation.')
                    cluster = object_list.graspable_objects[i].cluster
                    bbox = self._bb_service(cluster)
                    cluster_pose = bbox.pose.pose
                    if (cluster_pose != None):
                        rospy.loginfo('Adding unrecognized object with\n' +
                            '- pose:' + World.pose_to_string(cluster_pose) +
                            '- dimensions: ' + World.vector_to_string(
                                bbox.box_dims) + '\n' +
                            '- in ref frame: ' + str(bbox.pose.header.frame_id))
                        self._add_new_object(cluster_pose, # pose
                            bbox.box_dims, # dimensions
                            False) # is_recognized
        else:
            rospy.logwarn('... but the list was empty.')
        self._lock.release()

    @staticmethod
    def get_pose_from_transform(transform):
        '''Returns pose for transformation matrix'''
        pos = transform[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return Pose(Point(pos[0], pos[1], pos[2]),
                    Quaternion(rot[0], rot[1], rot[2], rot[3]))

    @staticmethod
    def get_matrix_from_pose(pose):
        '''Returns the transformation matrix for given pose'''
        rotation = [pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w]
        transformation = tf.transformations.quaternion_matrix(rotation)
        position = [pose.position.x, pose.position.y, pose.position.z]
        transformation[:3, 3] = position
        return transformation

    @staticmethod
    def get_absolute_pose(arm_state):
        '''Returns absolute pose of an end effector state'''
        if (arm_state.refFrame == ArmState.OBJECT):
            arm_state_copy = ArmState(arm_state.refFrame,
                            Pose(arm_state.ee_pose.position,
                                 arm_state.ee_pose.orientation),
                            arm_state.joint_pose[:],
                            arm_state.refFrameObject)
            World.convert_ref_frame(arm_state_copy, ArmState.ROBOT_BASE)
            return arm_state_copy.ee_pose
        else:
            return arm_state.ee_pose

    @staticmethod
    def get_most_similar_obj(ref_object, ref_frame_list):
        '''Finds the most similar object in the world'''
        best_dist = 10000
        chosen_obj_index = -1
        for i in range(len(ref_frame_list)):
            dist = World.object_dissimilarity(ref_frame_list[i], ref_object)
            if (dist < best_dist):
                best_dist = dist
                chosen_obj_index = i
        if chosen_obj_index == -1:
            rospy.logwarn('Did not find a similar object..')
            return None
        else:
            print 'Object dissimilarity is --- ', best_dist
            if best_dist > 0.075:
                rospy.logwarn('Found some objects, but not similar enough.')
                return None
            else:
                rospy.loginfo('Most similar to new object '
                                        + str(chosen_obj_index))
                return ref_frame_list[chosen_obj_index]

    @staticmethod
    def _get_mesh_marker(marker, mesh):
        '''Function that generated a marker from a mesh'''
        marker.type = Marker.TRIANGLE_LIST
        index = 0
        marker.scale = Vector3(1.0, 1.0, 1.0)
        while (index + 2 < len(mesh.triangles)):
            if ((mesh.triangles[index] < len(mesh.vertices))
                    and (mesh.triangles[index + 1] < len(mesh.vertices))
                    and (mesh.triangles[index + 2] < len(mesh.vertices))):
                marker.points.append(mesh.vertices[mesh.triangles[index]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 1]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 2]])
                index += 3
            else:
                rospy.logerr('Mesh contains invalid triangle!')
                break
        return marker

    def _add_new_object(self, pose, dimensions, is_recognized, mesh=None):
        '''Function to add new objects'''
        dist_threshold = 0.02
        to_remove = None
        if (is_recognized):
            # Temporary HACK for testing.
            # Will remove all recognition completely if this works.
            return False
            # Check if there is already an object
            for i in range(len(World.objects)):
                distance = World.pose_distance(World.objects[i].object.pose,
                                               pose)
                if (distance < dist_threshold):
                    if (World.objects[i].is_recognized):
                        rospy.loginfo('Previously recognized object at ' +
                            'the same location, will not add this object.')
                        return False
                    else:
                        rospy.loginfo('Previously unrecognized object at ' +
                            'the same location, will replace it with the ' +
                            'recognized object.')
                        to_remove = i
                        break

            if (to_remove != None):
                self._remove_object(to_remove)
            n_objects = len(World.objects)
            self._add_new_object_internal(WorldObject(pose, n_objects,
                dimensions, is_recognized), mesh)
            return True
        else:
            for i in range(len(World.objects)):
                if (World.pose_distance(World.objects[i].object.pose, pose)
                        < dist_threshold):
                    rospy.loginfo('Previously detected object at the same ' +
                                  'location, will not add this object.')
                    return False
            # NOTE(max): Not passing mesh along here because it's None if it
            # wasn't recognized.
            n_objects = len(World.objects)
            self._add_new_object_internal(WorldObject(pose, n_objects,
                dimensions, is_recognized))
            return True

    def _add_new_object_internal(self, worldObject, mesh=None):
        '''Refactoring as this code is used in three places. Just does the guts
        of adding a new object internally after it's been created'''
        World.objects.append(worldObject)
        int_marker = self._get_object_marker(len(World.objects) - 1, mesh)
        World.objects[-1].int_marker = int_marker
        self._im_server.insert(int_marker, self.marker_feedback_cb)
        self._im_server.applyChanges()
        World.objects[-1].menu_handler.apply(self._im_server, int_marker.name)
        # NOTE(max): Necessary to call applyChanges() twice?
        self._im_server.applyChanges()


    def _remove_object(self, to_remove):
        '''Function to remove object by index'''
        obj = World.objects.pop(to_remove)
        rospy.loginfo('Removing object ' + obj.int_marker.name)
        self._im_server.erase(obj.int_marker.name)
        self._im_server.applyChanges()
#        if (obj.is_recognized):
#            for i in range(len(World.objects)):
#                if ((World.objects[i].is_recognized)
#                    and World.objects[i].index>obj.index):
#                    World.objects[i].decrease_index()
#            self.n_recognized -= 1
#        else:
#            for i in range(len(World.objects)):
#                if ((not World.objects[i].is_recognized) and
#                    World.objects[i].index>obj.index):
#                    World.objects[i].decrease_index()
#            self.n_unrecognized -= 1

    def _remove_surface(self):
        '''Function to request removing surface'''
        rospy.loginfo('Removing surface')
        self._im_server.erase('surface')
        self._im_server.applyChanges()
        self.surface = None

    def _get_object_marker(self, index, mesh=None):
        '''Generate a marker for world objects'''
        int_marker = InteractiveMarker()
        int_marker.name = World.objects[index].get_name()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = World.objects[index].object.pose
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.NONE
        button_control.always_visible = True

        object_marker = Marker(type=Marker.CUBE, id=index,
                lifetime=rospy.Duration(2),
                scale=World.objects[index].object.dimensions,
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.2, 0.8, 0.0, 0.6),
                pose=World.objects[index].object.pose)

        if (mesh != None):
            object_marker = World._get_mesh_marker(object_marker, mesh)
        button_control.markers.append(object_marker)

        text_pos = Point()
        text_pos.x = World.objects[index].object.pose.position.x
        text_pos.y = World.objects[index].object.pose.position.y
        text_pos.z = (World.objects[index].object.pose.position.z +
                     World.objects[index].object.dimensions.z / 2 + 0.06)
        # Remove object label
        button_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
                id=index, scale=Vector3(0, 0, 0.05),
                text=int_marker.name, color=ColorRGBA(0.0, 0.0, 0.0, 1.0),
                header=Header(frame_id='base_link'),
                pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))
        int_marker.controls.append(button_control)
        return int_marker

    @staticmethod
    def _get_surface_marker(pose, dimensions):
        ''' Function that generates a surface marker'''
        int_marker = InteractiveMarker()
        int_marker.name = 'surface'
        int_marker.header.frame_id = 'base_link'
        pose.position.z = pose.position.z + 0.03
        int_marker.pose = pose
        int_marker.scale = 1
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.NONE
        button_control.always_visible = True
        object_marker = Marker(type=Marker.CUBE, id=2000,
                            lifetime=rospy.Duration(2),
                            scale=dimensions,
                            header=Header(frame_id='base_link'),
                            color=ColorRGBA(0.5, 0.5, 0.5, 0.4),
                            pose=pose)
        button_control.markers.append(object_marker)
        text_pos = Point()
        position = pose.position
        dimensions = dimensions
        text_pos.x = position.x + dimensions.x / 2 - 0.06
        text_pos.y = position.y - dimensions.y / 2 + 0.06
        text_pos.z = position.z + dimensions.z / 2 + 0.06
        text_marker = Marker(type=Marker.TEXT_VIEW_FACING, id=2001,
                scale=Vector3(0, 0, 0.05), text=int_marker.name,
                color=ColorRGBA(0.0, 0.0, 0.0, 1.0),
                header=Header(frame_id='base_link'),
                pose=Pose(text_pos, Quaternion(0, 0, 0, 1)))
        #button_control.markers.append(text_marker)
        int_marker.controls.append(button_control)
        return int_marker

    def get_frame_list(self, action_index):
        '''Function that returns the list of ref. frames; actually used now as
        a signal for which action index to load the mocked objects.'''
        # NOTE(max): Implicitly, this is called with the action_index that we'll
        # be in next / soon (unless it's called with an invalid number, which is
        # possible). This means that here we don't want to just return the world
        # object list, but we want to use it as a trigger for mocking in the new
        # objects. This is because the world isn't notified when you switch
        # actions, so this method call is the best of a notification we get!
        if action_index > 0 and action_index <= self._max_mocked_action_idx \
            and action_index != self._cur_action_idx:
            # valid new action, so we mock the new objects
            rospy.loginfo("Mocking objects for action " + str(action_index))
            self._mock_objects_for_action(action_index)
            self._cur_action_idx = action_index
        return self._get_underlying_objects()

    def _get_underlying_objects(self):
        '''Grabs the static object list and returns it.'''
        objects = []
        for i in range(len(World.objects)):
            objects.append(World.objects[i].object)
        return objects        

    @staticmethod
    def has_objects():
        '''Function that checks if there are any objects'''
        return len(World.objects) > 0

    @staticmethod
    def object_dissimilarity(obj1, obj2):
        '''Distance between two objects'''
        dims1 = obj1.dimensions
        dims2 = obj2.dimensions
        return norm(array([dims1.x, dims1.y, dims1.z]) -
                    array([dims2.x, dims2.y, dims2.z]))

    @staticmethod
    def get_ref_from_name(ref_name):
        '''Ref. frame type from ref. frame name'''
        if ref_name == 'base_link':
            return ArmState.ROBOT_BASE
        else:
            return ArmState.OBJECT

    @staticmethod
    def convert_ref_frame(arm_frame, ref_frame, ref_frame_obj=Object()):
        '''Transforms an arm frame to a new ref. frame'''
        if ref_frame == ArmState.ROBOT_BASE:
            if (arm_frame.refFrame == ArmState.ROBOT_BASE):
                rospy.logwarn('No reference frame transformations ' +
                              'needed (both absolute).')
            elif (arm_frame.refFrame == ArmState.OBJECT):
                abs_ee_pose = World.transform(arm_frame.ee_pose,
                                arm_frame.refFrameObject.name, 'base_link')
                arm_frame.ee_pose = abs_ee_pose
                arm_frame.refFrame = ArmState.ROBOT_BASE
                arm_frame.refFrameObject = Object()
            else:
                rospy.logerr('Unhandled reference frame conversion:' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        elif ref_frame == ArmState.OBJECT:
            if (arm_frame.refFrame == ArmState.ROBOT_BASE):
                rel_ee_pose = World.transform(arm_frame.ee_pose,
                            'base_link', ref_frame_obj.name)
                arm_frame.ee_pose = rel_ee_pose
                arm_frame.refFrame = ArmState.OBJECT
                arm_frame.refFrameObject = ref_frame_obj
            elif (arm_frame.refFrame == ArmState.OBJECT):
                if (arm_frame.refFrameObject.name == ref_frame_obj.name):
                    rospy.logwarn('No reference frame transformations ' +
                                  'needed (same object).')
                else:
                    rel_ee_pose = World.transform(arm_frame.ee_pose,
                        arm_frame.refFrameObject.name, ref_frame_obj.name)
                    arm_frame.ee_pose = rel_ee_pose
                    arm_frame.refFrame = ArmState.OBJECT
                    arm_frame.refFrameObject = ref_frame_obj
            else:
                rospy.logerr('Unhandled reference frame conversion:' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        return arm_frame

    @staticmethod
    def has_object(object_name):
        '''Checks if the world contains the object'''
        for obj in World.objects:
            if obj.object.name == object_name:
                return True
        return False

    @staticmethod
    def is_frame_valid(object_name):
        '''Checks if the frame is valid for transforms'''
        return (object_name == 'base_link') or World.has_object(object_name)

    @staticmethod
    def transform(pose, from_frame, to_frame):
        ''' Function to transform a pose between two ref. frames
        if there is a TF exception or object does not exist it
        will return the pose back without any transforms'''
        if World.is_frame_valid(from_frame) and World.is_frame_valid(to_frame):
            pose_stamped = PoseStamped()
            try:
                common_time = World.tf_listener.getLatestCommonTime(from_frame,
                                                                    to_frame)
                pose_stamped.header.stamp = common_time
                pose_stamped.header.frame_id = from_frame
                pose_stamped.pose = pose
                rel_ee_pose = World.tf_listener.transformPose(to_frame,
                                                              pose_stamped)
                return rel_ee_pose.pose
            except tf.Exception:
                rospy.logerr('TF exception during transform.')
                return pose
            except rospy.ServiceException:
                rospy.logerr('Exception during transform.')
                return pose
        else:
            rospy.logwarn('One of the frame objects might not exist: ' +
                          from_frame + ' or ' + to_frame)
            return pose

    @staticmethod
    def pose_to_string(pose):
        '''For printing a pose to stdout'''
        return '\t- position:   ' + World.vector_to_string(pose.position) + \
            '\n\t- orientation: ' + World.vector_to_string(pose.orientation) + \
            '\n'

    @staticmethod
    def vector_to_string(vector_like):
        '''For printing something with x, y, z, and possibly w attributes to
        stdout, when we don't want a bunch of newlines. This returns a string of
        the form "[x, y, z[, w]]" (w optional). We might want this when printing
        an object that is a:
         - Point (e.g. pose.position)
         - Quaternion (e.g. pose.orientation)
         - Vector3 (e.g. boudning box dimensions)
         - etc.
         '''
        # TODO(max): Actually write this.
        w = '' if not hasattr(vector_like, 'w') else ', ' + str(vector_like.w)
        return '[' + str(vector_like.x) + ', ' + str(vector_like.y) + ', ' + \
            str(vector_like.z) + w + ']'

    def _publish_tf_pose(self, pose, name, parent):
        ''' Publishes a TF for object pose'''
        if (pose != None):
            pos = (pose.position.x, pose.position.y, pose.position.z)
            rot = (pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w)
            self._tf_broadcaster.sendTransform(pos, rot,
                                        rospy.Time.now(), name, parent)

    def update_object_pose(self):
        ''' Function to externally update an object pose. Returns bool of
        success or failure'''
        # NOTE(max): Mocking this for now
        return True

        Response.perform_gaze_action(GazeGoal.LOOK_DOWN)
        while (Response.gaze_client.get_state() == GoalStatus.PENDING or
               Response.gaze_client.get_state() == GoalStatus.ACTIVE):
            time.sleep(0.1)

        if (Response.gaze_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logerr('Could not look down to take table snapshot')
            return False

        rospy.loginfo('Looking at table now.')
        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        self._reset_objects()

        if (self._object_action_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logerr('Could not reset recognition.')
            return False

        # Do segmentation
        goal = UserCommandGoal(UserCommandGoal.SEGMENT, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Table segmentation is complete.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())

        if (self._object_action_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logerr('Could not segment.')
            return False

        # Do recognition
        goal = UserCommandGoal(UserCommandGoal.RECOGNIZE, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Objects on the table have been recognized.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())

        # Record the result
        if (self._object_action_client.get_state() == GoalStatus.SUCCEEDED):
            wait_time = 0
            total_wait_time = 5
            while (not World.has_objects() and wait_time < total_wait_time):
                time.sleep(0.1)
                wait_time += 0.1

            if (not World.has_objects()):
                rospy.logerr('Timeout waiting for a recognition result.')
                return False
            else:
                rospy.loginfo('Got the object list.')
                return True
        else:
            rospy.logerr('Could not recognize.')
            return False

    def clear_all_objects(self):
        '''Removes all objects from the world'''
        # NOTE(max): Mocking this as well; for now we just clear the mocked
        # objects until they (or different ones) are added.
        self._reset_objects()
        return

        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        if (self._object_action_client.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('Successfully reset object localization pipeline.')
            self._reset_objects()
        self._remove_surface()

    def get_nearest_object(self, arm_pose):
        '''Gives a pointed to the nearest object'''
        # DEBUG(max): arm_pose is being passed here as None???
        if arm_pose == None:
            rospy.logwarn("arm_pose passed to World::get_nearest_object is None")
            return None
        distances = []
        toRet = None
        # NOTE(max): Race conditions can happen on world objects when they're
        # being swapped out during mocking.
        self._lock.acquire()
        for i in range(len(World.objects)):
            dist = World.pose_distance(World.objects[i].object.pose,
                                                            arm_pose)
            distances.append(dist)
        dist_threshold = 0.4
        if (len(distances) > 0):
            if (min(distances) < dist_threshold):
                chosen = distances.index(min(distances))
                toRet = World.objects[chosen].object
        self._lock.release()
        return toRet

    @staticmethod
    def pose_distance(pose1, pose2, is_on_table=True):
        '''Distance between two world poses'''
        if pose1 == [] or pose2 == []:
            return 0.0
        else:
            if (is_on_table):
                arr1 = array([pose1.position.x, pose1.position.y])
                arr2 = array([pose2.position.x, pose2.position.y])
            else:
                arr1 = array([pose1.position.x,
                              pose1.position.y, pose1.position.z])
                arr2 = array([pose2.position.x,
                              pose2.position.y, pose2.position.z])
            dist = norm(arr1 - arr2)
            if dist < 0.0001:
                dist = 0
            return dist

    def marker_feedback_cb(self, feedback):
        '''Callback for when feedback from a marker is received'''
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
            rospy.loginfo('Number of objects ' + str(len(World.objects)))
        else:
            rospy.loginfo('Unknown event' + str(feedback.event_type))

    def update(self):
        '''Update function called in a loop'''
        # Visualize the detected object
        is_world_changed = False
        self._lock.acquire()
        if (World.has_objects()):
            to_remove = None
            for i in range(len(World.objects)):
                self._publish_tf_pose(World.objects[i].object.pose,
                    World.objects[i].get_name(), 'base_link')
                if (World.objects[i].is_removed):
                    to_remove = i
            if to_remove != None:
                self._remove_object(to_remove)
                is_world_changed = True

        self._lock.release()
        return is_world_changed

'''Everything related to perception of the world'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# System builtins
import struct
import threading
import time

# 3rd party
import numpy as np
from numpy.linalg import norm

# ROS builtins
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Header
import tf
from tf import TransformListener, TransformBroadcaster

# ROS 3rd party
from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer)
from interactive_markers.menu_handler import MenuHandler
from manipulation_msgs.msg import GraspableObjectList
from object_manipulation_msgs.srv import FindClusterBoundingBox
from pr2_interactive_object_detection.msg import (
    UserCommandAction, UserCommandGoal)
from visualization_msgs.msg import (
    Marker, InteractiveMarker, InteractiveMarkerControl,
    InteractiveMarkerFeedback)

# Local
from pr2_pbd_interaction.msg import Object, ArmState, HandsFreeCommand
from pr2_social_gaze.msg import GazeGoal
from response import Response


# ######################################################################
# Module level constants
# ######################################################################

# Two objects must be closer than this to be considered 'the same'.
OBJ_SIMILAR_DIST_THRESHHOLD = 0.075

# When adding objects, if they are closer than this they'll replace one
# another.
OBJ_ADD_DIST_THRESHHOLD = 0.02

# How close to 'nearest' object something must be to be counted as
# 'near' it.
OBJ_NEAREST_DIST_THRESHHOLD = 0.4

# Object distances below this will be clamped to zero.
OBJ_DIST_ZERO_CLAMP = 0.0001

ACTION_OBJ_DETECTION = 'object_detection_user_command'
TOPIC_OBJ_RECOGNITION = 'interactive_object_recognition_result'
TOPIC_TABLE_SEG = 'tabletop_segmentation_markers'
TOPIC_IM_SERVER = 'world_objects'
SERVICE_BB = 'find_cluster_bounding_box'

# Scales
SCALE_TEXT = Vector3(0.0, 0.0, 0.03)
SURFACE_HEIGHT = 0.01  # 0.01 == 1cm (I think)
OFFSET_OBJ_TEXT_Z = 0.06  # How high objects' labels are above them.
# Object dimensions. I don't fully understand this, as it seems like
# each object's dimensions should be extracted from the point cloud.
# But apparently this works and is a default or something?
DIMENSIONS_OBJ = Vector3(0.2, 0.2, 0.2)

# Colors
COLOR_OBJ = ColorRGBA(0.2, 0.8, 0.0, 0.6)
COLOR_SURFACE = ColorRGBA(0.8, 0.0, 0.4, 0.4)
COLOR_TEXT = ColorRGBA(0.0, 0.0, 0.0, 0.5)
COLOR_REACHABLE = ColorRGBA(0.2, 0.8, 0.0, 0.6)
COLOR_UNREACHABLE = ColorRGBA(0.8, 0.2, 0.0, 0.6)

# Frames
BASE_LINK = 'base_link'

# Time
MARKER_DURATION = rospy.Duration(2)
# How long to pause when waiting for external code, like gaze actions or
# object segmentation, to finish before checking again.
PAUSE_SECONDS = rospy.Duration(0.1)
# How long we're willing to wait for object recognition.
RECOGNITION_TIMEOUT_SECONDS = rospy.Duration(5.0)
# How long we're willing to wait for the PR2 to look at the table.
LOOK_AT_TABLE_TIMEOUT_SECONDS = rospy.Duration(5.0)


# For ids to not clash. Assumes that we have < this many steps and
# aren't forgetting about other offsets elsewhere in the program that
# are taking up this range. Hmm.
PBDOBJ_ID_BASE_OFFSET = 500
# Must have < this many pbd obj reachability markers.
PBDOBJ_ID_MULTIPLIER_OFFSET = 10


# ######################################################################
# Classes
# ######################################################################

class PbdObject(object):
    '''Class for representing objects interally in PbD.'''

    # For making uids
    locnames = [
        HandsFreeCommand.ABOVE,
        HandsFreeCommand.TO_LEFT_OF,
        HandsFreeCommand.TO_RIGHT_OF,
        HandsFreeCommand.IN_FRONT_OF,
        HandsFreeCommand.BEHIND,
        HandsFreeCommand.ON_TOP_OF,
    ]

    def __init__(
            self, graspable_object, cluster, pose, index, dimensions,
            is_recognized):
        '''
        Args:
            graspable_object (GraspableObject)
            cluster (sensor_msgs/PointCloud)
            pose (Pose): Position of bounding box
            index (int): For naming object in world (e.g. "thing 0")
            dimensions (Vector3): Size of bounding box
            is_recognized (bool): Result of object recognition.
        '''
        # Core attrs.
        self.gobj = graspable_object
        self.cluster = cluster
        self.pose = pose
        self.index = index
        self.dimensions = dimensions
        self.is_recognized = is_recognized
        self.assigned_name = None
        self.name = self.get_name()

        # Attrs computed for Hands-free.
        self.type = 'unknown'
        self.color = PbdObject.get_color(cluster)
        self.points = PbdObject.get_points(cluster)
        self.endpoints = PbdObject.get_endpoints(self.points)
        self.vol = PbdObject.get_vol(dimensions)

        # These are for hands-free, but are computed later on demand.
        self.reachability_map = None  # { str: ReachableResult}
        self.pickupable = None

        # Object stuff
        self.object = Object(
            Object.TABLE_TOP, self.name, pose, dimensions)
        self.menu_handler = MenuHandler()
        self.int_marker = None
        self.is_removed = False
        self.menu_handler.insert('Remove from scene', callback=self.remove)
        self.display()

    def display(self):
        '''Display info (for debug).'''
        rospy.loginfo('Object: ' + self.get_name())
        rospy.loginfo('\t- color: ' + self.color)
        rospy.loginfo('\t- type:  ' + self.type)
        rospy.loginfo('\t- vol:   ' + str(self.vol))
        p_descs = [
            'rightmost',
            'leftmost',
            'farthest',
            'nearest',
            'highest',
            'lowest',
        ]
        for idx, desc in enumerate(p_descs):
            rospy.loginfo(
                '\t- ' + desc + ' point:  ' + str(self.endpoints[idx]))

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return self.name

    def add_reachable_marker(self, loc_name, loc, reachable):
        '''
        Args:
            loc_name (str)
            loc (Point)
            reachable (bool)
        '''
        uid = (
            PBDOBJ_ID_BASE_OFFSET + self.index * PBDOBJ_ID_MULTIPLIER_OFFSET +
            PbdObject.locnames.index(loc_name))
        radius = 0.05
        color = COLOR_REACHABLE if reachable else COLOR_UNREACHABLE
        # name = self.name + 'r-' + str(PbdObject.locnames.index(loc_name))
        marker = Marker(
            type=Marker.SPHERE,
            # name=name,
            id=uid,
            lifetime=rospy.Duration(20),
            scale=Vector3(radius, radius, radius),
            pose=Pose(loc, Quaternion(0, 0, 0, 1)),
            header=Header(frame_id=BASE_LINK),
            color=color
        )
        # In case it already exists, remove it.
        # World.im_server.erase(name)
        rospy.loginfo(
            "Appending marker for " + self.name + ' ' + loc_name + ' ' +
            str(reachable))
        self.int_marker.controls[0].markers.append(marker)
        # NOTE(mbforbes): Must apply changes elsewhere.

    @staticmethod
    def get_endpoints(points):
        '''
        Returns the farthest points in each direction.

        Args:
            points (numpy Array [n_points x 3]): A numpy array of
                dimensions n_points x 3, with the (x, y, z) coordinates
                of each point.

        Returns:
            [float]: Elements:
                - rightmost val (y axis most negative) [0]
                - leftmost val (y axis most positive)  [1]
                - farthest val (x axis most positive)  [2]
                - nearest val (x axis most negative)   [3]
                - highest val (z axis most positive)   [4]
                - lowest val (z axis most negative)    [5]
        '''
        # Look column-wise down the 2D points array.
        xmax, ymax, zmax = np.max(points, 0)
        xmin, ymin, zmin = np.min(points, 0)
        return [
            ymin,
            ymax,
            xmax,
            xmin,
            zmax,
            xmin
        ]

    @staticmethod
    def get_color(cluster):
        '''
        Returns color of cluster as string.

        Args:
            cluster (PointCloud)

        Returns:
            str: one of:
                - 'red'
                - 'green'
                - 'blue'
                - 'unknown'
        '''
        for c in cluster.channels:
            if c.name == 'rgb':
                # r, g, b values packed into least significant 24 bits.
                res = np.zeros((len(c.values), 3))
                mask = (2 << 7) - 1
                for idx, v in enumerate(c.values):
                    raw = PbdObject.ftb(v)
                    res[idx] = [(raw >> shift) & mask for shift in [0, 8, 16]]
                # Average down the columns.
                # NOTE(mbforbes): This is where computer vision people
                # cry.
                avgs = np.average(res, 0)

                # Temp logging
                b, g, r = avgs
                rospy.loginfo('red: ' + str(int(r)))
                rospy.loginfo('green: ' + str(int(g)))
                rospy.loginfo('blue: ' + str(int(b)))

                # Ret!
                return ['blue', 'green', 'red'][np.argmax(avgs)]

        # No rgb channel found.
        return 'unknown'

    @staticmethod
    def get_points(cluster):
        '''
        Creates a new numpy array from a point cloud that contains the
        points position data. This is faster to use later.

        Args:
            cluster (sensor_msgs/PointCloud)

        Returns:
            2D numpy Array (n_points x 3)
        '''
        # NOTE(mbforbes): Do we have to do any transforms???
        ret = np.zeros((len(cluster.points), 3))
        for idx, point in enumerate(cluster.points):
            ret[idx] = point.x, point.y, point.z
        return ret

    @staticmethod
    def ftb(f):
        '''
        Float to bits (well, to int, but then we can get bits).

        Args:
            f (float)

        Returns:
            int
        '''
        s = struct.pack('>f', f)
        return struct.unpack('>l', s)[0]

    @staticmethod
    def get_vol(d):
        '''
        Returns volume.

        Args:
            d (Vector3): Dimensions of the object.

        Returns:
            float
        '''
        return d.x * d.y * d.z

    def get_name(self):
        '''Return this object's name.

        Returns:
            str
        '''
        if self.assigned_name is None:
            if self.is_recognized:
                return 'object ' + str(self.index)
            else:
                return 'thing ' + str(self.index)
        else:
            return self.assigned_name

    def remove(self, __):
        '''Function for removing object from the world.

        Args:
            __ (???): Unused
        '''
        rospy.loginfo('Will remove object: ' + self.get_name())
        self.is_removed = True

    # TODO(mbforbes): Re-implement object recognition or remove
    # this dead code.

    # def assign_name(self, name):
    #     '''Function for assigning a different name to this object.

    #     Args:
    #         name (str): The new name.
    #     '''
    #     self.assigned_name = name
    #     self.object.name = name

    # def decrease_index(self):
    #     '''Function to decrese object index.'''
    #     self.index -= 1


class World:
    '''Handles object recognition, localization, and coordinate space
    transformations.'''

    tf_listener = None

    # Type: [PbdObject]
    objects = []
    im_server = None

    def __init__(self):
        # Public attributes
        if World.tf_listener is None:
            World.tf_listener = TransformListener()
        self.surface = None
        if World.im_server is None:
            World.im_server = InteractiveMarkerServer(TOPIC_IM_SERVER)

        # Private attributes
        self._lock = threading.Lock()
        self._tf_broadcaster = TransformBroadcaster()
        rospy.wait_for_service(SERVICE_BB)
        self._bb_service = rospy.ServiceProxy(
            SERVICE_BB, FindClusterBoundingBox)
        self._object_action_client = actionlib.SimpleActionClient(
            ACTION_OBJ_DETECTION, UserCommandAction)
        self._object_action_client.wait_for_server()
        rospy.loginfo(
            'Interactive object detection action server has responded.')

        # Setup other ROS machinery
        rospy.Subscriber(
            TOPIC_OBJ_RECOGNITION, GraspableObjectList,
            self.receive_object_info)
        rospy.Subscriber(TOPIC_TABLE_SEG, Marker, self.receive_table_marker)

        # Init
        self.clear_all_objects()

    # ##################################################################
    # Static methods: Public (API)
    # ##################################################################

    @staticmethod
    def get_pose_from_transform(transform):
        '''Returns pose for transformation matrix.

        Args:
            transform (Matrix3x3): (I think this is the correct type.
                See ActionStepMarker as a reference for how to use.)

        Returns:
            Pose
        '''
        pos = transform[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return Pose(
            Point(pos[0], pos[1], pos[2]),
            Quaternion(rot[0], rot[1], rot[2], rot[3])
        )

    @staticmethod
    def get_matrix_from_pose(pose):
        '''Returns the transformation matrix for given pose.

        Args:
            pose (Pose)

        Returns:
            Matrix3x3: (I think this is the correct type. See
                ActionStepMarker as a reference for how to use.)
        '''
        pp, po = pose.position, pose.orientation
        rotation = [po.x, po.y, po.z, po.w]
        transformation = tf.transformations.quaternion_matrix(rotation)
        position = [pp.x, pp.y, pp.z]
        transformation[:3, 3] = position
        return transformation

    @staticmethod
    def get_absolute_pose(arm_state):
        '''Returns absolute pose of an end effector state (trasnforming
        if relative).

        Args:
            arm_state (ArmState)

        Returns:
            Pose
        '''
        if arm_state.refFrame == ArmState.OBJECT:
            arm_state_copy = ArmState(
                arm_state.refFrame, Pose(
                    arm_state.ee_pose.position,
                    arm_state.ee_pose.orientation),
                arm_state.joint_pose[:],
                arm_state.refFrameObject)
            World.convert_ref_frame(arm_state_copy, ArmState.ROBOT_BASE)
            return arm_state_copy.ee_pose
        else:
            return arm_state.ee_pose

    @staticmethod
    def get_most_similar_obj(ref_object, ref_frame_list):
        '''Finds the most similar object in the world.

        Args:
            ref_object (?)
            ref_frame_list ([Object]): List of objects (as defined by
                Object.msg).

        Returns:
            Object|None: As in one of Object.msg, or None if no object
                was found close enough.
        '''
        best_dist = 10000  # Not a constant; an absurdly high number.
        chosen_obj = None
        for ref_frame in ref_frame_list:
            dist = World.object_dissimilarity(ref_frame, ref_object)
            if dist < best_dist:
                best_dist = dist
                chosen_obj = ref_frame
        if chosen_obj is None:
            rospy.loginfo('Did not find a similar object.')
        else:
            rospy.loginfo('Object dissimilarity is --- ' + str(best_dist))
            if best_dist > OBJ_SIMILAR_DIST_THRESHHOLD:
                rospy.loginfo('Found some objects, but not similar enough.')
                chosen_obj = None
            else:
                rospy.loginfo(
                    'Most similar to new object: ' + str(chosen_obj.name))

        # Regardless, return the "closest object," which may be None.
        return chosen_obj

    @staticmethod
    def get_frame_list():
        '''Function that returns the list of reference frames (Objects).

        Returns:
            [Object]: List of Object (as defined by Object.msg), the
                current reference frames.
        '''
        return [w_obj.object for w_obj in World.objects]

    @staticmethod
    def get_objs():
        '''
        Now we'll actually use the internal class for once.

        Returns:
            [PbdObject]
        '''
        return World.objects

    @staticmethod
    def has_objects():
        '''Returns whetehr there are any objects (reference frames).

        Returns:
            bool
        '''
        return len(World.objects) > 0

    @staticmethod
    def object_dissimilarity(obj1, obj2):
        '''Returns distance between two objects.

        Returns:
            float
        '''
        d1 = obj1.dimensions
        d2 = obj2.dimensions
        return norm(
            np.array([d1.x, d1.y, d1.z]) - np.array([d2.x, d2.y, d2.z]))

    @staticmethod
    def get_ref_from_name(ref_name):
        '''Returns the reference frame type from the reference frame
        name specified by ref_name.

        Args:
            ref_name (str): Name of a referene frame.

        Returns:
            int: One of ArmState.*, the number code of the reference
                frame specified by ref_name.
        '''
        if ref_name == 'base_link':
            return ArmState.ROBOT_BASE
        else:
            return ArmState.OBJECT

    @staticmethod
    def convert_ref_frame(arm_frame, ref_frame, ref_frame_obj=Object()):
        '''Transforms an arm frame to a new ref. frame.

        Args:
            arm_frame (ArmState)
            ref_frame (int): One of ArmState.*
            ref_frame_obj (Object): As in Object.msg

        Returns:
            ArmState: arm_frame (passed in), but modified.
        '''
        if ref_frame == ArmState.ROBOT_BASE:
            if arm_frame.refFrame == ArmState.ROBOT_BASE:
                # Transform from robot base to itself (nothing to do).
                rospy.logdebug(
                    'No reference frame transformations needed (both ' +
                    'absolute).')
            elif arm_frame.refFrame == ArmState.OBJECT:
                # Transform from object to robot base.
                abs_ee_pose = World.transform(
                    arm_frame.ee_pose,
                    arm_frame.refFrameObject.name,
                    'base_link'
                )
                arm_frame.ee_pose = abs_ee_pose
                arm_frame.refFrame = ArmState.ROBOT_BASE
                arm_frame.refFrameObject = Object()
            else:
                rospy.logerr(
                    'Unhandled reference frame conversion: ' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        elif ref_frame == ArmState.OBJECT:
            if arm_frame.refFrame == ArmState.ROBOT_BASE:
                # Transform from robot base to object.
                rel_ee_pose = World.transform(
                    arm_frame.ee_pose, 'base_link', ref_frame_obj.name)
                arm_frame.ee_pose = rel_ee_pose
                arm_frame.refFrame = ArmState.OBJECT
                arm_frame.refFrameObject = ref_frame_obj
            elif arm_frame.refFrame == ArmState.OBJECT:
                # Transform between the same object (nothign to do).
                if arm_frame.refFrameObject.name == ref_frame_obj.name:
                    rospy.logdebug(
                        'No reference frame transformations needed (same ' +
                        'object).')
                else:
                    # Transform between two different objects.
                    rel_ee_pose = World.transform(
                        arm_frame.ee_pose,
                        arm_frame.refFrameObject.name,
                        ref_frame_obj.name
                    )
                    arm_frame.ee_pose = rel_ee_pose
                    arm_frame.refFrame = ArmState.OBJECT
                    arm_frame.refFrameObject = ref_frame_obj
            else:
                rospy.logerr(
                    'Unhandled reference frame conversion: ' +
                    str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        return arm_frame

    @staticmethod
    def has_object(object_name):
        '''Returns whether the world contains an Object with object_name.

        Args:
            object_name (str)

        Returns:
            bool
        '''
        return object_name in [wobj.object.name for wobj in World.objects]

    @staticmethod
    def is_frame_valid(object_name):
        '''Returns whether the frame (object) name is valid for
        transforms.

        Args:
            object_name (str)

        Returns:
            bool
        '''
        return object_name == 'base_link' or World.has_object(object_name)

    @staticmethod
    def transform(pose, from_frame, to_frame):
        '''Transforms a pose between two reference frames. If there is a
        TF exception or object does not exist, it will return the pose
        back without any transforms.

        Args:
            pose (Pose)
            from_frame (str)
            to_frame (str)

        Returns:
            Pose
        '''
        if World.is_frame_valid(from_frame) and World.is_frame_valid(to_frame):
            pose_stamped = PoseStamped()
            try:
                common_time = World.tf_listener.getLatestCommonTime(
                    from_frame, to_frame)
                pose_stamped.header.stamp = common_time
                pose_stamped.header.frame_id = from_frame
                pose_stamped.pose = pose
                rel_ee_pose = World.tf_listener.transformPose(
                    to_frame, pose_stamped)
                return rel_ee_pose.pose
            except tf.Exception:
                rospy.logerr('TF exception during transform.')
                return pose
            except rospy.ServiceException:
                rospy.logerr('ServiceException during transform.')
                return pose
        else:
            rospy.logdebug(
                'One of the frame objects might not exist: ' + from_frame +
                ' or ' + to_frame)
            return pose

    @staticmethod
    def pose_distance(pose1, pose2, is_on_table=True):
        '''Returns distance between two world poses.

        Args:
            pose1 (Pose)
            pose2 (Pose)
            is_on_table (bool, optional): Whether the objects are on the
                table (if so, disregards z-values in computations).

        Returns:
            float
        '''
        if pose1 == [] or pose2 == []:
            return 0.0
        else:
            p1p = pose1.position
            p2p = pose2.position
            if is_on_table:
                arr1 = np.array([p1p.x, p1p.y])
                arr2 = np.array([p2p.x, p2p.y])
            else:
                arr1 = np.array([p1p.x, p1p.y, p1p.z])
                arr2 = np.array([p2p.x, p2p.y, p2p.z])
            dist = norm(arr1 - arr2)
            if dist < OBJ_DIST_ZERO_CLAMP:
                dist = 0
            return dist

    @staticmethod
    def log_pose(log_fn, pose):
        '''For printing a pose to rosout. We don't do it on one line
        becuase that messes up the indentation with the rest of the log.

        Args:
            log_fn (function(str)): A logging function that takes a
                string as an argument. For example, rospy.loginfo.
            pose (Pose): The pose to log
        '''
        p, o = pose.position, pose.orientation
        log_fn(' - position: (%f, %f, %f)' % (p.x, p.y, p.z))
        log_fn(' - orientation: (%f, %f, %f, %f)' % (o.x, o.y, o.z, o.w))

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _get_mesh_marker(marker, mesh):
        '''Generates and returns a marker from a mesh.

        Args:
            marker (Marker)
            mesh (Mesh)

        Returns:
            Marker
        '''
        marker.type = Marker.TRIANGLE_LIST
        index = 0
        marker.scale = Vector3(1.0, 1.0, 1.0)
        while index + 2 < len(mesh.triangles):
            if (mesh.triangles[index] < len(mesh.vertices)
                    and mesh.triangles[index + 1] < len(mesh.vertices)
                    and mesh.triangles[index + 2] < len(mesh.vertices)):
                marker.points.append(mesh.vertices[mesh.triangles[index]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 1]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 2]])
                index += 3
            else:
                rospy.logerr('Mesh contains invalid triangle!')
                break
        return marker

    @staticmethod
    def _get_surface_marker(pose, dimensions):
        '''Returns a surface marker with provided pose and dimensions.

        Args:
            pose (Pose)
            dimensions  (Vector3)

        Returns:
            InteractiveMarker
        '''
        int_marker = InteractiveMarker()
        int_marker.name = 'surface'
        int_marker.header.frame_id = BASE_LINK
        int_marker.pose = pose
        int_marker.scale = 1
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        object_marker = Marker(
            type=Marker.CUBE,
            id=2000,
            lifetime=MARKER_DURATION,
            scale=dimensions,
            header=Header(frame_id=BASE_LINK),
            color=COLOR_SURFACE,
            pose=pose
        )
        button_control.markers.append(object_marker)
        text_pos = Point()
        position = pose.position
        dimensions = dimensions
        text_pos.x = position.x + dimensions.x / 2 - 0.06
        text_pos.y = position.y - dimensions.y / 2 + 0.06
        text_pos.z = position.z + dimensions.z / 2 + 0.06
        text_marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=2001,
            scale=SCALE_TEXT, text=int_marker.name,
            color=COLOR_TEXT,
            header=Header(frame_id=BASE_LINK),
            pose=Pose(text_pos, Quaternion(0, 0, 0, 1))
        )
        button_control.markers.append(text_marker)
        int_marker.controls.append(button_control)
        return int_marker

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def receive_table_marker(self, marker):
        '''Callback function for markers to determine table'''
        if marker.type == Marker.LINE_STRIP:
            if len(marker.points) == 6:
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
                dimensions = Vector3(depth, width, SURFACE_HEIGHT)
                self.surface = World._get_surface_marker(pose, dimensions)
                World.im_server.insert(
                    self.surface, self.marker_feedback_cb)
                World.im_server.applyChanges()

    def receive_object_info(self, object_list):
        '''Callback function to receive object info'''
        self._lock.acquire()
        rospy.loginfo('Received recognized object list.')
        if len(object_list.graspable_objects) > 0:
            for gobj in object_list.graspable_objects:
                models = gobj.potential_models
                if len(models) > 0:
                    object_pose = None
                    best_confidence = 0.0
                    for model in models:
                        if best_confidence < model.confidence:
                            object_pose = model.pose.pose
                            best_confidence = model.confidence
                    if object_pose is not None:
                        rospy.logwarn(
                            'Adding the recognized object with most ' +
                            'confident model.')
                        self._add_new_object(
                            gobj,
                            gobj.cluster,
                            object_pose,
                            DIMENSIONS_OBJ,
                            True,
                            object_list.meshes[i]
                        )
                else:
                    rospy.logwarn(
                        '... this is not a recognition result, it is ' +
                        'probably just segmentation.')
                    cluster = gobj.cluster
                    bbox = self._bb_service(cluster)
                    cluster_pose = bbox.pose.pose
                    if cluster_pose is not None:
                        rospy.loginfo('Adding unrecognized object with pose:')
                        World.log_pose(rospy.loginfo, cluster_pose)
                        rospy.loginfo(
                            '...in ref frame ' +
                            str(bbox.pose.header.frame_id))
                        self._add_new_object(
                            gobj,
                            cluster,
                            cluster_pose,
                            bbox.box_dims,
                            False
                        )
        else:
            rospy.logwarn('... but the list was empty.')
        self._lock.release()

    def update_object_pose(self):
        '''Call to update object list.

        Returns:
            bool: Success?
        '''
        # Look at the table.
        # NOTE(mbforbes): Ignoring results of this call as it's been
        # returning failure constantly even when it succeeds.
        self._look_at_table()

        # Reset object recognition.
        rospy.loginfo('About to attempt to reset object recognition.')
        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            rospy.sleep(PAUSE_SECONDS)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        self._reset_objects()  # Also do this internally.
        if self._object_action_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Could not reset recognition.')
            return False

        # Do segmentation
        rospy.loginfo('About to attempt table segmentation.')
        goal = UserCommandGoal(UserCommandGoal.SEGMENT, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            rospy.sleep(PAUSE_SECONDS)
        rospy.loginfo('Table segmentation is complete.')
        rospy.loginfo(
            'STATUS: ' + self._object_action_client.get_goal_status_text())
        if self._object_action_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logwarn('Could not segment.')
            return False

        # Do recognition
        rospy.loginfo('About to attempt object recognition.')
        goal = UserCommandGoal(UserCommandGoal.RECOGNIZE, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            rospy.sleep(PAUSE_SECONDS)
        rospy.loginfo('Objects on the table have been recognized.')
        rospy.loginfo(
            'STATUS: ' + self._object_action_client.get_goal_status_text())

        # Record the result
        if self._object_action_client.get_state() == GoalStatus.SUCCEEDED:
            wait_time = rospy.Duration(0.0)
            while (not World.has_objects() and
                    wait_time < RECOGNITION_TIMEOUT_SECONDS):
                rospy.sleep(PAUSE_SECONDS)
                wait_time += PAUSE_SECONDS

            if not World.has_objects():
                rospy.logerr('Timeout waiting for a recognition result.')
                return False
            else:
                rospy.loginfo('Got the object list.')
                return True
        else:
            rospy.logerr('Could not recognize.')
            return False

    def clear_all_objects(self):
        '''Removes all objects from the world.'''
        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
               self._object_action_client.get_state() == GoalStatus.PENDING):
            rospy.sleep(PAUSE_SECONDS)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        if self._object_action_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Successfully reset object localization pipeline.')
        self._reset_objects()
        self._remove_surface()

    def get_nearest_object(self, arm_pose):
        '''Returns the nearest object, if one exists.

        Args:
            arm_pose (Pose): End-effector pose.

        Returns:
            Object|None: As in Object.msg, the nearest object (if it
                is close enough), or None if there were none close
                enough.
        '''
        # First, find which object is the closest.
        distances = []
        for wobj in World.objects:
            dist = World.pose_distance(wobj.object.pose, arm_pose)
            distances.append(dist)

        # Then, see if the closest is actually below our threshhold for
        # a 'closest object.'
        if len(distances) > 0:
            if min(distances) < OBJ_NEAREST_DIST_THRESHHOLD:
                chosen = distances.index(min(distances))
                return World.objects[chosen].object

        # We didn't have any objects or none were close enough.
        return None

    def marker_feedback_cb(self, feedback):
        '''Callback for when feedback from a marker is received.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
            rospy.loginfo('Number of objects ' + str(len(World.objects)))
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))

    def update(self):
        '''Update function called in a loop.

        Returns:
            bool: Whether any tracked objects were removed, AKA "is
                world changed."
        '''
        # Visualize the detected object
        is_world_changed = False
        self._lock.acquire()
        if World.has_objects():
            to_remove = None
            for i in range(len(World.objects)):
                self._publish_tf_pose(
                    World.objects[i].object.pose,
                    World.objects[i].get_name(),
                    BASE_LINK
                )
                if World.objects[i].is_removed:
                    to_remove = i
            if to_remove is not None:
                self._remove_object(to_remove)
                is_world_changed = True

        self._lock.release()
        return is_world_changed

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _look_at_table(self):
        '''
        A step in update_object_pose(...).
        '''
        rospy.loginfo('Head attempting to look at table.')
        Response.perform_gaze_action(GazeGoal.LOOK_DOWN)
        wait_time = rospy.Duration(0.0)
        while ((Response.gaze_client.get_state() == GoalStatus.PENDING or
                Response.gaze_client.get_state() == GoalStatus.ACTIVE) and
                wait_time < LOOK_AT_TABLE_TIMEOUT_SECONDS):
            rospy.sleep(PAUSE_SECONDS)
            wait_time += PAUSE_SECONDS
        # TODO(mbforbes): Let's do a more complicated check of the head
        # angle, as it often "fails" here, but the head actually
        # successfully went where we wanted it to.
        if Response.gaze_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Could not look down to take table snapshot')
            return False
        rospy.loginfo('Head is now (successfully) stairing at table.')
        return True

    def _reset_objects(self):
        '''Removes all objects.'''
        self._lock.acquire()
        for wobj in World.objects:
            World.im_server.erase(wobj.int_marker.name)
            World.im_server.applyChanges()
        if self.surface is not None:
            self._remove_surface()
        World.im_server.clear()
        World.im_server.applyChanges()
        World.objects = []
        self._lock.release()

    def _add_new_object(
            self, graspable_object, cluster, pose, dimensions, is_recognized,
            mesh=None):
        '''Maybe add a new object with the specified properties to our
        object list.

        It might not be added if too similar of an object already
        exists (and has been added).

        Args:
            cluster (sensor_msgs/PointCloud)
            pose (Pose)
            dimensions (Vector3)
            is_recognized (bool)
            mesh (Mesh, optional): A mesh, if it exists. Default is
                None.

        Returns:
            bool: Whether the object was actually added.
        '''
        to_remove = None
        if is_recognized:
            # TODO(mbforbes): Re-implement object recognition or remove
            # this dead code.
            return False
            # # Check if there is already an object
            # for i in range(len(World.objects)):
            #     distance = World.pose_distance(
            #         World.objects[i].object.pose, pose)
            #     if distance < OBJ_ADD_DIST_THRESHHOLD:
            #         if World.objects[i].is_recognized:
            #             rospy.loginfo(
            #                 'Previously recognized object at the same ' +
            #                 'location, will not add this object.')
            #             return False
            #         else:
            #             rospy.loginfo(
            #                 'Previously unrecognized object at the same ' +
            #                 'location, will replace it with the recognized '+
            #                 'object.')
            #             to_remove = i
            #             break

            # # Remove any duplicate objects.
            # if to_remove is not None:
            #     self._remove_object(to_remove)

            # # Actually add the object.
            # self._add_new_object_internal(
            #     cluster, pose, dimensions, is_recognized, mesh)
            # return True
        else:
            # Whether whether we already have an object at ~ the same
            # location (and if so, don't add).
            for wobj in World.objects:
                if (World.pose_distance(wobj.object.pose, pose)
                        < OBJ_ADD_DIST_THRESHHOLD):
                    rospy.loginfo(
                        'Previously detected object at the same location, ' +
                        'will not add this object.')
                    return False

            # Actually add the object.
            self._add_new_object_internal(
                graspable_object, cluster, pose, dimensions, is_recognized,
                mesh)
            return True

    def _add_new_object_internal(
            self, graspable_object, cluster, pose, dimensions, is_recognized,
            mesh):
        '''Does the 'internal' adding of an object with the passed
        properties. Call _add_new_object to do all pre-requisite checks
        first (it then calls this function).

        Args:
            cluster (sensor_msgs/PointCloud)
            pose (Pose)
            dimensions (Vector3)
            is_recognized (bool)
            mesh (Mesh|None): A mesh, if it exists (can be None).
        '''
        n_objects = len(World.objects)
        World.objects.append(PbdObject(
            graspable_object, cluster, pose, n_objects, dimensions,
            is_recognized))
        int_marker = self._get_object_marker(len(World.objects) - 1)
        World.objects[-1].int_marker = int_marker
        World.im_server.insert(int_marker, self.marker_feedback_cb)
        World.im_server.applyChanges()
        World.objects[-1].menu_handler.apply(
            World.im_server, int_marker.name)
        World.im_server.applyChanges()

    def refresh_objects(self):
        '''TODO(mbforbes): Document.
        '''
        for obj in World.objects:
            World.im_server.erase(obj.int_marker.name)
        World.im_server.applyChanges()
        for obj in World.objects:
            World.im_server.insert(obj.int_marker, self.marker_feedback_cb)
            obj.menu_handler.apply(World.im_server, obj.int_marker.name)
        World.im_server.applyChanges()

    def _remove_object(self, to_remove):
        '''Remove an object by index.

        Args:
            to_remove (int): Index of the object to remove in
                World.objects.
        '''
        obj = World.objects.pop(to_remove)
        rospy.loginfo('Removing object ' + obj.int_marker.name)
        World.im_server.erase(obj.int_marker.name)
        World.im_server.applyChanges()
        # TODO(mbforbes): Re-implement object recognition or remove
        # this dead code.
        # if (obj.is_recognized):
        #     for i in range(len(World.objects)):
        #         if ((World.objects[i].is_recognized)
        #                 and World.objects[i].index > obj.index):
        #             World.objects[i].decrease_index()
        #     self.n_recognized -= 1
        # else:
        #     for i in range(len(World.objects)):
        #         if ((not World.objects[i].is_recognized) and
        #                 World.objects[i].index > obj.index):
        #             World.objects[i].decrease_index()
        #     self.n_unrecognized -= 1

    def _remove_surface(self):
        '''Function to request removing surface (from IM).'''
        rospy.loginfo('Removing surface')
        World.im_server.erase('surface')
        World.im_server.applyChanges()
        self.surface = None

    def _get_object_marker(self, index, mesh=None):
        '''Generate and return a marker for world objects.

        Args:
            index (int): ID for the new marker.
            mesh (Mesh, optional):  Mesh to use for the marker. Only
                utilized if not None. Defaults to None.

        Returns:
            InteractiveMarker
        '''
        int_marker = InteractiveMarker()
        int_marker.name = World.objects[index].get_name()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = World.objects[index].object.pose
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True

        object_marker = Marker(
            type=Marker.CUBE,
            id=index,
            lifetime=MARKER_DURATION,
            scale=World.objects[index].object.dimensions,
            header=Header(frame_id=BASE_LINK),
            color=COLOR_OBJ,
            pose=World.objects[index].object.pose
        )

        if mesh is not None:
            object_marker = World._get_mesh_marker(object_marker, mesh)
        button_control.markers.append(object_marker)

        text_pos = Point()
        text_pos.x = World.objects[index].object.pose.position.x
        text_pos.y = World.objects[index].object.pose.position.y
        text_pos.z = (
            World.objects[index].object.pose.position.z +
            World.objects[index].object.dimensions.z / 2 + OFFSET_OBJ_TEXT_Z)
        button_control.markers.append(
            Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=index,
                scale=SCALE_TEXT,
                text=int_marker.name,
                color=COLOR_TEXT,
                header=Header(frame_id=BASE_LINK),
                pose=Pose(text_pos, Quaternion(0, 0, 0, 1))
            )
        )
        int_marker.controls.append(button_control)
        return int_marker

    def _publish_tf_pose(self, pose, name, parent):
        ''' Publishes a TF for object named name with pose pose and
        parent reference frame parent.

        Args:
            pose (Pose): The object's pose.
            name (str): The object's name.
            parent (str): The parent reference frame.
        '''
        if pose is not None:
            pp = pose.position
            po = pose.orientation
            pos = (pp.x, pp.y, pp.z)
            rot = (po.x, po.y, po.z, po.w)
            # TODO(mbforbes): Is it necessary to change the position
            # and orientation into tuples to send to TF?
            self._tf_broadcaster.sendTransform(
                pos, rot, rospy.Time.now(), name, parent)

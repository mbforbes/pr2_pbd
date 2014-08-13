'''Objects for Hands-Free PbD system. Separate from PbD, living here now
for convenience.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# System builtins
from collections import OrderedDict
from threading import Lock

# ROS builtins
from geometry_msgs.msg import Quaternion, Vector3, Pose, Point

# Pbd (3rd party / local)
from pr2_pbd_interaction.msg import (
    HandsFreeCommand, WorldObject, WorldObjects, Side)

# Local
from feedback import Feedback
from robotlink import Link


# ######################################################################
# Classes
# ######################################################################

class ReachableResult(object):
    '''Saving reachable results.'''

    def __init__(
            self, reachable=False, loc_str=None, pose=None, joints=None):
        '''
        See body of method for info.

        Args:
            reachable (bool)
            loc_str (str)
            pose (Pose)
        '''

        # Is the location reachable? Type: bool
        self.reachable = reachable

        # What was the location used? (E.g. if 'near', then this could
        # be 'left' or 'right'). Only set if reachable. Type: str
        self.loc_str = loc_str

        # What was the pose used? Only set if reachable. Type: Pose.
        self.pose = pose

        # What was the joints (IK solution) for pose? Only set if
        # reachable. Type: [float] (length 7).
        self.joints = joints


class ObjectsHandler(object):
    '''Manages world objects.'''

    # Unreachable
    UNR = ReachableResult(reachable=False)

    # Orientation options to try for computing IK towards locations. We
    # use an OrderedDict so we know in which order we try them.
    orientations = OrderedDict([
        # NOTE(mbforbes): Trying all upside-down orientations for now as
        # this actually lets the grippers more easily tilt down (the
        # default, "rightside-up" (as I'm calling it) orientation is
        # biased towards tilting upwards, which is less useful for us
        # because the PR2's arms are higher up than the table, and
        # we're only concerned here with tabletop manipulation tasks).
        # ('flat-upwards', Quaternion(
        #     0.0,
        #     0.0,
        #     0.0,
        #     1.0
        # )),
        ('flat-upsidedown', Quaternion(
            1.0,
            0.0,
            0.0,
            0.0
        )),
        ('smalltilt-upsidedown', Quaternion(
            0.958600311321,
            0.0389047107548,
            -0.280663429733,
            -0.0282826064416
        )),
        ('45deg-upsidedown', Quaternion(
            0.947183721725,
            0.0378169124572,
            -0.317060828047,
            -0.029754155172
        )),
        ('largetilt-upsidedown', Quaternion(
            0.84375001925,
            0.0296645574088,
            -0.534565233277,
            -0.0380253917916
        )),
        ('vert-upsidedown', Quaternion(
            0.710535569959,
            0.0208582222416,
            -0.702007727379,
            -0.0434659532153
        )),
        ('45deg+righttilt-upsidedown', Quaternion(
            0.816555509917,
            -0.166496173679,
            -0.388895710032,
            0.392780154914
        )),
        ('45deg+right-upsidedown', Quaternion(
            0.626434852697,
            -0.291921804483,
            -0.305953683145,
            0.654792623021
        )),
        ('45deg+lefttilt-upsidedown', Quaternion(
            0.83986672291,
            0.176672428537,
            -0.384052777203,
            -0.34046175272
        )),
        ('45deg+left-upsidedown', Quaternion(
            -0.631060356859,
            -0.316931066071,
            0.279972459512,
            0.650332951091
        )),
    ])

    # TODO(mbforbes): Refactor into constant somewhere.
    topic_worldobjs = 'handsfree_worldobjects'

    # Settings
    close_delta = 0.01
    # Might have to change for sides to account for gripper size
    near_delta = 0.05

    # Vecs. Applied to border of that object (e.g. ABOVE_VEC added to
    # middle top of object, TO_LEFT_OF_VEC added to middle left side of
    # object, etc.)
    ABOVE_VEC = Vector3(0.0, 0.0, near_delta)
    TO_LEFT_OF_VEC = Vector3(0.0, near_delta, 0.0)
    TO_RIGHT_OF_VEC = Vector3(0.0, -near_delta, 0.0)
    IN_FRONT_OF_VEC = Vector3(-near_delta, 0.0, 0.0)
    BEHIND_VEC = Vector3(near_delta, 0.0, 0.0)
    ON_TOP_OF_VEC = Vector3(0.0, 0.0, close_delta)

    # Reachability mapping from pbdobj names to ROS object fields.
    rm_name_map = {
        HandsFreeCommand.ABOVE: 'is_above_reachable',
        HandsFreeCommand.TO_LEFT_OF: 'is_leftof_reachable',
        HandsFreeCommand.TO_RIGHT_OF: 'is_rightof_reachable',
        HandsFreeCommand.IN_FRONT_OF: 'is_frontof_reachable',
        HandsFreeCommand.BEHIND: 'is_behind_reachable',
        HandsFreeCommand.ON_TOP_OF: 'is_topof_reachable',
        HandsFreeCommand.NEXT_TO: 'is_nextto_reachable',
        HandsFreeCommand.NEAR: 'is_near_reachable',
    }

    # Reachability spaces. (OrderedDict as we compute in oder.)
    # Reference either a direction to apply, or one or more previous
    # elements of the reachability space as options.
    rel_positions = OrderedDict([
        (HandsFreeCommand.ABOVE, ABOVE_VEC),
        (HandsFreeCommand.TO_LEFT_OF, TO_LEFT_OF_VEC),
        (HandsFreeCommand.TO_RIGHT_OF, TO_RIGHT_OF_VEC),
        (HandsFreeCommand.IN_FRONT_OF, IN_FRONT_OF_VEC),
        (HandsFreeCommand.BEHIND, BEHIND_VEC),
        (HandsFreeCommand.ON_TOP_OF, ON_TOP_OF_VEC),
        (HandsFreeCommand.NEXT_TO, [
            HandsFreeCommand.TO_LEFT_OF,
            HandsFreeCommand.TO_RIGHT_OF,
        ]),
        (HandsFreeCommand.NEAR, [
            HandsFreeCommand.NEXT_TO,
            HandsFreeCommand.IN_FRONT_OF,
            HandsFreeCommand.BEHIND,
        ]),
    ])

    # Class variables
    world_object_pub = rospy.Publisher(topic_worldobjs, WorldObjects)
    objects = []
    objects_lock = Lock()

    @staticmethod
    def get_obj_by_name(objname):
        '''
        Args:
            objname (str)

        Returns:
            PbdObject|None: None if not found.
        '''
        for pbdobj in Link.get_objs():
            if pbdobj.name == objname:
                return pbdobj
        return None

    @staticmethod
    def record():
        '''
        Records and broadcasts world objects.
        '''
        ObjectsHandler._record_internal()
        ObjectsHandler.broadcast()

        # Feedback to user.
        n_objs = len(ObjectsHandler.objects)
        if n_objs == 0:
            objs_str = 'no objects.'
        if n_objs == 1:
            objs_str = '1 object.'
        else:
            objs_str = '%d objecs.' % (n_objs)
        Feedback('Found ' + objs_str).issue()


    # Reachabilities don't change, so probably no need for this.
    # @staticmethod
    # def async_update():
    #     '''
    #     Updates the existing world objects by assuming they haven't
    #     changed and computing reachabilities. Also broadcasts.

    #     Non-blocking.
    #     '''
    #     Thread(
    #         group=None,
    #         target=ObjectsHandler.update,
    #         name='update_world_objects_thread'
    #     ).start()

    # Reachabilities don't change, so probably no need for this.
    # @staticmethod
    # def update():
    #     '''
    #     Updates the existing world objects by assuming they haven't
    #     changed and computing reachabilities. Also broadcasts.

    #     Blocking.
    #     '''
    #     # TODO(mbforbes): Implement.
    #     # - compute properties of existing objects
    #     ObjectsHandler._update_internal()
    #     ObjectsHandler.broadcast()

    @staticmethod
    def _record_internal():
        '''
        Moves hands to side, gets the world objects (actually observes).

        Note that it doesn't move the hands back. This is useful because
        now we start programming from the same location every time.
        '''
        # Move arms to side.
        Link.move_to_named_position('side')

        # Record and grab objects
        Link.update_object_pose()
        ObjectsHandler.objects_lock.acquire()
        ObjectsHandler.objects = Link.get_objs()

        # Compute properties
        # NOTE(mbforbes); Originally, we had _update_internal() here.
        for pbd_obj in ObjectsHandler.objects:
            ObjectsHandler._compute_reachability_for(pbd_obj)
        Link.refresh_objects()

        # TODO(mbforbes): Compute global properties (which farthest,
        # etc.)

        ObjectsHandler.objects_lock.release()

    @staticmethod
    def _compute_reachability_for(pbd_obj):
        '''
        Args:
            pbd_obj (PbdObject)
        '''
        pbd_obj.reachability_map = {}
        for pos, val in ObjectsHandler.rel_positions.iteritems():
            # Set defaults to unreachable.
            pbd_obj.reachability_map[pos] = [
                ObjectsHandler.UNR,  # r
                ObjectsHandler.UNR,  # l
            ]
            for side in [Side.RIGHT, Side.LEFT]:
                if type(val) is list:
                    # It's referencing previously-computed locations. If
                    # any match, it's good.
                    for str_loc in val:
                        rr = pbd_obj.reachability_map[str_loc][side]
                        if rr.reachable:
                            pbd_obj.reachability_map[pos][side] = rr
                            break
                else:
                    # It's referencing a vector directly. Compute.
                    pbd_obj.reachability_map[pos][side] = (
                        ObjectsHandler._get_loc_reachable(
                            pbd_obj, pos, val, side))

    # Reachabilities don't change, so probably no need for this.
    # @staticmethod
    # def _update_internal():
    #     '''
    #     Updates reachability properties of existing WorldObjects.
    #     '''
    #     ObjectsHandler.objects_lock.acquire()
    #     for pbd_object in ObjectsHandler.objects:
    #         # TODO(mbforbes): compute properties.
    #         pass
    #     ObjectsHandler.objects_lock.release()

    @staticmethod
    def _get_loc_reachable(pbd_obj, rel_pos_str, rel_pos_vec, side):
        '''
        Tries several hand orientations to reach rel_pos_vec of pbd_obj
        with side hand. Returns reachable result containing whether any
        are possible.

        Args:
            pbd_obj (PbdObject)
            rel_pos_str (str): One of
                - HandsFreeCommand.ABOVE
                - HandsFreeCommand.TO_LEFT_OF
                - ...
            rel_pos_vec (Vector3): One of
                - ObjectsHandler.ABOVE_VEC
                - ObjectsHandler.TO_LEFT_OF_VEC
                - ...
            side (int): Side.RIGHT or Side.LEFT


        Returns:
            ReachableResult
        '''
        # Get the required position.
        position = ObjectsHandler._get_position_for(
            pbd_obj, rel_pos_str, rel_pos_vec)

        # Try a bunch of orientations.
        res = ObjectsHandler.UNR
        for o_name, orientation in ObjectsHandler.orientations.iteritems():
            pose = Pose(position, orientation)
            joints = Link.get_ik_for_ee(side, pose, [0.0] * 7)
            if joints is not None:
                # Hooray!
                res = ReachableResult(
                    True,
                    rel_pos_str,
                    pose,
                    joints
                )

        # Display in visualization and return.
        pbd_obj.add_reachable_marker(rel_pos_str, position, res.reachable)
        return res

    @staticmethod
    def _get_position_for(pbd_obj, rel_pos_str, rel_pos_vec):
        '''
        Returns the location (x, y, z) that is requested for the pbd_obj
        by rel_pos_vec.

        Note that this involves
         - computing where on the object to apply rel_pos_vec
         - applying rel_pos_vec to that point

        Args:
            pbd_obj (PbdObject)
            rel_pos_str (str): One of
                - HandsFreeCommand.ABOVE
                - HandsFreeCommand.TO_LEFT_OF
                - ...
            rel_pos_vec (Vector3): One of
                - ObjectsHandler.ABOVE_VEC
                - ObjectsHandler.TO_LEFT_OF_VEC
                - ...

        Returns:
            Point
        '''
        # Find the starting position based on the type of vector.
        op = pbd_obj.pose.position
        loc = Point(op.x, op.y, op.z)  # Don't want to modify obj pos.
        endpoints = pbd_obj.endpoints
        if rel_pos_str in [HandsFreeCommand.ABOVE, HandsFreeCommand.ON_TOP_OF]:
            loc.z = endpoints[4]
        elif rel_pos_str == HandsFreeCommand.TO_LEFT_OF:
            loc.y = endpoints[1]
        elif rel_pos_str == HandsFreeCommand.TO_RIGHT_OF:
            loc.y = endpoints[0]
        elif rel_pos_str == HandsFreeCommand.IN_FRONT_OF:
            loc.x = endpoints[3]
        elif rel_pos_str == HandsFreeCommand.BEHIND:
            loc.x = endpoints[2]
        else:
            rospy.logwarn(
                'Relative position ' + rel_pos_str + ' unimplemented; using ' +
                'center of object ' + str(pbd_obj) + ' as starting point.')

        # Now add the vector to get the final position.
        return Point(
            loc.x + rel_pos_vec.x,
            loc.y + rel_pos_vec.y,
            loc.z + rel_pos_vec.z,
        )

    @staticmethod
    def make_worldobjs():
        '''
        Uses ObjectsHandler.objects to make a WorldObjects.

        Returns:
            WorldObjects
        '''
        wobjs = []
        for pbd_obj in ObjectsHandler.objects:
            wo = WorldObject()

            # Basic properties
            wo.name = pbd_obj.name
            wo.color = pbd_obj.color
            wo.type = pbd_obj.type

            # Reachability
            rm = pbd_obj.reachability_map
            for pbdname, rosname in ObjectsHandler.rm_name_map.iteritems():
                bool_arr = [rr.reachable for rr in rm[pbdname]]
                setattr(wo, rosname, bool_arr)

            # TODO(mbforbes): Add multi-object properties.

            # Slap it on.
            wobjs += [wo]

        return WorldObjects(wobjs)

    @staticmethod
    def broadcast():
        '''
        Actually publishes the WorldObjects.
        '''
        ObjectsHandler.objects_lock.acquire()
        ObjectsHandler.world_object_pub.publish(
            ObjectsHandler.make_worldobjs())
        ObjectsHandler.objects_lock.release()

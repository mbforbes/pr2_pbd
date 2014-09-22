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
import operator
from threading import Lock

# ROS builtins
from geometry_msgs.msg import Vector3, Pose, Point

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

    def __init__(self, reachable=False, loc_str=None, pose=None):
        '''
        See body of method for info.

        NOTE(mbforbes): Previously had joints ([float]) cached here, but
        this is implementation-specific (i.e. using IK).

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


class UniqueProperty(object):
    '''For tracking biggest/smallest/shortest/leftmost/etc.'''

    # These are starting values that are supposted to be so 'absurdly'
    # low / high that any object's property will be > / < these values.
    low_start = -1000
    high_start = 1000

    def __init__(self, name, prop_name, val, attr_get_fn, cmp_op):
        '''
        Args:
            name (str): Internal use.
            prop_name (str): For setting object property.
            val (float): The current max/min val of this property.
            attr_get_fn (function): The function to get the desired
                attribute from an object for comparison to val.
            cmp_op (function): The operator function to determine
                whether "beats" the current val and becomes special.
                Appled as cmp_op(obj_val, val).
        '''
        self.name = name
        self.prop_name = prop_name
        self.val = val
        self.orig_val = val  # For resetting.
        self.attr_get_fn = attr_get_fn
        self.cmp_op = cmp_op
        self.special = None

    def check(self, pbd_obj):
        '''
        Checks pbd_obj to see if it "beats" val and gets to be special.

        Args:
            pbd_obj (PbdObject)
        '''
        obj_val = self.attr_get_fn(pbd_obj)
        if self.cmp_op(obj_val, self.val):
            self.val = obj_val
            self.special = pbd_obj

    def reset(self):
        '''
        Resets the unique property so that it can be computed anew.
        '''
        self.val = self.orig_val
        self.special = None


class ObjectsHandler(object):
    '''Manages:
        - PbdObject's (though they live in World)
        - WorldObject's (they are created for returning to parser)
        - WorldObjects (it is created for returning to parser)
    '''

    # Unreachable
    UNR = ReachableResult(reachable=False)

    # Unite properties.
    unique_properties = [
        UniqueProperty(
            'rightmost',
            'is_rightmost',
            UniqueProperty.high_start,
            lambda o: o.endpoints[0],
            operator.lt
        ),
        UniqueProperty(
            'leftmost',
            'is_leftmost',
            UniqueProperty.low_start,
            lambda o: o.endpoints[1],
            operator.gt
        ),
        UniqueProperty(
            'farthest',
            'is_farthest',
            UniqueProperty.low_start,
            lambda o: o.endpoints[2],
            operator.gt
        ),
        UniqueProperty(
            'nearest',
            'is_nearest',
            UniqueProperty.high_start,
            lambda o: o.endpoints[3],
            operator.lt
        ),
        UniqueProperty(
            'tallest',
            'is_tallest',
            UniqueProperty.low_start,
            lambda o: o.endpoints[4],
            operator.gt
        ),
        UniqueProperty(
            'shortest',
            'is_shortest',
            UniqueProperty.high_start,
            lambda o: o.endpoints[5],
            operator.lt
        ),
        UniqueProperty(
            'biggest',
            'is_biggest',
            UniqueProperty.low_start,
            lambda o: o.vol,
            operator.gt
        ),
        UniqueProperty(
            'smallest',
            'is_smallest',
            UniqueProperty.high_start,
            lambda o: o.vol,
            operator.lt
        ),
    ]

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
    def get_reachable_with_cur_orient(pbd_obj, pos, side):
        '''
        Returns whether pos of pbd_obj is reachable with the current
        orientation of side hand.

        Args:
            pbd_obj (PbdObject): The object to be relative to.
            pos (str): The string describing how to be relative
                to pbd_obj (e.g. HandsFreeCommand.[...]).
            side (int): Side.LEFT or Side.RIGHT

        Returns:
            ReachableResult
        '''
        return ObjectsHandler._get_reachability_for(
            pbd_obj,
            pos,
            side,
            Link.get_cur_orient(side)
        )

    @staticmethod
    def record():
        '''
        Records and broadcasts world objects.
        '''
        ObjectsHandler._clear()
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

    @staticmethod
    def _clear():
        '''
        Clears all objects.
        '''
        ObjectsHandler.objects_lock.acquire()
        ObjectsHandler.objects = []
        Link.clear_objects()
        ObjectsHandler.objects_lock.release()

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
        Link.move_to_named_position('to_side')

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
        Computes and saves (in reachability_map) reachabilities for
        pbd_obj of all positions in rel_positions.

        Args:
            pbd_obj (PbdObject)
        '''
        pbd_obj.reachability_map = {}
        # rel_positions is { str: Vector3 | [str] }
        for pos in ObjectsHandler.rel_positions.keys():
            # Make space.
            pbd_obj.reachability_map[pos] = [None, None]
            for side in [Side.RIGHT, Side.LEFT]:
                pbd_obj.reachability_map[pos][side] = (
                    ObjectsHandler._get_reachability_for(pbd_obj, pos, side))

    @staticmethod
    def _get_reachability_for(pbd_obj, pos, side, orient=None):
        '''
        Gets (and doesn't save) the reachability for pbd_obj at pos
        relative to it for arm on side. Will use orient if provided.

        Args:
            pbd_obj (PbdObject)
            pos (str): Relative position string
            side (int): Side.RIGHT or Side.LEFT
            orient (Quaternion, optional): The orientation to use.
                Defaults to None, in which case the perscribed options
                are tested.
        '''
        # rel_positions is { str: Vector3 | [str] }
        val = ObjectsHandler.rel_positions[pos]
        res = ObjectsHandler.UNR  # Default.
        if type(val) is list:
            # It's referencing other locations. If any work, it's good.
            for str_loc in val:
                # Recurse. Not caching because we don't save the
                # orientation when computing.
                rr = ObjectsHandler._get_reachability_for(
                    pbd_obj, str_loc, side)
                if rr.reachable:
                    res = rr
                    break
        else:
            # It's referencing a vector directly. Compute.
            res = ObjectsHandler._get_loc_reachable(
                pbd_obj, pos, val, side, orient)
        return res

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
    def _get_loc_reachable(
            pbd_obj, rel_pos_str, rel_pos_vec, side, orient=None):
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
            orient (Quaternion, optional): The orientation to use.
                Defaults to None, in which case the perscribed options
                are tested.

        Returns:
            ReachableResult
        '''
        # Get the required position.
        position = ObjectsHandler._get_position_for(
            pbd_obj, rel_pos_str, rel_pos_vec)

        # Try a bunch of orientations if none specified.
        if orient is None:
            prefix = Link.orientation_prefix_map[rel_pos_str]
            opts = [
                o for n, o in Link.orientations.iteritems()
                if n.startswith(prefix)
            ]
        else:
            # Orientation was specified; use it.
            opts = [orient]

        # Do the tests.
        res = ObjectsHandler.UNR  # Default.
        for orientation in opts:
            pose = Pose(position, orientation)
            if Link.get_computed_pose_possible(side, pose):
                # Hooray!
                res = ReachableResult(
                    True,
                    rel_pos_str,
                    pose,
                )
                break

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

        # Compute unique properties (if more than one obj).
        if len(ObjectsHandler.objects) > 1:
            for pbd_obj in ObjectsHandler.objects:
                for prop in ObjectsHandler.unique_properties:
                    prop.check(pbd_obj)

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

            # Add any unique properties.
            for prop in ObjectsHandler.unique_properties:
                if prop.special == pbd_obj:
                    setattr(wo, prop.prop_name, True)

            # Slap it on.
            wobjs += [wo]

        # Reset the unique properties themselves so they can be computed
        # again next time.
        for prop in ObjectsHandler.unique_properties:
            prop.reset()

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

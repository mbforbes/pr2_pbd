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
from pr2_pbd_interaction.srv import (
    WorldChange, WorldChangeRequest, WorldChangeResponse)
from util import Logger


# ######################################################################
# Module-level constants
# ######################################################################

# Saliency delta threshholds; objects' properties must differ by at
# least these (respectively) to be assiend a unique identifier.
# For more info: https://docs.google.com/document/d/
#     1tHmfctkPF-oPROQNBxMEwaHfqvHGVxkh7OnUx28yO0g/edit
VOL_DELTA = 0.0005
HEIGHT_DELTA = 0.1
DIST_DELTA = 0.1


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
    '''For tracking biggest/smallest/shortest/leftmost/etc. If a
    property isn't salient enough, not object will receive it.

    Process for using:
        - create the UniqueProperty
        - call check(...) with all PbdObjects
        - call get_special() to see if any PbdObject gets the property
        - call reset() to reset cached values and use again
    '''

    # These are starting values that are supposted to be so 'absurdly'
    # low / high that any object's property will be > / < these values.
    low_start = -1000
    high_start = 1000

    def __init__(self, name, prop_name, val, attr_get_fn, cmp_op, delta):
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
            delta (float): The delta needed between the 'best' val and
                the 'second-best' val in order to allow an object to
                receive this property.
        '''
        self.name = name
        self.prop_name = prop_name
        self.val = val
        self.second_val = val  # For checking saliency.
        self.orig_val = val  # For resetting.
        self.attr_get_fn = attr_get_fn
        self.cmp_op = cmp_op
        self.special = None
        self.delta = delta

    def check(self, pbd_obj):
        '''
        Checks pbd_obj to see if it "beats" val and gets to be special.

        Args:
            pbd_obj (PbdObject)
        '''
        obj_val = self.attr_get_fn(pbd_obj)
        if self.cmp_op(obj_val, self.val):
            # obj_val wins; shift self.val to second.
            self.second_val = self.val
            self.val = obj_val
            self.special = pbd_obj
        elif self.cmp_op(obj_val, self.second_val):
            # obj_val second
            self.second_val = obj_val

    def get_special(self):
        '''
        Returns a PbdObject if it is both:
            - the 'best' property holder for this unique property
            - and salient enough in its 'best-ness'
        Returns None if no such object meets this critera.

        Returns:
            PbdObject | None
        '''
        return (
            self.special if abs(self.val - self.second_val) >= self.delta
            else None
        )

    def reset(self):
        '''
        Resets the unique property so that it can be computed anew.
        '''
        self.val = self.orig_val
        self.second_val = self.orig_val
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
            operator.lt,
            DIST_DELTA
        ),
        UniqueProperty(
            'leftmost',
            'is_leftmost',
            UniqueProperty.low_start,
            lambda o: o.endpoints[1],
            operator.gt,
            DIST_DELTA
        ),
        UniqueProperty(
            'farthest',
            'is_farthest',
            UniqueProperty.low_start,
            lambda o: o.endpoints[2],
            operator.gt,
            DIST_DELTA
        ),
        UniqueProperty(
            'nearest',
            'is_nearest',
            UniqueProperty.high_start,
            lambda o: o.endpoints[3],
            operator.lt,
            DIST_DELTA
        ),
        UniqueProperty(
            'tallest',
            'is_tallest',
            UniqueProperty.low_start,
            lambda o: o.endpoints[4],
            operator.gt,
            HEIGHT_DELTA
        ),
        UniqueProperty(
            'shortest',
            'is_shortest',
            UniqueProperty.high_start,
            lambda o: o.endpoints[4],  # Not a bug; use 4 for height.
            operator.lt,
            HEIGHT_DELTA
        ),
        UniqueProperty(
            'biggest',
            'is_biggest',
            UniqueProperty.low_start,
            lambda o: o.vol,
            operator.gt,
            VOL_DELTA
        ),
        UniqueProperty(
            'smallest',
            'is_smallest',
            UniqueProperty.high_start,
            lambda o: o.vol,
            operator.lt,
            VOL_DELTA
        ),
    ]

    # Settings
    close_delta = 0.05  # Try to take into account object size.
    # Might have to change for sides to account for gripper size
    near_delta = 0.05
    # This is all kind of wrong because we're not taking into account
    # the object we're holding, which matters a great deal. This is to
    # get clearance.
    above_delta = 0.10

    # Vecs. Applied to border of that object (e.g. ABOVE_VEC added to
    # middle top of object, TO_LEFT_OF_VEC added to middle left side of
    # object, etc.)
    ABOVE_VEC = Vector3(0.0, 0.0, above_delta)
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
    world_change_srv = rospy.ServiceProxy(
        'handsfree_worldchange', WorldChange)
    objects = []
    objects_lock = Lock()
    descs = {}

    @staticmethod
    def get_obj_by_name(objname):
        '''
        Args:
            objname (str)

        Returns:
            PbdObject|None: None if not found.
        '''
        objs = Link.get_objs()
        for pbdobj in objs:
            if pbdobj.name == objname:
                return pbdobj

        # This is actually a bad problem if it doesn't exist.
        rospy.logerr("Couldn't find object " + objname)
        rospy.logerr("Objects I have: " + [o.name for o in objs])
        return None

    @staticmethod
    def get_obj_by_spec(obj_spec):
        '''
        Finds the object that most closely matches the specification of
        obj_spec, or None if no such object exists.

        Args:
            obj_spec (ObjectSpec): The specification of the object that
                we're looking for.

        Returns:
            PbdObject|None: None if not found.
        '''
        # If no objects, return None.
        objs = Link.get_objs()
        if len(objs) == 0:
            return None
        # Compute all the scores, return the obj that scored the lowest.
        scores = [obj_spec.score(s) for s in [o.get_spec() for o in objs]]
        return objs[scores.index(min(scores))]

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
    def get_description_for(pbd_obj):
        '''
        Gets the latest description for pbd_obj if we have it.

        Args:
            pbd_obj (PbdObject)

        Returns:
            str
        '''
        rospy.loginfo("Object map: " + str(ObjectsHandler.descs))
        if pbd_obj is None:
            return 'the object'
        if pbd_obj.name not in ObjectsHandler.descs:
            return pbd_obj.name
        return ObjectsHandler.descs[pbd_obj.name]

    @staticmethod
    def save_descriptions(names, descs):
        '''
        Recrods received object descriptions. The arrays are aligned.

        Args:
            names ([str]): Names of objects.
            descs ([str]): Their descriptions.
        '''
        mapping = {}
        for i in range(len(names)):
            mapping[names[i]] = descs[i]
        ObjectsHandler.descs = mapping

    @staticmethod
    def record():
        '''
        Records and broadcasts world objects.
        '''
        ObjectsHandler._clear()
        ObjectsHandler._record_internal()
        ObjectsHandler.change_world()

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
        ObjectsHandler.descs = {}
        ObjectsHandler.objects_lock.release()

    @staticmethod
    def _record_internal():
        '''
        Moves hands to side, gets the world objects (actually observes).

        Note that it doesn't move the hands back to where they were when
        this was called. This is useful because now we start programming
        from the same location every time.
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

        # Set properties for objects.
        for pbd_obj in ObjectsHandler.objects:
            wo = WorldObject()

            # Basic properties
            wo.name = pbd_obj.name
            wo.color = pbd_obj.color
            wo.type = pbd_obj.shape

            # Reachability
            rm = pbd_obj.reachability_map
            for pbdname, rosname in ObjectsHandler.rm_name_map.iteritems():
                bool_arr = [rr.reachable for rr in rm[pbdname]]
                setattr(wo, rosname, bool_arr)

            # Add any unique properties.
            for prop in ObjectsHandler.unique_properties:
                if prop.get_special() == pbd_obj:
                    setattr(wo, prop.prop_name, True)

            # Special unique propery: middle (is_middle). Conditions:
            # (1) must have three objects
            # (2) can't be l/r-most or farthest/nearest
            # (3) two other objects must have l/r-most OR
            #     farthest/nearest
            #
            # (1) Must have 3 objs.
            if len(ObjectsHandler.objects) == 3:
                up = ObjectsHandler.unique_properties
                lprop = [p for p in up if p.name == 'leftmost'][0]
                lobj = lprop.get_special()
                rprop = [p for p in up if p.name == 'rightmost'][0]
                robj = rprop.get_special()
                fprop = [p for p in up if p.name == 'farthest'][0]
                fobj = fprop.get_special()
                nprop = [p for p in up if p.name == 'nearest'][0]
                nobj = nprop.get_special()
                # (2) Can't be l/r-most or farthest/nearest
                if (lobj != pbd_obj and
                        robj != pbd_obj and
                        fobj != pbd_obj and
                        nobj != pbd_obj):
                    # (3) two other objects must have l/r-most OR
                    # farthest/nearest
                    if ((lobj is not None and
                            robj is not None and
                            lobj != robj) or (
                            fobj is not None and
                            nobj is not None and
                            fobj != nobj)):
                        setattr(wo, 'is_middle', True)

            # Slap it on.
            wobjs += [wo]

        # Reset the unique properties themselves so they can be computed
        # again next time.
        for prop in ObjectsHandler.unique_properties:
            prop.reset()

        return WorldObjects(wobjs)

    @staticmethod
    def change_world():
        '''
        Calls the WorldChange service to give the WorldObjects to the
        parser and get a Description in return.
        '''
        ObjectsHandler.objects_lock.acquire()
        try:
            # Call the parser as a service, extract description.
            resp = ObjectsHandler.world_change_srv(
                WorldChangeRequest(ObjectsHandler.make_worldobjs()))
            desc = resp.desc

            # Log the description.
            Logger.L.save_desc(desc)

            # Save it for use internally.
            names, descs = desc.object_names, desc.descriptions
            ObjectsHandler.save_descriptions(names, descs)
        except rospy.ServiceException, e:
            rospy.logwarn("World Change service call failed: " + str(e))
        ObjectsHandler.objects_lock.release()

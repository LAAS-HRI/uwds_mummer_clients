#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import uuid
import math
import numpy
import rospy
from std_msgs.msg import String
import message_filters
import uuid
from tf.transformations import *
from uwds_msgs.msg import Changes, Node, Situation, Property
from std_msgs.msg import Int32
from pyuwds.types.nodes import CAMERA
from pyuwds.types.situations import FACT, ACTION
from pyuwds.uwds import PROVIDER
from pyuwds.uwds_client import UwdsClient
from perception_msgs.msg import GazeInfoArray, VoiceActivityArray, TrackedPersonArray
from geometry_msgs.msg import PointStamped, Point, TransformStamped

import tf2_ros


DEFAULT_CLIP_PLANE_NEAR = 0.2
DEFAULT_CLIP_PLANE_FAR = 1000.0
DEFAULT_HORIZONTAL_FOV = 60.0
DEFAULT_ASPECT = 1.33333

LOOK_AT_THRESHOLD = 0.4
ENGAGING_DISTANCE = 2.0
ENGAGING_MIN_TIME = 2.5

NB_MIN_DETECTION = 9
MAX_DIST = 2.5

CLOSE_MAX_DISTANCE = 1.0
NEAR_MAX_DISTANCE = 2.0

PERSON_PERSISTENCE = 10.0

MIN_ENGAGEMENT_DURATION = 6.0


def transformation_matrix(t, q):
    translation_mat = translation_matrix(t)
    rotation_mat = quaternion_matrix(q)
    return numpy.dot(translation_mat, rotation_mat)

class MultiModalHumanProvider(UwdsClient):
    def __init__(self):
        super(MultiModalHumanProvider, self).__init__("multimodal_human_provider", PROVIDER)
        self.distances = {}
        self.persons = {}
        self.footprints = {}
        self.predicates_by_id = {}
        self.perceived_ids = []
        self.alternate_id_map = {}
        self.last_speaking_id = ""
        self.nb_detection = {}

        self.previously_perceived_ids = []
        self.previously_near_ids = []
        self.previously_close_ids = []
        self.previously_speaking_ids = []
        self.previously_looking_at = {}
        self.previously_speaking_to = {}

        self.previously_engaging_with = {}
        self.previously_engaged_with = {}
        self.last_engagement_time = {}

        self.world = rospy.get_param("~output_world", "robot/humans")

        self.look_at_threshold = rospy.get_param("uwds_mummer/look_at_threshold", LOOK_AT_THRESHOLD)
        self.engaging_distance = rospy.get_param("uwds_mummer/engaging_distance", ENGAGING_DISTANCE)
        self.engaging_min_time = rospy.get_param("uwds_mummer/engaging_min_time", ENGAGING_MIN_TIME)
        self.nb_min_detection = rospy.get_param("uwds_mummer/nb_min_detection", NB_MIN_DETECTION)
        self.max_dist = rospy.get_param("uwds_mummer/max_dist", MAX_DIST)
        self.close_max_distance = rospy.get_param("uwds_mummer/close_max_distance", CLOSE_MAX_DISTANCE)
        self.near_max_distance = rospy.get_param("uwds_mummer/near_max_distance", NEAR_MAX_DISTANCE)
        self.min_engagement_duration = rospy.get_param("uwds_mummer/min_engagement_duration", MIN_ENGAGEMENT_DURATION)
        self.person_persistence = rospy.get_param("uwds_mummer/person_persistence", PERSON_PERSISTENCE)

        # self.person_of_interest_sub = rospy.Subscriber("person_of_interest", Int32, self.handle_person)

        # self.person_of_interest = None
        self.last_update_person = None

        self.predicates_map = {}
        self.human_distances = {}

        self.tfBuffer = tf2_ros.Buffer()
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pepper_id = str(uuid.uuid4())

        self.ros_sub = {"gaze_tracker": message_filters.Subscriber("wp2/gaze", GazeInfoArray),
                        "voice_tracker": message_filters.Subscriber("wp2/voice", VoiceActivityArray),
                        "person_tracker": message_filters.Subscriber("wp2/track", TrackedPersonArray),
                        "speech_recognition": rospy.Subscriber("speech_recognition", String, self.callback_speech)}

        self.ts = message_filters.TimeSynchronizer([self.ros_sub["gaze_tracker"], self.ros_sub["voice_tracker"], self.ros_sub["person_tracker"]], 50)
        self.ts.registerCallback(self.callback)

    # def handle_person(self, msg):
    #     self.person_of_interest = str(msg.data)
    #     self.last_update_person = rospy.Time.now()

    def callback(self, gaze_msg, voice_msg, person_msg):
        # self.look_at_threshold = rospy.get_param("look_at_threshold", LOOK_AT_THRESHOLD)
        # self.engaging_distance = rospy.get_param("engaging_distance", ENGAGING_DISTANCE)
        # self.engaging_min_time = rospy.get_param("engaging_min_time", ENGAGING_MIN_TIME)
        # self.nb_min_detection = rospy.get_param("nb_min_detection", NB_MIN_DETECTION)
        # self.max_dist = rospy.get_param("max_dist", MAX_DIST)
        # self.close_max_distance = rospy.get_param("close_max_distance", CLOSE_MAX_DISTANCE)
        # self.near_max_distance = rospy.get_param("near_max_distance", NEAR_MAX_DISTANCE)
        # self.min_engagement_duration = rospy.get_param("min_engagement_duration", MIN_ENGAGEMENT_DURATION)
        # self.person_persistence = rospy.get_param("person_persistence", PERSON_PERSISTENCE)
        #print "perception callback called"

        self.changes = Changes()

        now = rospy.Time.now()

        gaze_attention_point = None
        voice_attention_point = None
        min_dist = 10000

        self.currently_perceived_ids = []
        self.currently_near_ids = []

        self.currently_close_ids = []
        self.currently_speaking_ids = []
        self.currently_looking_at = {}
        self.currently_speaking_to = {}

        self.start_engaging_time = {}

        self.currently_engaging_with = {}
        self.currently_engaged_with = {}

        min_dist = float("inf")
        min_id_0 = None
        i = 0
        min_it = None
        for person in person_msg.data:
            if person.head_distance < self.engaging_distance:
                if person.head_distance < min_dist:
                    min_dist = person.head_distance
                    min_id_0 = person.person_id
                    min_it = i
            i += 1
        if min_id_0 is not None:
            person_msg.data[min_it].person_id = 0

            for gaze in gaze_msg.data:
                if gaze.person_id == min_id_0:
                    gaze.person_id = 0

        if len(person_msg.data) > 0:
            for person in person_msg.data:
                if str(person.person_id) not in self.nb_detection:
                    self.nb_detection[str(person.person_id)] = 0
                else:
                    self.nb_detection[str(person.person_id)] += 1

                min_id = person.person_id
                for id in person.alternate_ids:
                    if id < min_id:
                        min_id = id
                self.alternate_id_map[str(person.person_id)] = str(min_id)

                if self.nb_detection[str(person.person_id)] > self.nb_min_detection:
                    if person.head_distance < self.max_dist:
                        if str(person.person_id) not in self.persons:
                            new_node = Node()
                            new_node.name = "human-"+str(person.person_id)
                            new_node.id = str(person.person_id)
                            new_node.type = CAMERA
                            clipnear_property = Property(name="clipnear", data=str(DEFAULT_CLIP_PLANE_NEAR))
                            clipfar_property = Property(name="clipfar", data=str(DEFAULT_CLIP_PLANE_FAR))
                            hfov_property = Property(name="hfov", data=str(DEFAULT_HORIZONTAL_FOV))
                            aspect_property = Property(name="aspect", data=str(DEFAULT_ASPECT))
                            class_property = Property(name="class", data="Human")
                            new_node.properties.append(clipnear_property)
                            new_node.properties.append(clipfar_property)
                            new_node.properties.append(hfov_property)
                            new_node.properties.append(aspect_property)
                            new_node.properties.append(class_property)
                            self.persons[str(person.person_id)] = new_node
                            self.changes.nodes_to_update.append(self.persons[str(person.person_id)])
                            #self.predicates_map[str(person.person_id)] = []
                        self.currently_perceived_ids.append(str(person.person_id))
                        self.human_distances[str(person.person_id)] = person.head_distance

                        t = [person.head_pose.position.x, person.head_pose.position.y, person.head_pose.position.z]
                        q = [person.head_pose.orientation.x, person.head_pose.orientation.y, person.head_pose.orientation.z, person.head_pose.orientation.w]

                        offset = euler_matrix(0, math.radians(90), math.radians(90), "rxyz")

                        transform = numpy.dot(transformation_matrix(t, q), offset)

                        position = translation_from_matrix(transform)
                        quaternion = quaternion_from_matrix(transform)

                        if self.ctx.worlds()[self.world].scene().nodes().has(str(person.person_id)):
                            try :
                                delta_x = self.persons[str(person.person_id)].position.pose.position.x - position[0]
                                delta_y = self.persons[str(person.person_id)].position.pose.position.y - position[1]
                                delta_z = self.persons[str(person.person_id)].position.pose.position.z - position[2]
                                delta_t = person_msg.header.stamp - self.persons[str(person.person_id)].last_observation.data

                                self.persons[str(person.person_id)].velocity.position.x = delta_x / delta_t
                                self.persons[str(person.person_id)].velocity.position.y = delta_y / delta_t
                                self.persons[str(person.person_id)].velocity.position.z = delta_z / delta_t
                            except Exception:
                                pass

                        self.persons[str(person.person_id)].position.pose.position.x = position[0]
                        self.persons[str(person.person_id)].position.pose.position.y = position[1]
                        self.persons[str(person.person_id)].position.pose.position.z = position[2]

                        self.persons[str(person.person_id)].position.pose.orientation.x = quaternion[0]
                        self.persons[str(person.person_id)].position.pose.orientation.y = quaternion[1]
                        self.persons[str(person.person_id)].position.pose.orientation.z = quaternion[2]
                        self.persons[str(person.person_id)].position.pose.orientation.w = quaternion[3]

                        self.persons[str(person.person_id)].last_observation.data = person_msg.header.stamp
                        self.changes.nodes_to_update.append(self.persons[str(person.person_id)])
                        transform = TransformStamped()
                        transform.header = person_msg.header
                        transform.transform.translation.x = position[0]
                        transform.transform.translation.y = position[1]
                        transform.transform.translation.z = position[2]
                        transform.transform.rotation.x = quaternion[0]
                        transform.transform.rotation.y = quaternion[1]
                        transform.transform.rotation.z = quaternion[2]
                        transform.transform.rotation.w = quaternion[3]
                        transform.child_frame_id = "gaze_human_"+str(person.person_id)
                        self.tfBroadcaster.sendTransform(transform)

            # min_dist = 10000
            # min_id = None
            # for voice in voice_msg.data:
            #     if str(voice.person_id) in self.persons:
            #         if voice.is_speaking:
            #             self.currently_speaking_ids.append(str(voice.person_id))
            #             if str(voice.person_id) in self.distances:
            #                 if self.human_distances[str(voice.person_id)] < min_dist:
            #                     min_dist = self.human_distances[str(voice.person_id)]
            #                     min_id = voice.person_id
            # if min_id is not None:
            #     voice_attention_point = PointStamped()
            #     voice_attention_point.header.frame_id = str(min_id)
            #     voice_attention_point.point = Point(0, 0, 0)

            for gaze in gaze_msg.data:
                if str(gaze.person_id) in self.persons:
                    if self.human_distances[str(gaze.person_id)] < self.engaging_distance:

                        if str(gaze.person_id) not in self.last_engagement_time:
                            self.last_engagement_time[str(gaze.person_id)] = {}

                        if gaze.probability_looking_at_robot > self.look_at_threshold:
                            self.currently_looking_at[str(gaze.person_id)] = "robot"
                            self.last_engagement_time[str(gaze.person_id)]["robot"] = now
                            self.currently_engaging_with[str(gaze.person_id)] = "robot"
                        else:
                            if gaze.probability_looking_at_screen > self.look_at_threshold:
                                self.currently_looking_at[str(gaze.person_id)] = "robot"
                                self.last_engagement_time[str(gaze.person_id)]["robot"] = now
                                self.currently_engaging_with[str(gaze.person_id)] = "robot"
                            else:
                                for attention in gaze.attentions:
                                    if attention.target_id in self.persons:
                                        if attention.probability_looking_at_target > self.look_at_threshold:
                                            if str(attention.target_id) in self.alternate_id_map:
                                                self.currently_looking_at[str(gaze.person_id)] = self.alternate_id_map[str(attention.target_id)]
                                                self.currently_engaging_with[str(gaze.person_id)] = self.alternate_id_map[str(attention.target_id)]
                                                self.last_engagement_time[str(gaze.person_id)][self.alternate_id_map[str(attention.target_id)]] = now

            for person_id in self.last_engagement_time:
                for person2_id in self.last_engagement_time[person_id]:
                    if person_id+"isEngagingWith"+person2_id in self.predicates_map:
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[person_id+"isEngagingWith"+person2_id]]
                        if fact is not None:
                            if (now - fact.start.data).to_sec() > self.engaging_min_time:
                                self.currently_engaged_with[person_id] = person2_id
                    else:
                        if person_id+"isEngagedWith"+person2_id in self.predicates_map:
                            if (now - self.last_engagement_time[person_id][person2_id]).to_sec() < self.min_engagement_duration:
                                self.currently_engaged_with[person_id] = person2_id


            # min_dist = 10000
            # min_id = None
            # for id, dist in self.human_distances.items():
            #     if id in self.currently_perceived_ids:
            #         if dist < NEAR_MAX_DISTANCE:
            #             if min_dist > dist:
            #                 min_dist = dist
            #                 min_id = id
            #             if dist < CLOSE_MAX_DISTANCE:
            #                 self.currently_close_ids.append(id)
            #             else:
            #                 self.currently_near_ids.append(id)
            # if min_id is not None:
            #     voice_attention_point = PointStamped()
            #     voice_attention_point.header.frame_id = str(min_id)
            #     voice_attention_point.point = Point(0, 0, 0)

        for id in self.currently_perceived_ids:
            if id not in self.previously_perceived_ids:
                # start fact
                # print("start: robot is perceiving human-"+id)
                fact = Situation(description="robot is perceiving human-"+id, type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data="robot")
                object_property = Property(name="object", data=id)
                predicate_property = Property(name="predicate", data="isPerceiving")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(object_property)
                fact.properties.append(predicate_property)
                self.predicates_map[id+"isPerceiving"] = fact.id
                self.changes.situations_to_update.append(fact)

        for id in self.previously_perceived_ids:
            if id not in self.currently_perceived_ids:
                # stop fact
                if id+"isPerceiving" in self.predicates_map:
                    if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isPerceiving"]):
                        # print("stop: "+"human-"+id+" is perceived")
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isPerceiving"]]
                        fact.end.data = now
                        fact.description = fact.description.replace("is","was")
                        self.changes.situations_to_update.append(fact)
                        del self.predicates_map[id+"isPerceiving"]

        for id in self.currently_near_ids:
            if id not in self.previously_near_ids:
                # start fact
                # print("start: "+"human-"+id+" is near")
                fact = Situation(description="human-"+str(id)+" is near", type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data=id)
                predicate_property = Property(name="predicate", data="isNear")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(predicate_property)
                self.predicates_map[id+"isNear"] = fact.id
                self.changes.situations_to_update.append(fact)

        for id in self.previously_near_ids:
            if id not in self.currently_perceived_ids or id not in self.currently_near_ids:
                # stop fact
                if id+"isNear" in self.predicates_map:
                    print("stop: "+"human-"+id+" is near")
                    if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isNear"]):
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isNear"]]
                        fact.end.data = now
                        fact.description = fact.description.replace("is","was")
                        self.changes.situations_to_update.append(fact)
                        del self.predicates_map[id+"isNear"]

        for id in self.currently_close_ids:
            if id not in self.previously_close_ids:
                # start fact
                # print "start: "+"human-"+id+" is close"
                fact = Situation(description="human-"+id+" is close", type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data=id)
                predicate_property = Property(name="predicate", data="isClose")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(predicate_property)
                self.predicates_map[id+"isClose"] = fact.id
                self.changes.situations_to_update.append(fact)

        for id in self.previously_close_ids:
            if id not in self.currently_perceived_ids or id not in self.currently_close_ids:
                # stop fact
                if id+"isClose" in self.predicates_map:
                    if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isClose"]):
                        print("end: "+"human-"+id+" is close")
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isClose"]]
                        fact.end.data = now
                        fact.description = fact.description.replace("is","was")
                        self.changes.situations_to_update.append(fact)
                        del self.predicates_map[id+"isClose"]


        # for id in self.currently_speaking_ids:
        #     if id not in self.previously_speaking_ids:
        #         # start fact
        #         print("start: "+"human-"+id+" is speaking")
        #         fact = Situation(description="human-"+id+" is speaking", type=FACT, id=str(uuid.uuid4().hex))
        #         subject_property = Property(name="subject", data=id)
        #         predicate_property = Property(name="predicate", data="isSpeaking")
        #         fact.start.data = now
        #         fact.end.data = rospy.Time(0)
        #         fact.properties.append(subject_property)
        #         fact.properties.append(predicate_property)
        #         self.predicates_map[id+"isSpeaking"] = fact.id
        #         self.changes.situations_to_update.append(fact)

        # for id in self.previously_speaking_ids:
        #     if id not in self.currently_perceived_ids or id not in self.currently_speaking_ids:
        #         # stop fact
        #         if id+"isSpeaking" in self.predicates_map:
        #             if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isSpeaking"]):
        #                 print("stop: "+"human-"+id+" is speaking")
        #                 fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isSpeaking"]]
        #                 fact.end.data = now
        #                 fact.description = fact.description.replace("is","was")
        #                 self.changes.situations_to_update.append(fact)
        #                 self.last_speaking_id = fact.id
        #                 del self.predicates_map[id+"isSpeaking"]

        for subject_id in self.currently_looking_at.keys():
            object_id = self.currently_looking_at[subject_id]
            start_fact = False
            if subject_id not in self.previously_looking_at:
                start_fact = True
            else:
                if subject_id not in self.previously_looking_at:
                    start_fact = True
                else:
                    if object_id != self.previously_looking_at[subject_id]:
                        start_fact = True
            if start_fact is True:
                if object_id != "robot":
                    if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                        object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
                        # print("start: "+"human-"+subject_id+" is looking at "+object.name)
                        fact = Situation(description="human-"+subject_id+" is looking at "+object.name, type=ACTION, id=str(uuid.uuid4().hex))
                        subject_property = Property(name="subject", data=subject_id)
                        object_property = Property(name="object", data=object_id)
                        predicate_property = Property(name="predicate", data="isLookingAt")
                        fact.start.data = now
                        fact.end.data = rospy.Time(0)
                        fact.properties.append(subject_property)
                        fact.properties.append(object_property)
                        fact.properties.append(predicate_property)
                        self.predicates_map[subject_id+"isLookingAt"+object_id] = fact.id
                        self.changes.situations_to_update.append(fact)
                else:
                    # print("start: "+"human-"+subject_id+" is looking at "+object_id)
                    fact = Situation(description="human-"+subject_id+" is looking at "+object_id, type=ACTION, id=str(uuid.uuid4().hex))
                    subject_property = Property(name="subject", data=subject_id)
                    object_property = Property(name="object", data=object_id)
                    predicate_property = Property(name="predicate", data="isLookingAt")
                    fact.start.data = now
                    fact.end.data = rospy.Time(0)
                    fact.properties.append(subject_property)
                    fact.properties.append(object_property)
                    fact.properties.append(predicate_property)
                    self.predicates_map[subject_id+"isLookingAt"+object_id] = fact.id
                    self.changes.situations_to_update.append(fact)

        for subject_id in self.previously_looking_at.keys():
            object_id = self.previously_looking_at[subject_id]
            stop_fact = False
            if subject_id not in self.currently_looking_at:
                stop_fact = True
            else:
                if object_id != self.previously_looking_at[subject_id]:
                    stop_fact = True
            if stop_fact is True:
                #if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                    #object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
                # print("stop: "+"human-"+subject_id+" is looking at "+object_id)
                fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[subject_id+"isLookingAt"+object_id]]
                fact.end.data = now
                fact.description = fact.description.replace("is","was")
                self.changes.situations_to_delete.append(fact.id)
                del self.predicates_map[subject_id+"isLookingAt"+object_id]


        for subject_id in self.currently_engaging_with.keys():
            object_id = self.currently_engaging_with[subject_id]
            start_fact = False
            if subject_id not in self.previously_engaging_with:
                start_fact = True
            else:
                if object_id != self.previously_engaging_with[subject_id]:
                    start_fact = True
            if start_fact is True:
                if object_id != "robot":
                    if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                        # print("start: "+"human-"+subject_id+" is engaging with "+object.name)
                        fact = Situation(description="human-"+subject_id+" is engaging with "+object.name, type=ACTION, id=str(uuid.uuid4().hex))
                        subject_property = Property(name="subject", data=subject_id)
                        object_property = Property(name="object", data=object_id)
                        predicate_property = Property(name="predicate", data="isEngagingWith")
                        fact.start.data = now
                        fact.end.data = rospy.Time(0)
                        fact.properties.append(subject_property)
                        fact.properties.append(object_property)
                        fact.properties.append(predicate_property)
                        self.predicates_map[subject_id+"isEngagingWith"+object_id] = fact.id
                        self.changes.situations_to_update.append(fact)
                else:
                    # print("start: "+"human-"+subject_id+" is engaging with "+object_id)
                    fact = Situation(description="human-"+subject_id+" is engaging with "+object_id, type=ACTION, id=str(uuid.uuid4().hex))
                    subject_property = Property(name="subject", data=subject_id)
                    object_property = Property(name="object", data=object_id)
                    predicate_property = Property(name="predicate", data="isEngagingWith")
                    fact.start.data = now
                    fact.end.data = rospy.Time(0)
                    fact.properties.append(subject_property)
                    fact.properties.append(object_property)
                    fact.properties.append(predicate_property)
                    self.predicates_map[subject_id+"isEngagingWith"+object_id] = fact.id
                    self.changes.situations_to_update.append(fact)

        for subject_id in self.previously_engaging_with.keys():
            object_id = self.previously_engaging_with[subject_id]
            stop_fact = False
            if subject_id not in self.currently_engaging_with:
                stop_fact = True
            else:
                if object_id != self.currently_engaging_with[subject_id]:
                    stop_fact = True
            if stop_fact is True:
                #if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                #object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
                # print("stop: human-"+subject_id+" is engaging with "+object_id)
                fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[subject_id+"isEngagingWith"+object_id]]
                fact.end.data = now
                fact.description = fact.description.replace("is","was")
                self.changes.situations_to_delete.append(fact.id)
                del self.predicates_map[subject_id+"isEngagingWith"+object_id]


        for subject_id in self.currently_engaged_with.keys():
            object_id = self.currently_engaged_with[subject_id]
            start_fact = False
            if subject_id not in self.previously_engaged_with:
                start_fact = True
            else:
                if object_id not in self.previously_engaged_with[subject_id]:
                    start_fact = True
            if start_fact is True:
                if object_id != "robot":
                    if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                        print("start: "+"human-"+subject_id+" is engaged with "+object.name)
                        fact = Situation(description="human-"+subject_id+" is engaged with "+object.name, type=ACTION, id=str(uuid.uuid4().hex))
                        subject_property = Property(name="subject", data=subject_id)
                        object_property = Property(name="object", data=object_id)
                        predicate_property = Property(name="predicate", data="isEngagedWith")
                        fact.start.data = now
                        fact.end.data = rospy.Time(0)
                        fact.properties.append(subject_property)
                        fact.properties.append(object_property)
                        fact.properties.append(predicate_property)
                        self.predicates_map[subject_id+"isEngagedWith"+object_id] = fact.id
                        self.changes.situations_to_update.append(fact)
                else:
                    print("start: "+"human-"+subject_id+" is engaged with "+object_id)
                    fact = Situation(description="human-"+subject_id+" is engaged with "+object_id, type=ACTION, id=str(uuid.uuid4().hex))
                    subject_property = Property(name="subject", data=subject_id)
                    object_property = Property(name="object", data=object_id)
                    predicate_property = Property(name="predicate", data="isEngagedWith")
                    fact.start.data = now
                    fact.end.data = rospy.Time(0)
                    fact.properties.append(subject_property)
                    fact.properties.append(object_property)
                    fact.properties.append(predicate_property)
                    self.predicates_map[subject_id+"isEngagedWith"+object_id] = fact.id
                    self.changes.situations_to_update.append(fact)

        for subject_id in self.previously_engaged_with.keys():
            object_id = self.previously_engaged_with[subject_id]
            stop_fact = False
            if subject_id not in self.currently_engaged_with:
                stop_fact = True
            else:
                if object_id != self.currently_engaged_with[subject_id]:
                    #if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                    stop_fact = True
            if stop_fact is True:
                #if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                #object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
                print("stop: "+"human-"+subject_id+" is engaged with "+object_id)
                fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[subject_id+"isEngagedWith"+object_id]]
                fact.end.data = now
                fact.description = fact.description.replace("is","was")
                self.changes.situations_to_delete.append(fact.id)
                del self.predicates_map[subject_id+"isEngagedWith"+object_id]
                del self.last_engagement_time[subject_id][object_id]


        # for subject_id in self.currently_speaking_to.keys():
        #     for object_id in self.currently_speaking_to[subject_id]:
        #         start_fact = False
        #         if subject_id not in self.previously_speaking_to:
        #             start_fact = True
        #         else:
        #             if object_id not in self.previously_speaking_to[subject_id]:
        #                 if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
        #                     start_fact = True
        #         if start_fact is True:
        #             object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
        #             fact = Situation(description="human is speaking to "+object.name, type=ACTION, id=str(uuid.uuid4().hex))
        #             subject_property = Property(name="subject", data=subject_id)
        #             object_property = Property(name="object", data=object_id)
        #             predicate_property = Property(name="action", data="isSpeakingTo")
        #             fact.start.data = now
        #             fact.description = fact.description.replace("is","was")
        #             fact.end.data = rospy.Time(0)
        #             fact.properties.append(subject_property)
        #             fact.properties.append(object_property)
        #             fact.properties.append(predicate_property)
        #             self.predicates_map[subject_id+"isSpeakingTo"+object_id] = fact.id
        #             self.changes.situations_to_update.append(fact)
        #
        # for subject_id in self.previously_speaking_to.keys():
        #     for object_id in self.previously_speaking_to[subject_id]:
        #         stop_fact = False
        #         if subject_id not in currently_speaking_at:
        #             stop_fact = True
        #         else:
        #             if object_id not in self.previously_speaking_to[subject_id]:
        #                 if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
        #                     stop_fact = True
        #         if stop_fact is True:
        #             fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[subject_id+"isSpeakingTo"+object_id]]
        #             fact.end.data = now
        #             fact.description = fact.description.replace("is","was")
        #             self.changes.situations_to_update(fact)
        #             del self.predicates_map[subject_id+"isSpeakingTo"+object_id]

        ids_overrided = []
        #print("local timeline size : "+str(len(self.ctx.worlds()[self.world].timeline().situations())))
        to_repair = []
        # if self.person_of_interest is not None:
        #     if rospy.Time().now() - self.last_update_person > rospy.Duration(2.0):
        #         self.person_of_interest = None

        for situation in self.ctx.worlds()[self.world].timeline().situations():
            updated = False
            deleted = False
            if situation.end.data != rospy.Time(0):
                if rospy.Time().now().to_sec() - situation.end.data.to_sec() > 4.0:
                    deleted = True
            subject = self.ctx.worlds()[self.world].timeline().situations().get_situation_property(situation.id, "subject")
            object = self.ctx.worlds()[self.world].timeline().situations().get_situation_property(situation.id, "object")
            if subject in to_repair and subject in self.alternate_id_map:
                subject = self.alternate_id_map[subject]
            if object in to_repair and object in self.alternate_id_map:
                object = self.alternate_id_map[object]
            for property in situation.properties:
                if property.data in self.alternate_id_map and property.data in to_repair:
                    if property.name == "object":
                        property.data = object
                        #print "repair timeline for node :"+self.alternate_id_map[object]
                        updated = True
                    if property.name == "subject":
                        property.data = subject
                        #print "repair timeline for node :"+self.alternate_id_map[subject]
                        updated = True
            if updated and not deleted:
                print "repair (update) : " + situation.description
                self.changes.situations_to_update.append(situation)
            if deleted:
                print "repair (delete) : " + situation.description
                self.changes.situations_to_delete.append(situation.id)

        for node in self.ctx.worlds()[self.world].scene().nodes():
            if node.id in self.alternate_id_map:
                if self.alternate_id_map[node.id] != node.id or node.last_observation.data + rospy.Duration(self.person_persistence) < rospy.Time.now():
                    print "remove node : human-"+node.id
                    #self.clear_local_data(node.id)
                    if node.id in self.nb_detection:
                        self.nb_detection[node.id] = 0
                    self.changes.nodes_to_delete.append(node.id)


        self.previously_near_ids = self.currently_near_ids
        self.previously_close_ids = self.currently_close_ids
        self.previously_looking_at = self.currently_looking_at
        self.previously_speaking_ids = self.currently_speaking_ids
        self.previously_speaking_to = self.currently_speaking_to
        self.previously_perceived_ids = self.currently_perceived_ids

        self.previously_engaging_with = self.currently_engaging_with
        self.previously_engaged_with = self.currently_engaged_with

        #print ("nb nodes updated : "+str(len(changes.nodes_to_update)))
        #print ("nb situations updated : "+str(len(changes.situations_to_update)))
        self.ctx.worlds()[self.world].update(self.changes, header=person_msg.header)

    def clear_local_data(self, id):

        if id in self.nb_detection:
            self.nb_detection[id] = 0

        if id in self.previously_near_ids:
            self.previously_near_ids.remove(id)

        if id in self.currently_near_ids:
            self.currently_near_ids.remove(id)

        if id in self.previously_close_ids:
            self.previously_close_ids.remove(id)

        if id in self.currently_close_ids:
            self.currently_close_ids.remove(id)

        if id in self.currently_perceived_ids:
            self.currently_perceived_ids.remove(id)

        if id in self.previously_perceived_ids:
            self.previously_perceived_ids.remove(id)

        if id in self.previously_looking_at:
            del self.previously_looking_at[id]

        if id in self.currently_looking_at:
            del self.currently_looking_at[id]

        if id in self.previously_speaking_to:
            del self.previously_speaking_to[id]

        if id in self.currently_speaking_to:
            del self.currently_speaking_to[id]

        to_remove = []
        for sit in self.changes.situations_to_update:
            for property in sit.properties:
                if property.name == "object" or property.name == "subject":
                    if property.data == id:
                        to_remove.append(sit)

        for sit in to_remove:
            self.changes.situations_to_update.remove(sit)

        to_remove = []
        for node in self.changes.nodes_to_update:
            if node.id == id:
                to_remove.append(node)

        for node in to_remove:
            self.changes.nodes_to_update.remove(node)

    def callback_speech(self, msg):
        changes = Changes()
        if self.last_speaking_id != "":
            if self.ctx.worlds()[self.world].timeline().situations().has(self.last_speaking_id):
                fact = self.ctx.worlds()[self.world].timeline().situations()[self.last_speaking_id]
                updated = False
                for property in fact.properties:
                    if property.name == "speech":
                        property.data = msg.data
                        updated = True
                if updated is False:
                    speech_property = Property(name="speech", data=msg.data)
                    fact.properties.append(speech_property)
                changes.situations_to_update.append(fact)
        self.ctx.worlds()[self.world].update(changes)

if __name__ == '__main__':
    rospy.init_node("multimodal_human_provider")
    mhp = MultiModalHumanProvider()
    rospy.spin()

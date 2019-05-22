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
from pyuwds.types.nodes import CAMERA
from pyuwds.types.situations import FACT
from pyuwds.uwds import PROVIDER
from pyuwds.uwds_client import UwdsClient
from perception_msgs.msg import GazeInfoArray, VoiceActivityArray, TrackedPersonArray
from geometry_msgs.msg import PointStamped, Point

import tf2_ros

DEFAULT_CLIP_PLANE_NEAR = 0.2
DEFAULT_CLIP_PLANE_FAR = 1000.0
DEFAULT_HORIZONTAL_FOV = 60.0
DEFAULT_ASPECT = 1.33333
LOOK_AT_THRESHOLD = 0.6
MIN_NB_DETECTION = 1
MAX_DIS = 2.5

CLOSE_MAX_DISTANCE = 1.0
NEAR_MAX_DISTANCE = 2.0

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

        self.ros_sub = {"gaze_tracker": message_filters.Subscriber("wp2/gaze", GazeInfoArray),
                        "voice_tracker": message_filters.Subscriber("wp2/voice", VoiceActivityArray),
                        "person_tracker": message_filters.Subscriber("wp2/track", TrackedPersonArray),
                        "speech_recognition" : rospy.Subscriber("speech_recognition", String, self.callback_speech)}

        self.ts = message_filters.TimeSynchronizer([self.ros_sub["gaze_tracker"], self.ros_sub["voice_tracker"], self.ros_sub["person_tracker"]], 50)
        self.ts.registerCallback(self.callback)

        self.previously_perceived_ids = []
        self.previously_near_ids = []
        self.previously_close_ids = []
        self.previously_speaking_ids = []
        self.previously_looking_at = {}
        self.previously_speaking_to = {}

        self.world = rospy.get_param("~output_world", "robot/humans")

        self.predicates_map = {}
        self.human_distances = {}

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pepper_id = str(uuid.uuid4())

        print " Ready"

    def callback(self, gaze_msg, voice_msg, person_msg):
        #print "perception callback called"

        #trans = self.tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
        changes = Changes()

        now = rospy.Time.now()

        gaze_attention_point = None
        voice_attention_point = None
        min_dist = 10000

        currently_perceived_ids = []
        currently_near_ids = []
        currently_close_ids = []
        currently_speaking_ids = []
        currently_looking_at = {}
        currently_speaking_to = {}

        if len(person_msg.data) > 0:
            for person in person_msg.data:
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
                    #self.predicates_map[str(person.person_id)] = []
                if person.head_distance < min_dist:
                    min_dist = person.head_distance
                    closest_human = str(person.person_id)
                    currently_perceived_ids.append(str(person.person_id))
                self.human_distances[str(person.person_id)] = person.head_distance

                t = [person.head_pose.position.x, person.head_pose.position.y, person.head_pose.position.z]
                q = [person.head_pose.orientation.x, person.head_pose.orientation.y, person.head_pose.orientation.z, person.head_pose.orientation.w]

                offset = euler_matrix(0, math.radians(90), math.radians(90), "rxyz")

                transform = numpy.dot(transformation_matrix(t, q), offset)

                position = translation_from_matrix(transform)
                quaternion = quaternion_from_matrix(transform)

                self.persons[str(person.person_id)].position.pose.position.x = position[0]
                self.persons[str(person.person_id)].position.pose.position.y = position[1]
                self.persons[str(person.person_id)].position.pose.position.z = position[2]

                self.persons[str(person.person_id)].position.pose.orientation.x = quaternion[0]
                self.persons[str(person.person_id)].position.pose.orientation.y = quaternion[1]
                self.persons[str(person.person_id)].position.pose.orientation.z = quaternion[2]
                self.persons[str(person.person_id)].position.pose.orientation.w = quaternion[3]

                changes.nodes_to_update.append(self.persons[str(person.person_id)])
                min_id = person.person_id
                for id in person.alternate_ids:
                    if id < min_id:
                        min_id = id
                self.alternate_id_map[str(person.person_id)] = str(min_id)

            min_dist = 10000
            min_id = None
            for voice in voice_msg.data:
                if str(voice.person_id) in self.persons:
                    if voice.is_speaking:
                        currently_speaking_ids.append(str(voice.person_id))
                        if str(voice.person_id) in self.distances:
                            if self.human_distances[str(voice.person_id)] < min_dist:
                                min_dist = self.human_distances[str(voice.person_id)]
                                min_id = voice.person_id
            if min_id is not None:
                voice_attention_point = PointStamped()
                voice_attention_point.header.frame_id = str(min_id)
                voice_attention_point.point = Point(0, 0, 0)

            for gaze in gaze_msg.data:
                if str(gaze.person_id) in self.persons:
                    if gaze.probability_looking_at_robot > LOOK_AT_THRESHOLD:
                        currently_looking_at[str(gaze.person_id)] = "robot"
                    else:
                        if gaze.probability_looking_at_screen > LOOK_AT_THRESHOLD:
                            currently_looking_at[str(gaze.person_id)] = "screen"
                        else:
                            for attention in gaze.attentions:
                                if attention.target_id in self.persons:
                                    if attention.probability_looking_at_target > LOOK_AT_THRESHOLD:
                                        if str(attention.target_id) in self.alternate_id_map:
                                            currently_looking_at[str(gaze.person_id)] = self.alternate_id_map[str(attention.target_id)]

            min_dist = 10000
            min_id = None
            for id, dist in self.human_distances.items():
                if dist < NEAR_MAX_DISTANCE:
                    if min_dist > dist:
                        min_dist = dist
                        min_id = id
                    if dist < CLOSE_MAX_DISTANCE:
                        currently_close_ids.append(id)
                    else:
                        currently_near_ids.append(id)
            if min_id is not None:
                voice_attention_point = PointStamped()
                voice_attention_point.header.frame_id = str(min_id)
                voice_attention_point.point = Point(0, 0, 0)

        for id in currently_near_ids:
            if id not in self.previously_near_ids:
                # start fact
                print("start: "+"human-"+id+" is near")
                fact = Situation(description="human-"+str(id)+" is near", type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data=id)
                predicate_property = Property(name="predicate", data="isNear")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(predicate_property)
                self.predicates_map[id+"isNear"] = fact.id
                changes.situations_to_update.append(fact)

        for id in self.previously_near_ids:
            if id not in currently_perceived_ids or id not in currently_near_ids:
                # stop fact
                if id+"isNear" in self.predicates_map:
                    print("stop: "+"human-"+id+" is near")
                    if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isNear"]):
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isNear"]]
                        fact.end.data = now
                        fact.description = fact.description.replace("is","was")
                        changes.situations_to_update.append(fact)
                        del self.predicates_map[id+"isNear"]

        for id in currently_close_ids:
            if id not in self.previously_close_ids:
                # start fact
                print "start: "+"human-"+id+" is close"
                fact = Situation(description="human-"+id+" is close", type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data=id)
                predicate_property = Property(name="predicate", data="isClose")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(predicate_property)
                self.predicates_map[id+"isClose"] = fact.id
                changes.situations_to_update.append(fact)

        for id in self.previously_close_ids:
            if id not in currently_perceived_ids or id not in currently_close_ids:
                # stop fact
                if id+"isClose" in self.predicates_map:
                    print("end: "+"human"+id+" is close")
                    if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isClose"]):
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isClose"]]
                        fact.end.data = now
                        fact.description = fact.description.replace("is","was")
                        changes.situations_to_update.append(fact)
                        del self.predicates_map[id+"isClose"]


        for id in currently_speaking_ids:
            if id not in self.previously_speaking_ids:
                # start fact
                print("start: "+"human-"+id+" is speaking")
                fact = Situation(description="human-"+id+" is speaking", type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data=id)
                predicate_property = Property(name="predicate", data="isSpeaking")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(predicate_property)
                self.predicates_map[id+"isSpeaking"] = fact.id
                changes.situations_to_update.append(fact)

        for id in self.previously_speaking_ids:
            if id not in currently_perceived_ids or id not in currently_speaking_ids:
                # stop fact
                if id+"isSpeaking" in self.predicates_map:
                    print("stop: "+"human-"+id+" is speaking")
                    if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isSpeaking"]):
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isSpeaking"]]
                        fact.end.data = now
                        fact.description = fact.description.replace("is","was")
                        changes.situations_to_update.append(fact)
                        self.last_speaking_id = fact.id
                        del self.predicates_map[id+"isSpeaking"]

        for id in currently_perceived_ids:
            if id not in self.previously_perceived_ids:
                # start fact
                print("start: "+"human-"+id+" is perceived")
                fact = Situation(description="human-"+id+" is perceived", type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data=id)
                predicate_property = Property(name="predicate", data="isPerceived")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(predicate_property)
                self.predicates_map[id+"isPerceived"] = fact.id
                changes.situations_to_update.append(fact)

        for id in self.previously_perceived_ids:
            if id not in currently_perceived_ids:
                # stop fact
                if id+"isPerceived" in self.predicates_map:
                    print("stop: "+"human-"+id+" is perceived")
                    if self.ctx.worlds()[self.world].timeline().situations().has(self.predicates_map[id+"isPerceived"]):
                        fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[id+"isPerceived"]]
                        fact.end.data = now
                        fact.description = fact.description.replace("is","was")
                        changes.situations_to_update.append(fact)
                        del self.predicates_map[id+"isPerceived"]

        for subject_id in currently_looking_at.keys():
            object_id = currently_looking_at[subject_id]
            start_fact = False
            if subject_id not in self.previously_looking_at:
                start_fact = True
            else:
                if object_id not in self.previously_looking_at[subject_id]:
                    if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                        start_fact = True
            if start_fact is True:
                if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                    object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
                    print("start: "+"human-"+id+"is looking at "+object.name)
                    fact = Situation(description="human is looking at "+object.name, type=ACTION, id=str(uuid.uuid4().hex))
                    subject_property = Property(name="subject", data=subject_id)
                    object_property = Property(name="object", data=object_id)
                    predicate_property = Property(name="action", data="isLookingAt")
                    fact.start.data = now
                    fact.end.data = rospy.Time(0)
                    fact.properties.append(subject_property)
                    fact.properties.append(object_property)
                    fact.properties.append(predicate_property)
                    self.predicates_map[subject_id+"isLookingAt"+object_id] = fact.id
                    changes.situations_to_update.append(fact)

        for subject_id in self.previously_looking_at.keys():
            object_id = self.previously_looking_at[subject_id]
            stop_fact = False
            if subject_id not in currently_looking_at:
                stop_fact = True
            else:
                if object_id not in self.previously_looking_at[subject_id]:
                    if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                        stop_fact = True
            if stop_fact is True:
                if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                    object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
                    print("stop: "+"human-"+id+"is looking at "+object.name)
                    fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[subject_id+"isLookingAt"+object_id]]
                    fact.end.data = now
                    fact.description = fact.description.replace("is","was")
                    changes.situations_to_update(fact)
                    del self.predicates_map[subject_id+"isLookingAt"+object_id]

        for subject_id in currently_speaking_to.keys():
            for object_id in currently_speaking_to[subject_id]:
                start_fact = False
                if subject_id not in self.previously_speaking_to:
                    start_fact = True
                else:
                    if object_id not in self.previously_speaking_to[subject_id]:
                        if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                            start_fact = True
                if start_fact is True:
                    object = self.ctx.worlds()[self.world].scene().nodes()[object_id]
                    fact = Situation(description="human is speaking to "+object.name, type=ACTION, id=str(uuid.uuid4().hex))
                    subject_property = Property(name="subject", data=subject_id)
                    object_property = Property(name="object", data=object_id)
                    predicate_property = Property(name="action", data="isSpeakingTo")
                    fact.start.data = now
                    fact.description = fact.description.replace("is","was")
                    fact.end.data = rospy.Time(0)
                    fact.properties.append(subject_property)
                    fact.properties.append(object_property)
                    fact.properties.append(predicate_property)
                    self.predicates_map[subject_id+"isSpeakingTo"+object_id] = fact.id
                    changes.situations_to_update.append(fact)

        for subject_id in self.previously_speaking_to.keys():
            for object_id in self.previously_speaking_to[subject_id]:
                stop_fact = False
                if subject_id not in currently_speaking_at:
                    stop_fact = True
                else:
                    if object_id not in self.previously_speaking_to[subject_id]:
                        if self.ctx.worlds()[self.world].scene().nodes().has(object_id):
                            stop_fact = True
                if stop_fact is True:
                    fact = self.ctx.worlds()[self.world].timeline().situations()[self.predicates_map[subject_id+"isSpeakingTo"+object_id]]
                    fact.end.data = now
                    fact.description = fact.description.replace("is","was")
                    changes.situations_to_update(fact)
                    del self.predicates_map[subject_id+"isSpeakingTo"+object_id]

        #print "send updates"
        print currently_looking_at

        self.previously_near_ids = currently_near_ids
        self.previously_close_ids = currently_close_ids
        self.previously_looking_at = currently_looking_at
        self.previously_speaking_ids = currently_speaking_ids
        self.previously_speaking_to = currently_speaking_to
        self.previously_perceived_ids = currently_perceived_ids

        ids_overrided = []
        #print("local timeline size : "+str(len(self.ctx.worlds()[self.world].timeline().situations())))
        for situation in self.ctx.worlds()[self.world].timeline().situations():
            updated = False
            deleted = False
            if rospy.Time().now().to_secs() - situation.end.data.to_secs() > 300.0:
                deleted = True
            else:
                for property in situation.properties:
                    if property.name == "subject" or property.name == "object":
                        if property.data in self.alternate_id_map:
                            if self.alternate_id_map[property.data] != property.data:
                                updated = True
                                situation.description.replace("human-"+property.data, "human-"+self.alternate_id_map[property.data])
                                property.data = self.alternate_id_map[property.data]
                                ids_overrided.append(property.data)
            if updated is True and deleted is False:
                subject = self.ctx.worlds()[self.world].timeline().situations().get_situation_property(situation.id, "subject")
                object = self.ctx.worlds()[self.world].timeline().situations().get_situation_property(situation.id, "object")
                changes.situations_to_update.append(situation)

            if deleted is True:
                changes.situations_to_remove.append(situation.id)

        for id in ids_overrided:
            print "remove node : human-"+id
            changes.nodes_to_remove.append(id)


        self.ctx.worlds()[self.world].update(changes, header=person_msg.header)


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

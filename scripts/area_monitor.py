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
from pyuwds.uwds import MONITOR
from pyuwds.uwds_client import UwdsClient
from perception_msgs.msg import GazeInfoArray, VoiceActivityArray, TrackedPersonArray
from geometry_msgs.msg import PointStamped, Point

X_MIN =
X_MAX =

Y_MIN =
Y_MAX =

ALPHA = 0.01

IS_IN_THRESHOLD = 0.85

class AreaMonitor(UwdsClient):
    def __init__(self):
        self.alpha = rospy.get_param("~alpha", alpha)
        self.threshold = rospy.get_param("~threshold", IS_IN_THRESHOLD)
        self.x_min = rospy.get_param("~x_min", X_MIN)
        self.x_max = rospy.get_param("~x_max", X_MAX)
        self.y_min = rospy.get_param("~y_min", Y_MIN)
        self.y_max = rospy.get_param("~y_max", Y_MAX)

        self.is_inside_area_prob = {}
        self.currently_inside_area = {}
        self.previously_inside_area = {}

        self.predicates_map = {}

        super(AreaMonitor, self).__init__("area_monitor", MONITOR)

    def onSubscribeChanges(self, world_name):
        """
        """
        pass

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass

    def onChanges(self, world_name, header, invalidations):
        """
        """
        changes = Changes()
        now = rospy.Time.now()

        self.currently_inside_area = {}

        for node_id in invalidations.nodes_to_update:
            if self.is_inside_area(self.worlds()[world_name].scene().nodes()[node_id]) is True:
                self.currently_inside_area[node_id] = "robot_infodesk"

        for node_id in invalidations.nodes_to_delete:
            if node_id in self.is_inside_area_prob[node.id]:
                del self.is_inside_area_prob[node.id]

            if node_id in self.previously_inside_area[node.id]:
                del self.previously_inside_area[node.id]

        for node in self.worlds()[world_name].scene().nodes():
            if node.id in self.currently_inside_area and node.id not in self.previously_inside_area:
                area = self.currently_inside_area[node.id]
                fact = Situation(description="human"+node_id+" is inside area "+ area, type=FACT, id=str(uuid.uuid4().hex))
                subject_property = Property(name="subject", data=node_id)
                object_property = Property(name="object", data=area)
                predicate_property = Property(name="predicate", data="isInsideArea")
                fact.start.data = now
                fact.end.data = rospy.Time(0)
                fact.properties.append(subject_property)
                fact.properties.append(object_property)
                fact.properties.append(predicate_property)
                self.predicates_map[node_id+"isInsideArea"+area] = fact.id
                changes.situations_to_update.append(fact)

            if node.id in self.previously_inside_area and node.id not in self.currently_inside_area:
                if node_id+"isInsideArea"+self.previously_inside_area[node.id] in self.predicates_map:
                    fact_id = self.predicates_map[node_id+"isInsideArea"+self.previously_inside_area[node.id]]
                    fact = self.worlds()[world_name].timeline()[fact_id]
                    fact.end.data = now
                    changes.situations_to_update.append(fact_id)

        self.previously_inside_area = self.currently_inside_area

        self.ctx.worlds()[self.output_world].update(changes, header=invalidations.header)


    def onReconfigure(self, worlds_names):
        """
        """
        pass

    def is_inside_area(world_name, node):
        if self.worlds()[world_name].scene().nodes().get_node_property(node.id, "class") == "Human":
            x = node.position.pose.position.x
            y = node.position.pose.position.y

            if x < self.x_max and x > self.x_min:
                if y < self.y_max and x > self.y_min:
                    if node.id not in self.is_inside_area_prob:
                        self.is_inside_area_prob[node.id] = 0
                    self.is_inside_area_prob[node.id] = self.is_inside_area_prob[node.id] + (1.0*self.alpha - self.is_inside_area_prob[node.id])

            return self.is_inside_area_prob[node.id] > self.threshold
        else:
            return False

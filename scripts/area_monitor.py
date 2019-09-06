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
from pyuwds.reconfigurable_client import ReconfigurableClient
from perception_msgs.msg import GazeInfoArray, VoiceActivityArray, TrackedPersonArray
from geometry_msgs.msg import PointStamped, Point

Y_MIN = 8.55
Y_MAX = 17.22

X_MIN = 4.55
X_MAX = 13.22

ALPHA = 0.1

IS_IN_THRESHOLD = 0.70

class AreaMonitor(ReconfigurableClient):
    def __init__(self):
        self.alpha = rospy.get_param("~alpha", ALPHA)
        self.threshold = rospy.get_param("~area_threshold", IS_IN_THRESHOLD)
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

        for sit_id in invalidations.situation_ids_updated:
            changes.situations_to_update.append(self.ctx.worlds()[world_name].timeline().situations()[sit_id])

        for sit_id in invalidations.situation_ids_deleted:
            changes.situations_to_delete.append(sit_id)

        for node_id in invalidations.node_ids_updated:
            changes.nodes_to_update.append(self.ctx.worlds()[world_name].scene().nodes()[node_id])

        for mesh_id in invalidations.mesh_ids_updated:
            changes.meshes_to_update.append(self.ctx.worlds()[world_name].meshes()[mesh_id])

        for mesh_id in invalidations.mesh_ids_deleted:
            changes.meshes_to_delete.append(mesh_id)

        for node_id in invalidations.node_ids_deleted:
            changes.nodes_to_delete.append(node_id)
            if node_id in self.is_inside_area_prob:
                del self.is_inside_area_prob[node_id]

            if node_id in self.previously_inside_area:
                del self.previously_inside_area[node_id]

            if node_id in self.currently_inside_area:
                if node_id+"isInsideArea"+self.previously_inside_area[node_id] in self.predicates_map:
                    fact = self.predicates_map[node_id+"isInsideArea"+self.previously_inside_area[node_id]]
                    print("stop : human-"+node_id+" is inside area "+ self.previously_inside_area[node_id])
                    fact.end.data = now
                    changes.situations_to_update.append(fact)
                    #changes.situations_to_delete.append(fact_id)
                    del self.predicates_map[node_id+"isInsideArea"+self.previously_inside_area[node_id]]


        for node in self.ctx.worlds()[world_name].scene().nodes():
            if self.ctx.worlds()[world_name].scene().nodes().get_node_property(node.id, "class") == "Human":
                if self.is_inside_area(world_name, self.ctx.worlds()[world_name].scene().nodes()[node.id]) is True:
                    self.currently_inside_area[node.id] = "robot_infodesk"
                if node.id in self.currently_inside_area and node.id not in self.previously_inside_area:
                    area = self.currently_inside_area[node.id]
                    print("start : human-"+node.id+" is inside area "+ area)
                    fact = Situation(description="human-"+node.id+" is inside area "+ area, type=FACT, id=str(uuid.uuid4().hex))
                    subject_property = Property(name="subject", data=node.id)
                    object_property = Property(name="object", data=area)
                    predicate_property = Property(name="predicate", data="isInsideArea")
                    fact.start.data = now
                    fact.end.data = rospy.Time(0)
                    fact.properties.append(subject_property)
                    fact.properties.append(object_property)
                    fact.properties.append(predicate_property)
                    self.predicates_map[node.id+"isInsideArea"+area] = fact
                    changes.situations_to_update.append(fact)

                if node.id in self.previously_inside_area and node.id not in self.currently_inside_area:
                    if node.id+"isInsideArea"+self.previously_inside_area[node.id] in self.predicates_map:
                        fact = self.predicates_map[node.id+"isInsideArea"+self.previously_inside_area[node.id]]
                        #fact = self.ctx.worlds()[self.output_world].timeline().situations()[fact_id]
                        #if fact is not None:
                        print("stop : human-"+node.id+" is inside area "+ self.previously_inside_area[node.id])
                        fact.end.data = now
                        changes.situations_to_update.append(fact)
                        del self.predicates_map[node.id+"isInsideArea"+self.previously_inside_area[node.id]]

        self.previously_inside_area = self.currently_inside_area

        self.ctx.worlds()[self.output_world].update(changes)

    def onReconfigure(self, worlds_names):
        """
        """
        pass

    def is_inside_area(self, world_name, node):
        if self.ctx.worlds()[world_name].scene().nodes().get_node_property(node.id, "class") == "Human":
            x = node.position.pose.position.x
            y = node.position.pose.position.y

            if x < self.x_max and x > self.x_min:
                if y < self.y_max and y > self.y_min:
                    is_inside_area = 1.0
                else:
                    is_inside_area = 0.0
            else:
                is_inside_area = 0.0

            if node.id not in self.is_inside_area_prob:
                self.is_inside_area_prob[node.id] = 0.0
            else:
                self.is_inside_area_prob[node.id] = self.is_inside_area_prob[node.id] + self.alpha * (is_inside_area - self.is_inside_area_prob[node.id])

            return self.is_inside_area_prob[node.id] > self.threshold
        else:
            return False

if __name__ == '__main__':
    rospy.init_node("area_monitor")
    am = AreaMonitor()
    rospy.spin()

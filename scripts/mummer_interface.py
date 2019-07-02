#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from pyuwds.reconfigurable_client import ReconfigurableClient
from pyuwds.uwds import READER
from std_msgs.msg import Header
from perspectives_msgs.msg import Fact, FactArrayStamped, AgentArrayStamped, ObjectArrayStamped
from perspectives_msgs.srv import HasMesh, GetName

import tf_conversions
import tf2_ros
import geometry_msgs.msg


class MummerInterface(ReconfigurableClient):
    def __init__(self):
        self.start = False
        super(MummerInterface, self).__init__("mummer_interface", READER)
        #self.input_world = ""
        self._pub = rospy.Publisher('base/current_facts', FactArrayStamped, queue_size=10)

        self._has_mesh_srv = rospy.Service("has_mesh", HasMesh, self.handleHasMesh)
        self._get_name = rospy.Service("get_name", GetName, self.handleGetName)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._timer = rospy.Timer(rospy.Duration(1/30.0), self.handleTimer)

        self.start = True

    def onSubscribeChanges(self, world_name):
        """
        """
        pass

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass

    def onReconfigure(self, worlds_names):
        """
        """

        self.input_world = worlds_names
        print self.input_world[0]
        if self.input_world[0] != "":
            self.start = True
        else:
            self.start = False

    def onChanges(self, world_name, header, invalidations):
        """
        """
        pass

    def handleHasMesh(self, req):
        try:
            nodes = self.ctx.worlds()[req.world].scene().nodes().by_name(req.name)
            if self.ctx.worlds()[req.world].scene().nodes().get_node_property(nodes[0].id, "meshes") != "":
                return True, True
            else:
                return False, True
        except Exception as e:
            print (str(e))
            return False, False

    def handleGetName(self, req):
        try:
            if self.ctx.worlds()[req.world].scene().nodes().has(req.id):
                return self.ctx.worlds()[req.world].scene().nodes()[req.id].name, True
            else:
                return "", False
        except Exception as e:
            print (str(e))
            return "", False

    def handleTimer(self, event):
        if self.start is True:
            header = Header()
            header.frame_id = self.global_frame_id
            header.stamp = rospy.Time.now()
            self.publish_world_facts(self.input_worlds[0], header)
            #self.ctx.worlds()[self.input_worlds[0]].scene().nodes()._lock()
            for node in self.ctx.worlds()[self.input_worlds[0]].scene().nodes():
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.global_frame_id
                t.child_frame_id = node.name.replace("-","_")
                t.transform.translation.x = node.position.pose.position.x
                t.transform.translation.y = node.position.pose.position.y
                t.transform.translation.z = node.position.pose.position.z
                t.transform.rotation.x = node.position.pose.orientation.x
                t.transform.rotation.y = node.position.pose.orientation.y
                t.transform.rotation.z = node.position.pose.orientation.z
                t.transform.rotation.w = node.position.pose.orientation.w
                self._tf_broadcaster.sendTransform(t)
            #self.ctx.worlds()[self.input_worlds[0]].scene().nodes()._unlock()

    def publish_world_facts(self, world_name, header):
        # publish facts
        if self.start is True:
            current_facts = []
            all_facts = []
            facts_by_predicate = {}
            #self.ctx.worlds()[world_name].timeline().situations()._lock()
            for situation in self.ctx.worlds()[world_name].timeline().situations():
                fact_msg = Fact()
                fact_msg.id = situation.id
                fact_msg.description = situation.description
                predicate = self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "predicate")
                subject = self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "subject")
                object = self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "object")
                fact_msg.predicate = predicate
                fact_msg.subject_name = subject
                fact_msg.object_name = object
                fact_msg.start_time = situation.start
                fact_msg.end_time = situation.end

                if situation.end.data == rospy.Time(0):
                    current_facts.append(fact_msg)
                all_facts.append(fact_msg)

                if predicate != "":
                    if predicate not in facts_by_predicate:
                        facts_by_predicate[predicate] = []
                        facts_by_predicate[predicate].append(fact_msg)
            #self.ctx.worlds()[world_name].timeline().situations()._unlock()
            fact_array_stamped = FactArrayStamped()
            fact_array_stamped.header = header
            fact_array_stamped.facts = current_facts

            self._pub.publish(fact_array_stamped)

    #     self.ros_publishers[world_name]["current_facts"].publish(fact_array_stamped)
    #
    #     fact_array_stamped = FactArrayStamped()
    #     fact_array_stamped.header = header
    #     fact_array_stamped.facts = all_facts
    #     self.ros_publishers[world_name]["all_facts"].publish(fact_array_stamped)
    #
    #     if facts_by_predicate:
    #         for predicate in facts_by_predicate.keys():
    #             if str(predicate) + "_facts" not in self.ros_publishers[world_name]:
    #                 self.ros_publishers[world_name][predicate + "_facts"] = rospy.Publisher(
    #                     str(world_name) + "/" + str(predicate) + "_facts", FactArrayStamped, queue_size=50)
    #
    #             fact_array_stamped = FactArrayStamped()
    #             fact_array_stamped.header = header
    #             fact_array_stamped.facts = facts_by_predicate[predicate]
    #             self.ros_publishers[world_name][predicate + "_facts"].publish(fact_array_stamped)
    #
    # def publish_world_objects(self, world_name, world_proxy, header):
    #     objects = []
    #     for node in world_proxy.scene.nodes:
    #         if self.isobject(world_proxy.scene, node):
    #             obj_msg = Object()
    #             obj_msg.id = node.id
    #             obj_msg.name = node.name
    #             t = translation_from_matrix(get_world_transform(world_proxy.scene, node))
    #             q = quaternion_from_matrix(get_world_transform(world_proxy.scene, node))
    #             pose = Pose()
    #             pose.position.x = t[0]
    #             pose.position.y = t[1]
    #             pose.position.z = t[2]
    #             pose.orientation.x = q[0]
    #             pose.orientation.y = q[1]
    #             pose.orientation.z = q[2]
    #             pose.orientation.w = q[3]
    #             pose_stamped = PoseStamped(header, pose)
    #             obj_msg.pose = pose_stamped
    #             objects.append(obj_msg)
    #
    #     object_array_stamped = ObjectArrayStamped()
    #     object_array_stamped.header = header
    #     object_array_stamped.objects = objects
    #     self.ros_publishers[world_name]["objects"].publish(object_array_stamped)
    #
    # def publish_world_agents(self, world_name, world_proxy, header):
    #     agents = []
    #
    #     for node in world_proxy.scene.nodes:
    #         if node.type == CAMERA:
    #             agent_msg = Agent()
    #             agent_msg.id = node.id
    #             agent_msg.name = node.name
    #             t = translation_from_matrix(get_world_transform(world_proxy.scene, node))
    #             q = quaternion_from_matrix(get_world_transform(world_proxy.scene, node))
    #             pose = Pose()
    #             pose.position.x = t[0]
    #             pose.position.y = t[1]
    #             pose.position.z = t[2]
    #             pose.orientation.x = q[0]
    #             pose.orientation.y = q[1]
    #             pose.orientation.z = q[2]
    #             pose.orientation.w = q[3]
    #             pose_stamped = PoseStamped(header, pose)
    #             agent_msg.head_gaze_pose = pose_stamped
    #             agent_bodies = []
    #             for child_id in node.children:
    #                 node = world_proxy.scene.nodes[child_id]
    #                 object_msg = Object()
    #                 object_msg.id = node.id
    #                 object_msg.name = node.name
    #                 t = translation_from_matrix(get_world_transform(world_proxy.scene, node))
    #                 q = quaternion_from_matrix(get_world_transform(world_proxy.scene, node))
    #                 pose = Pose()
    #                 pose.position.x = t[0]
    #                 pose.position.y = t[1]
    #                 pose.position.z = t[2]
    #                 pose.orientation.x = q[0]
    #                 pose.orientation.y = q[1]
    #                 pose.orientation.z = q[2]
    #                 pose.orientation.w = q[3]
    #                 pose_stamped = PoseStamped(header, pose)
    #                 object_msg.pose = pose_stamped
    #                 agent_bodies.append(object_msg)
    #             agent_msg.agent_bodies = agent_bodies
    #             agents.append(agent_msg)
    #
    #     agent_array_stamped = AgentArrayStamped()
    #     agent_array_stamped.header = header
    #     agent_array_stamped.agents = agents
    #     self.ros_publishers[world_name]["agents"].publish(agent_array_stamped)
    #
    #
    # def publish_data(self):
    #     header = Header()
    #     header.frame_id = self.reference_frame
    #     header.stamp = rospy.Time.now()
    #     for world in self.ctx.worlds:
    #         try:
    #             self.ros_publishers[world.name] = {
    #                 "current_facts": rospy.Publisher(str(world.name) + "/current_facts", FactArrayStamped,
    #                                                  queue_size=50),
    #                 "all_facts": rospy.Publisher(str(world.name) + "/all_facts", FactArrayStamped, queue_size=50),
    #                 "objects": rospy.Publisher(str(world.name) + "/objects", ObjectArrayStamped, queue_size=50),
    #                 "agents": rospy.Publisher(str(world.name) + "/agents", AgentArrayStamped, queue_size=50)}
    #
    #             self.publish_world_facts(str(world.name), world, header)
    #             # publish agents
    #             self.publish_world_agents(str(world.name), world, header)
    #             # publish objects
    #             self.publish_world_objects(str(world.name), world, header)
    #         except Exception as e:
    #             rospy.logwarn("[uwds_ros_bridge] Exception occurred : %s" % str(e))

if __name__ == '__main__':
    rospy.init_node("uwds_mummer_interface")
    mi = MummerInterface()
    rospy.spin()

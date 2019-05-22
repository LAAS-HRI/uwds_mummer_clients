#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from pyuwds.uwds_client import UwdsClient
from perspectives_msgs.msg import FactArrayStamped, AgentArrayStamped, ObjectArrayStamped

class MummerInterface(UwdsClient):
    def __init__(self):
        pass

    def publish_world_facts(self, world_name, header):
        # publish facts
        current_facts = []
        all_facts = []
        facts_by_predicate = {}

        for situation in self.ctx.worlds()[world_name].timeline().situation():
            fact_msg = Fact()
            fact_msg.id = situation.id
            fact_msg.description = situation.desc
            predicate, subject, obj = self.parse_situation_desc(situation.desc)
            fact_msg.predicate = predicate
            fact_msg.subject_name = subject
            fact_msg.object_name = obj
            fact_msg.start_time.data = rospy.Time.from_sec(situation.starttime)
            fact_msg.end_time.data = rospy.Time.from_sec(situation.endtime)

            if situation.endtime == 0.0:
                current_facts.append(fact_msg)
            all_facts.append(fact_msg)

            if predicate and predicate not in facts_by_predicate:
                facts_by_predicate[predicate] = []
            facts_by_predicate[predicate].append(fact_msg)

        fact_array_stamped = FactArrayStamped()
        fact_array_stamped.header = header
        fact_array_stamped.facts = current_facts

        self.ros_publishers[world_name]["current_facts"].publish(fact_array_stamped)

        fact_array_stamped = FactArrayStamped()
        fact_array_stamped.header = header
        fact_array_stamped.facts = all_facts
        self.ros_publishers[world_name]["all_facts"].publish(fact_array_stamped)

        if facts_by_predicate:
            for predicate in facts_by_predicate.keys():
                if str(predicate) + "_facts" not in self.ros_publishers[world_name]:
                    self.ros_publishers[world_name][predicate + "_facts"] = rospy.Publisher(
                        str(world_name) + "/" + str(predicate) + "_facts", FactArrayStamped, queue_size=50)

                fact_array_stamped = FactArrayStamped()
                fact_array_stamped.header = header
                fact_array_stamped.facts = facts_by_predicate[predicate]
                self.ros_publishers[world_name][predicate + "_facts"].publish(fact_array_stamped)

    def publish_world_objects(self, world_name, world_proxy, header):
        objects = []
        for node in world_proxy.scene.nodes:
            if self.isobject(world_proxy.scene, node):
                obj_msg = Object()
                obj_msg.id = node.id
                obj_msg.name = node.name
                t = translation_from_matrix(get_world_transform(world_proxy.scene, node))
                q = quaternion_from_matrix(get_world_transform(world_proxy.scene, node))
                pose = Pose()
                pose.position.x = t[0]
                pose.position.y = t[1]
                pose.position.z = t[2]
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                pose_stamped = PoseStamped(header, pose)
                obj_msg.pose = pose_stamped
                objects.append(obj_msg)

        object_array_stamped = ObjectArrayStamped()
        object_array_stamped.header = header
        object_array_stamped.objects = objects
        self.ros_publishers[world_name]["objects"].publish(object_array_stamped)

    def publish_world_agents(self, world_name, world_proxy, header):
        agents = []

        for node in world_proxy.scene.nodes:
            if node.type == CAMERA:
                agent_msg = Agent()
                agent_msg.id = node.id
                agent_msg.name = node.name
                t = translation_from_matrix(get_world_transform(world_proxy.scene, node))
                q = quaternion_from_matrix(get_world_transform(world_proxy.scene, node))
                pose = Pose()
                pose.position.x = t[0]
                pose.position.y = t[1]
                pose.position.z = t[2]
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                pose_stamped = PoseStamped(header, pose)
                agent_msg.head_gaze_pose = pose_stamped
                agent_bodies = []
                for child_id in node.children:
                    node = world_proxy.scene.nodes[child_id]
                    object_msg = Object()
                    object_msg.id = node.id
                    object_msg.name = node.name
                    t = translation_from_matrix(get_world_transform(world_proxy.scene, node))
                    q = quaternion_from_matrix(get_world_transform(world_proxy.scene, node))
                    pose = Pose()
                    pose.position.x = t[0]
                    pose.position.y = t[1]
                    pose.position.z = t[2]
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    pose_stamped = PoseStamped(header, pose)
                    object_msg.pose = pose_stamped
                    agent_bodies.append(object_msg)
                agent_msg.agent_bodies = agent_bodies
                agents.append(agent_msg)

        agent_array_stamped = AgentArrayStamped()
        agent_array_stamped.header = header
        agent_array_stamped.agents = agents
        self.ros_publishers[world_name]["agents"].publish(agent_array_stamped)


    def publish_data(self):
        header = Header()
        header.frame_id = self.reference_frame
        header.stamp = rospy.Time.now()
        for world in self.ctx.worlds:
            try:
                self.ros_publishers[world.name] = {
                    "current_facts": rospy.Publisher(str(world.name) + "/current_facts", FactArrayStamped,
                                                     queue_size=50),
                    "all_facts": rospy.Publisher(str(world.name) + "/all_facts", FactArrayStamped, queue_size=50),
                    "objects": rospy.Publisher(str(world.name) + "/objects", ObjectArrayStamped, queue_size=50),
                    "agents": rospy.Publisher(str(world.name) + "/agents", AgentArrayStamped, queue_size=50)}

                self.publish_world_facts(str(world.name), world, header)
                # publish agents
                self.publish_world_agents(str(world.name), world, header)
                # publish objects
                self.publish_world_objects(str(world.name), world, header)
            except Exception as e:
                rospy.logwarn("[uwds_ros_bridge] Exception occurred : %s" % str(e))

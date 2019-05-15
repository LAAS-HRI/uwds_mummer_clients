#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import time
import rospy
import underworlds
from underworlds.helpers.transformations import *
from underworlds.helpers.geometry import *
from underworlds.types import CAMERA, Situation
from std_msgs.msg import Header, String
from perspectives_msgs.msg import Agent, Fact, Object, FactArrayStamped, ObjectArrayStamped, AgentArrayStamped
from geometry_msgs.msg import PoseStamped, Pose
from perspectives_msgs.srv import StartFact, EndFact, HasMesh
# from visualization_msgs.msg import Marker, MarkerArray

class ROSBridgeNode(object):
    def __init__(self, ctx, reference_frame):
        self.ctx = ctx
        self.reference_frame = reference_frame
        self.current_situations_map = {}
        self.ros_publishers = {}

        self.log_pub = {"situation_log": rospy.Publisher("external_predicates/log", String, queue_size=5)}

        self.ros_services = {"has_mesh": rospy.Service("uwds_ros_bridge/has_mesh", HasMesh, self.handle_has_mesh),
                             "start_fact": rospy.Service("uwds_ros_bridge/start_fact", StartFact, self.handle_start_fact),
                             "end_fact": rospy.Service("uwds_ros_bridge/end_fact", EndFact, self.handle_end_fact)}

    def parse_situation_desc(self, description):
        if "," in description:
            predicate = description.split("(")[0]
            subject = description.split("(")[1].split(")")[0].split(",")[0]
            obj = description.split("(")[1].split(")")[0].split(",")[1]
        else:
            predicate = description.split("(")[0]
            subject = description.split("(")[1].split(")")[0]
            obj = ""
        return predicate, subject, obj

    def publish_world_facts(self, world_name, world_proxy, header):
        # publish facts
        current_facts = []
        all_facts = []
        facts_by_predicate = {}

        for situation in world_proxy.timeline:
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

    def isobject(self, scene, node):
        isobject = False
        if node != scene.rootnode:
            if node.type == MESH:
                if scene.nodes[node.parent].type != CAMERA:
                    isobject = True
        return isobject

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

    # def publish_markers(self):
    #     header = Header()
    #     header.frame_id = self.reference_frame
    #     header.stamp = rospy.Time.now()
    #     for world in self.ctx.worlds:
    #         color  = pick_color()
    #         for node in self.world.scene.nodes:
    #             if isobject(world.scene, node):

    def start_predicate(self, timeline, predicate, subject_name, object_name=None, isevent=False):
        if object_name is None:
            description = str(predicate) + "(" + str(subject_name) + ")"
        else:
            description = str(predicate) + "(" + str(subject_name) + "," + str(object_name) + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub["situation_log"].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_predicate(self, timeline, predicate, subject_name, object_name=None):
        if object_name is None:
            description = str(predicate) + "(" + str(subject_name) + ")"
        else:
            description = str(predicate) + "(" + str(subject_name) + "," + str(object_name) + ")"
        try:
            sit = self.current_situations_map[description]
            timeline.end(sit)
            self.log_pub["situation_log"].publish("END " + description)
        except Exception as e:
            rospy.logwarn("[uwds_ros_bridge] Exception occurred : " + str(e))

    def handle_start_fact(self, req):
        try:
            predicate, subject, obj = self.parse_situation_desc(req.description)
            #if req.world_name not in self.ctx.worlds:
            #    raise ValueError("The world <%s> does not exist" % req.world_name)
            world = self.ctx.worlds[req.world_name]
            if obj != "":
                fact_id = self.start_predicate(world.timeline, predicate, subject, object_name=obj)
            else:
                fact_id = self.start_predicate(world.timeline, predicate, subject)
            return fact_id, True
        except Exception as e:
            rospy.logwarn("[uwds_ros_bridge] Exception occurred : %s" % str(e))
            return "", False

    def handle_end_fact(self, req):
        #rospy.loginfo(req)
        try:
            #if req.world_name not in self.ctx.worlds:
            #    raise ValueError("The world <%s> does not exist" % req.world_name)
            world = self.ctx.worlds[req.world_name]
            sit = world.timeline[req.fact_id]
            world.timeline.end(sit)
            self.log_pub["situation_log"].publish("END " + sit.desc)
            return True
        except Exception as e:
            rospy.logwarn("[uwds_ros_bridge] Exception occurred : %s" % str(e))
            return False

    def handle_has_mesh(self, req):
        try:
            import copy
            nodes = self.ctx.worlds["env"].scene.nodes
            for node in nodes:
                if req.name == node.name and node.type == MESH:
                    return True, True
            return False, True
        except Exception as e:
            rospy.logwarn("[uwds_ros_bridge] Exception occurred : %s" % str(e))
            return False, False

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_data()
            rate.sleep()


if __name__ == "__main__":
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    # Manage command line options
    import argparse

    parser = argparse.ArgumentParser(description="Broadcast Underworlds data over ROS")
    parser.add_argument("--reference", default="map", help="The reference frame (default map)")
    parser.add_argument("--mode", default="data", help="The mode enabled data, viz or full")
    args = parser.parse_args()

    rospy.init_node("uwds_ros_bridge", anonymous=False)

    with underworlds.Context("uwds_ros_bridge") as ctx:  # Here we connect to the server
        ROSBridgeNode(ctx, args.reference).run()

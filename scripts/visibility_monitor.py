#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import math
import uuid
import numpy
import pybullet as p
import pybullet_data
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes, Situation, Property
from std_msgs.msg import Header
from pyuwds.types.nodes import MESH, CAMERA
from pyuwds.types.situations import FACT
from pyuwds.uwds import FILTER
import tf

class VisibilityMonitor(ReconfigurableClient):
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder", "")
        use_gui = rospy.get_param("~use_gui", False)
        if use_gui is True:
            p.connect(p.GUI) # Initialize bullet graphical version
        else:
            p.connect(p.DIRECT) # Initialize bullet non-graphical version
        p.setAdditionalSearchPath(self.ressource_folder)
        self.urdf_available = {}
        self.node_id_map = {}
        self.reverse_node_id_map = {}
        self.min_treshold = rospy.get_param("~min_treshold", 0.4)
        self.width = rospy.get_param("~width", 480/8)
        self.height = rospy.get_param("~height", 360/8)
        self.visibilities_situations = {}
        super(VisibilityMonitor, self).__init__("visibility_monitor", FILTER)


    def onReconfigure(self, worlds):
        """
        """
        pass

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
        changes = self.monitor(world_name, header, invalidations)
        if changes is not None:
            if len(changes.nodes_to_update) > 0 or len(changes.nodes_to_delete) > 0 or len(changes.situations_to_update) > 0 or len(changes.situations_to_delete) > 0:
                self.ctx.worlds()[world_name+"_visibilities"].update(changes, header)

    def monitor(self, world_name, header, invalidations):
        """
        """
        changes = Changes()

        for node_id in invalidations.node_ids_updated:
            self.updateBulletNodes(world_name, node_id)
            changes.nodes_to_update.append(self.ctx.worlds()[world_name].scene().nodes()[node_id])

        for node_id in invalidations.node_ids_deleted:
            for situation in self.ctx.worlds()[world_name].timeline().situations():
                #if self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "predicate") == "isVisibleBy":
                object_id = self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "object")
                subject_id = self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "subject")
                if node_id == object_id or node_id == subject_id:
                    changes.situations_to_delete.append(situation.id)
                    print "delete : %s" % situation.description
            changes.nodes_to_delete.append(node_id)

        for sit_id in invalidations.situation_ids_updated:
            changes.situations_to_update.append(self.ctx.worlds()[world_name].timeline().situations()[sit_id])

        for sit_id in invalidations.situation_ids_deleted:
            changes.situations_to_delete.append(sit_id)

        #self.ctx.worlds()[world_name].timeline().update(changes.situations_to_update)
        #self.ctx.worlds()[world_name].timeline().remove(changes.situations_to_delete)

        for node in self.ctx.worlds()[world_name].scene().nodes():
            if node.type == CAMERA and node.id == "0":
                if (rospy.Time.now() - node.last_update.data).to_sec() > 1.0:
                    for situation in self.ctx.worlds()[world_name].timeline().situations():
                        if self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "predicate") == "isVisibleBy":
                            camera_id = self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "object")
                            id_seen = self.ctx.worlds()[world_name].timeline().situations().get_situation_property(situation.id, "subject")
                            if node.id == camera_id or node.id == id_seen:
                                changes.situations_to_delete.append(situation.id)
                                print "delete : %s" % situation.description
                                if id_seen+camera_id in self.visibilities_situations:
                                    del self.visibilities_situations[id_seen+camera_id]
                else:
                    visibilities_ = self.computeVisibilities(world_name, node.id)
                    for object_seen in self.ctx.worlds()[world_name].scene().nodes():
                        if object_seen.id in visibilities_:
                            if object_seen.id+node.id not in self.visibilities_situations:
                                situation = Situation()
                                situation.id = str(uuid.uuid4())
                                situation.type = FACT
                                situation.description = object_seen.name + " is visible by " + node.name
                                predicate = Property()
                                predicate.name = "predicate"
                                predicate.data = "isVisibleBy"
                                situation.properties.append(predicate)
                                subject = Property()
                                subject.name = "subject"
                                subject.data = object_seen.id
                                situation.properties.append(subject)
                                object = Property()
                                object.name = "object"
                                object.data = node.id
                                situation.properties.append(object)
                                situation.confidence = visibilities_[object_seen.id]
                                situation.start.data = header.stamp
                                situation.end.data = rospy.Time(0)
                                print "start : %s" % situation.description
                                changes.situations_to_update.append(situation)
                                self.visibilities_situations[object_seen.id+node.id] = situation
                        else:
                            if object_seen.id+node.id in self.visibilities_situations:
                                changes.situations_to_delete.append(self.visibilities_situations[object_seen.id+node.id].id)
                                del self.visibilities_situations[object_seen.id+node.id]

        self.ctx.worlds()[world_name].timeline().update(changes.situations_to_update)
        self.ctx.worlds()[world_name].timeline().remove(changes.situations_to_delete)
        return changes

    def computeVisibilities(self, world_name, camera_id):
        """
        """
        visibilities = {}
        mean_distances_from_center = {}
        nb_pixel = {}
        if camera_id in self.ctx.worlds()[world_name].scene().nodes():
            camera_node = self.ctx.worlds()[world_name].scene().nodes()[camera_id]
            position = [camera_node.position.pose.position.x, camera_node.position.pose.position.y, camera_node.position.pose.position.z]
            orientation = [camera_node.position.pose.orientation.x, camera_node.position.pose.orientation.y, camera_node.position.pose.orientation.z, camera_node.position.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(orientation)
            view_matrix = p.computeViewMatrixFromYawPitchRoll(position, 0.7, math.degrees(euler[2]), math.degrees(euler[1]), math.degrees(euler[0]), 2)
            fov = float(self.ctx.worlds()[world_name].scene().nodes().get_node_property(camera_id, "hfov"))
            clipnear = float(self.ctx.worlds()[world_name].scene().nodes().get_node_property(camera_id, "clipnear"))
            clipfar = float(self.ctx.worlds()[world_name].scene().nodes().get_node_property(camera_id, "clipfar"))
            aspect = float(self.ctx.worlds()[world_name].scene().nodes().get_node_property(camera_id, "aspect"))
            proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, clipnear, clipfar)
            width, height, rgb, depth, seg = p.getCameraImage(self.width, self.height, viewMatrix=view_matrix, projectionMatrix=proj_matrix, flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
            background_nb_pixel = 0
            max_visibility = 0
            r = min(width, height)
            for line in range(0, height):
                for col in range(0, width):
                    pixel = seg[line][col]
                    if (pixel >= 0):
                        bullet_id = pixel & ((1 << 24)-1)
                        if bullet_id in self.reverse_node_id_map:
                            uwds_id = self.reverse_node_id_map[bullet_id]
                            #print "compute visibilities for " +self.ctx.worlds()[world_name].scene().nodes()[uwds_id].name
                            if uwds_id != self.ctx.worlds()[world_name].scene().root_id():
                                if uwds_id not in mean_distances_from_center:
                                    mean_distances_from_center[uwds_id] = 0.0
                                # line_dist = (line-(line/2.0))
                                col_dist = (col-(col/2.0))
                                dist_from_center = math.sqrt(col_dist*col_dist)
                                mean_distances_from_center[uwds_id] += dist_from_center
                                if uwds_id not in nb_pixel:
                                    nb_pixel[uwds_id] = 0
                                nb_pixel[uwds_id] += 1
                                if uwds_id not in visibilities:
                                    visibilities[uwds_id] = 0
                                visibilities[uwds_id] += 1 - min(1, dist_from_center/r)
                                if max_visibility < visibilities[uwds_id]:
                                    max_visibility = visibilities[uwds_id]
                    else:
                        background_nb_pixel += 1.0

            if len(mean_distances_from_center) > 0:
                #print "camera <%s> :" % self.ctx.worlds()[world_name].scene().nodes()[camera_id].name
                for node_id, mean_dist in mean_distances_from_center.items():
                    mean_distances_from_center[node_id] = mean_dist / nb_pixel[node_id]
                    camera_node = self.ctx.worlds()[world_name].scene().nodes()[camera_id]
                    object_node = self.ctx.worlds()[world_name].scene().nodes()[node_id]
                    if mean_distances_from_center[node_id] < r:
                        visibilities[node_id] = 1 - mean_distances_from_center[node_id]/r
                    else:
                        visibilities[node_id] = 0
                    if visibilities[node_id] > self.min_treshold:
                        pass
                        #print " - see object <%s> with %5f confidence" % (object_node.name, visibilities[node_id])
                    else:
                        del visibilities[node_id]
        return visibilities

    def updateBulletNodes(self, world_name, node_id):
        """ This function load the urdf corresponding to the uwds node and set it in the environment
        The urdf need to have the same name than the node name
        :return:
        """
        if self.ctx.worlds()[world_name].scene().root_id() not in self.node_id_map:
            self.node_id_map[self.ctx.worlds()[world_name].scene().root_id()] = p.loadURDF("plane.urdf")
        node = self.ctx.worlds()[world_name].scene().nodes()[node_id]
        if node.type == MESH:
            position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
            orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
            if node_id not in self.node_id_map:
                try:
                    self.node_id_map[node_id] = p.loadURDF(node.name+".urdf", position, orientation)
                    self.reverse_node_id_map[self.node_id_map[node_id]] = node_id
                    rospy.loginfo("[%s::updateBulletNodes] "+node.name+".urdf' loaded successfully", self.node_name)
                except Exception as e:
                    self.node_id_map[node_id] = -1
                if self.node_id_map[node_id] > 0:
                    self.reverse_node_id_map[self.node_id_map[node_id]] = node_id
            else:
                if self.node_id_map[node_id] > 0:
                    p.resetBaseVelocity(self.node_id_map[node_id], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                    p.resetBasePositionAndOrientation(self.node_id_map[node_id], position, orientation)
        else:
            self.node_id_map[node_id] = -1


if __name__ == '__main__':
    rospy.init_node("visibility_monitor")
    vm = VisibilityMonitor()
rospy.spin()

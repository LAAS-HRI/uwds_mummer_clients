#include "uwds_mummer_clients/gazes_provider.h"

using namespace uwds;

namespace uwds_mummer_clients
{
  void GazesProvider::onInit()
  {
    UwdsClientNodelet::onInit();

    default_camera_.name = "human_gaze";
    default_camera_.type = CAMERA;

    std::string hfov, aspect, clipnear, clipfar;
    pnh_->param<std::string>("hfov", hfov, "60.0");
    pnh_->param<std::string>("aspect", aspect, "1.3333");
    pnh_->param<std::string>("clipnear", clipnear, "0.3");
    pnh_->param<std::string>("clipfar", clipfar, "100");


    Property hfov_property;
    Property aspect_property;
    Property clipnear_property;
    Property clipfar_property;
    hfov_property.name = "hfov";
    aspect_property.name = "aspect";
    clipnear_property.name = "clipnear";
    clipfar_property.name = "clipfar";

    hfov_property.data = hfov;
    aspect_property.data = aspect;
    clipnear_property.data = clipnear;
    clipfar_property.data = clipfar;

    default_camera_.properties.push_back(clipnear_property);
    default_camera_.properties.push_back(clipfar_property);
    default_camera_.properties.push_back(hfov_property);
    default_camera_.properties.push_back(aspect_property);

    pnh_->param<int>("nb_min_detection", nb_min_detection_, 10);

    pnh_->param<std::string>("output_world", output_world_, "faces");
    pnh_->param<std::string>("global_frame_id", global_frame_id_, "map");
    input_subscriber_ = pnh_->subscribe("input", 1, &GazesProvider::callback, this);
    connection_status_ = CONNECTED;
  }

  void GazesProvider::callback(const perception_msgs::GazeInfoArrayPtr& msg)
  {
    if(msg->data.size()>0)
    {
      Changes changes;
      static tf::TransformBroadcaster br;
      for(const auto gaze : msg->data)
      {
        if(gaze.head_gaze_available>0)
        {
          if(nb_detection_.count(gaze.person_id)==0)
            nb_detection_.emplace(gaze.person_id, 1);
          else nb_detection_[gaze.person_id]++;

          if(nb_detection_[gaze.person_id] > nb_min_detection_)
          {
            std::string head_gaze_name=(boost::format("human_gaze_%d") % gaze.person_id).str();
            tf::Transform gaze_transform;
            if(gaze.eye_gaze_available>0)
            {
              gaze_transform.setOrigin(tf::Vector3(gaze.eye_gaze.position.x,
                                                   gaze.eye_gaze.position.y,
                                                   gaze.eye_gaze.position.z));
              gaze_transform.setRotation(tf::Quaternion(gaze.eye_gaze.orientation.x,
                                                gaze.eye_gaze.orientation.y,
                                                gaze.eye_gaze.orientation.z,
                                                gaze.eye_gaze.orientation.w));
            } else {
              gaze_transform.setOrigin(tf::Vector3(gaze.head_gaze.position.x,
                                                   gaze.head_gaze.position.y,
                                                   gaze.head_gaze.position.z));
              gaze_transform.setRotation(tf::Quaternion(gaze.head_gaze.orientation.x,
                                                gaze.head_gaze.orientation.y,
                                                gaze.head_gaze.orientation.z,
                                                gaze.head_gaze.orientation.w));
            }
            gaze_transform.setOrigin(tf::Vector3(gaze.head_gaze.position.x,
                                                 gaze.head_gaze.position.y,
                                                 gaze.head_gaze.position.z));
            gaze_transform.setRotation(tf::Quaternion(gaze.head_gaze.orientation.x,
                                              gaze.head_gaze.orientation.y,
                                              gaze.head_gaze.orientation.z,
                                              gaze.head_gaze.orientation.w));
            tf::Transform offset;
            tf::Quaternion q;
            q.setEuler(0.0,1.57,1.57);
            offset.setRotation(q.normalize());
            gaze_transform*=offset;

            br.sendTransform(tf::StampedTransform(gaze_transform, msg->header.stamp, msg->header.frame_id, head_gaze_name));

            if(head_gaze_map_.count(gaze.person_id)==0)
            {
              // Create a new camera node
              Node new_node = default_camera_;
              new_node.name = head_gaze_name;
              new_node.id = NEW_UUID;
              head_gaze_map_.emplace(gaze.person_id, new_node);
            }
            tf::StampedTransform temp;
            tf::Transform camera_transform;
            tf::Transform final_transform;
            tf_listener_.lookupTransform(global_frame_id_, msg->header.frame_id, ros::Time(), temp);
            camera_transform.setOrigin(temp.getOrigin());
            camera_transform.setRotation(temp.getRotation());

            final_transform = camera_transform * gaze_transform * offset;
            final_transform.setRotation(final_transform.getRotation().normalize());
            head_gaze_map_[gaze.person_id].position.pose.position.x = final_transform.getOrigin().getX();
            head_gaze_map_[gaze.person_id].position.pose.position.y = final_transform.getOrigin().getY();
            head_gaze_map_[gaze.person_id].position.pose.position.z = final_transform.getOrigin().getZ();

            head_gaze_map_[gaze.person_id].position.pose.orientation.x = final_transform.getRotation().getX();
            head_gaze_map_[gaze.person_id].position.pose.orientation.y = final_transform.getRotation().getY();
            head_gaze_map_[gaze.person_id].position.pose.orientation.z = final_transform.getRotation().getZ();
            head_gaze_map_[gaze.person_id].position.pose.orientation.w = final_transform.getRotation().getW();

            if (std::isnan(final_transform.getOrigin().getX())) continue;
            if (std::isnan(final_transform.getOrigin().getY())) continue;
            if (std::isnan(final_transform.getOrigin().getZ())) continue;

            if (std::isnan(final_transform.getRotation().getX())) continue;
            if (std::isnan(final_transform.getRotation().getY())) continue;
            if (std::isnan(final_transform.getRotation().getZ())) continue;
            if (std::isnan(final_transform.getRotation().getW())) continue;

            //ROS_ERROR("t : %f, %f, %f", final_transform.getOrigin().getX(), final_transform.getOrigin().getY(), final_transform.getOrigin().getZ());
            //ROS_ERROR("q : %f, %f, %f, %f", final_transform.getRotation().getX(), final_transform.getRotation().getY(), final_transform.getRotation().getZ(), final_transform.getRotation().getW());
            changes.nodes_to_update.push_back(head_gaze_map_[gaze.person_id]);
          }
        }
      }
      sendWorldChanges(output_world_, msg->header, changes);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_mummer_clients::GazesProvider, nodelet::Nodelet)

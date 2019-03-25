#ifndef GAZES_PROVIDER_HPP
#define GAZES_PROVIDER_HPP

#include <uwds/uwds.h>
#include <uwds/uwds_client_nodelet.h>
#include <uwds/tools/model_loader.h>
#include <perception_msgs/GazeInfoArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace uwds_mummer_clients
{
  class GazesProvider : public uwds::UwdsClientNodelet
  {
    public:
      /**@brief
       * The default constructor.
       */
      GazesProvider(): uwds::UwdsClientNodelet(uwds::PROVIDER) {}

      /**@brief
       * The default destructor
       */
      ~GazesProvider() = default;

      /** @brief
       * Initialize method. Subclass should call this method
       * in its onInit method.
       */
      virtual void onInit();

    protected:
      /**@brief
       * This method is called when perception data are received.
       */
      void callback(const perception_msgs::GazeInfoArrayPtr& msg);

      std::map<int, Node> head_gaze_map_;

      std::string global_frame_id_;

      std::string output_world_;

      int nb_min_detection_;

      std::map<int, int> nb_detection_;


      /**@brief
       * Input subscriber for perception data.
       */
      ros::Subscriber input_subscriber_;

      Node default_camera_;

      tf::TransformListener tf_listener_;
  };
}

#endif

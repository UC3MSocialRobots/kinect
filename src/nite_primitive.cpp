/*!
  \file        nite_primitive.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/10/1

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________
 */
#include "nite_primitive.h"

//! \namespace nite_nodelet_ns namespace for \a NitePrimitive class
namespace nite_nodelet_ns {

/*!
\class NitePrimitive
\brief An implementation of the \b NitePrimitiveClass class.
It is shaped as a ROS nodelet (cf <a href="http://www.ros.org/wiki/nodelet">
http://www.ros.org/wiki/nodelet
</a>).
 */
class NitePrimitive : public nodelet::Nodelet, public NitePrimitiveClass {
public:
  //////////////////////////////////////////////////////////////////////////////

  /*! implementation of nodelet::Nodelet::onInit()
   *  set the parameters to their correct values,
   *  starts NITE analysis, then run it in a thread.
   */
  virtual void onInit() {
    // set params
    ros::NodeHandle nh_private ("~");
    nh_private.setParam("display_images_flag", false);
    nh_private.setParam("publish_images_flag", true);
    nh_private.setParam("publish_skeletons_flag", true);
    nh_private.setParam("publish_transforms_flag", false);

    init_nite();
    // create a thread for aquiring data
    int rc = pthread_create(&thread_index, NULL, run_static, (void *)this);
    if (rc){
      printf("ERROR; return code from pthread_create() is %d\n", rc);
      exit(-1);
    }
  } // end onInit();

private:
  //! call run() from a static function (for threading)
  static void* run_static(void* threadid) {
    NitePrimitive* this_ptr = (NitePrimitive*) threadid;
    this_ptr->run();
    pthread_exit(NULL);
  }

  pthread_t thread_index;

}; // end class NitePrimitive

} // end namespace nite_nodelet_ns

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(nite_nodelet_ns, NitePrimitive, nite_nodelet_ns::NitePrimitive, nodelet::Nodelet)


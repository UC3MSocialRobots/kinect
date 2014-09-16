/**
  \file        nite_primitive.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/6

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

\class NitePrimitiveClass
A class that enables opening a connection with a Kinect device
and publishing its video and depth image streams.

In addition, by using NITE,
it also enables the detection of the users.

The image containing these different values is also published,
along with a message enclosing their 3D position.

\section Parameters
  - \b "rate"
        [int, Hz] (default: 30)
        The wanted FPS for output.

  - \b "camera_frame_id"
        [string] (default: "openni_depth_frame")
        What frame should be put in the messages sharing acquired images.

  - \b "display_images_flag"
        [bool] (default: false)
        If true, display the acquired RGB and depth images in windows.
        Not necesary for the acquisition of these images though.

  - \b "publish_images_flag"
        [bool] (default: false)
        If true, publish RGB, depth, user images on the dedicated topics
        (see below)

  - \b "publish_skeletons_flag"
        [bool] (default: false)
        If true, publish the skeleton message on the dedicated topic
        (see below)

  - \b "publish_transforms_flag"
        [bool] (default: false)
        If true, publish the TF transforms between each joint on the "tf" topic.

\section Subscriptions
  None

\section Publications
  - \b "rgb"
        [sensor_msgs/Image]
        The RGB image,
        published only if flag \a publish_images_flag is activated

  - \b "depth"
        [sensor_msgs/Image]
        The depth image,
        published only if flag \a publish_images_flag is activated

  - \b "user"
        [sensor_msgs/Image]
        The user image,
        published only if flag \a publish_images_flag is activated

  - \b "skeletons"
        [kinect::NiteSkeletonList]
        The list of skeletons,
        published only if flag \a publish_skeletons_flag is activated

 */

#ifndef NITE_PRIMITIVE_H
#define NITE_PRIMITIVE_H

#include <vector>

#ifdef NITE_FX
#include "vision/skills/dancer/effect_collection.h"
#else // NITE_FX
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/publisher.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "kinect_utils/kinect_openni_utils.h"
#endif // NITE_FX not defined

// NITE
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
// AD
#include "kinect_utils/skeleton_utils.h"
#include "kinect_utils/user_image_to_rgb.h"


////////////////////////////////////////////////////////////////////////////////

//#define DEBUG

#ifdef DEBUG
//#define DEBUG_PRINT(...)          ROS_WARN(__VA_ARGS__)
#define DEBUG_PRINT(...)          ROS_WARN_THROTTLE(1, __VA_ARGS__)
#define DEBUG_TIMER_INIT          Timer timer;
//#define DEBUG_TIMER_PRINT(...)    timer.printTime(__VA_ARGS__)
#define DEBUG_TIMER_PRINT(...)    ROS_WARN_THROTTLE(1, "Time for %s: %g ms", __VA_ARGS__, timer.time());
#else // no DEBUG
#define DEBUG_PRINT(...)          {}
#define DEBUG_TIMER_INIT          {}
#define DEBUG_TIMER_PRINT(...)    {}
#endif

#define CHECK_RC(nRetVal, what)  \
  if (nRetVal != XN_STATUS_OK) { \
  ROS_WARN("%s failed: %s", what, xnGetStatusString(nRetVal)); \
  }

////////////////////////////////////////////////////////////////////////////////

class NitePrimitiveClass  {
public:
  typedef XnUserID UserId;
  typedef XnSkeletonJoint JointId;
  static const int QUEUE_SIZE = 10;


  //  //! ctor
  //  NitePrimitiveClass() {
  //    init();
  //  }

  //////////////////////////////////////////////////////////////////////////////

  void init_nite() {
    ROS_WARN("init_nite()");
    g_bNeedPose   = FALSE;
    char empty_str[]="";
    strcpy (g_strPose,empty_str);


    // get params
#ifdef NITE_FX
    std::string configFilename = "openni_tracker.xml";
    rate = 30;
    display_images_flag = false;
    publish_images_flag = false;
    publish_skeletons_flag = false;
    publish_transforms_flag = false;
#else // NITE_FX
    std::string configFilename =
        ros::package::getPath("kinect") + "/launch/openni_tracker.xml";
    nh_private = ros::NodeHandle("~");
    nh_private.param("rate", rate, 30);
    camera_frame_id = "openni_depth_frame";
    nh_private.param("camera_frame_id", camera_frame_id, camera_frame_id);
    nh_private.param("display_images_flag", display_images_flag, false);
    nh_private.param("publish_images_flag", publish_images_flag, false);
    nh_private.param("publish_skeletons_flag", publish_skeletons_flag, false);
    nh_private.param("publish_transforms_flag", publish_transforms_flag, false);
    std::string param_name;

    if (nh_private.searchParam("skeleton_joints", param_name)){
        nh_private.getParam(param_name, _published_joints);
        ROS_INFO("Parameter: %s found", param_name.c_str());
    } else {
        ROS_ERROR("Parameter 'skeleton_joints' could not be found");
    }

    // Check if pubhished_skeketon_joints is ok
    ROS_ASSERT(_published_joints.getType() == XmlRpc::XmlRpcValue::TypeArray);

#endif // not NITE_FX

    // init nite
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
    CHECK_RC(nRetVal, "Find image generator");

    // hardware_registration -> align depth on image
    g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK) {
      nRetVal = g_UserGenerator.Create(g_Context);
      CHECK_RC(nRetVal, "Find user generator");
    }

    // TODO set format here
    // g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
    // g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
    // g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_YUV422);

    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
      ROS_WARN("Supplied user generator doesn't support skeleton");
      // return 1;
    }

    XnCallbackHandle hUserCallbacks;
    g_UserGenerator.RegisterUserCallbacks
        (User_NewUser, User_LostUser, this, hUserCallbacks);

    XnCallbackHandle hCalibrationCallbacks;
    g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks
        (UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, this, hCalibrationCallbacks);

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
      g_bNeedPose = TRUE;
      if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
        ROS_WARN("Pose required, but not supported");
        // return 1;
      }

      XnCallbackHandle hPoseCallbacks;
      g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, this, hPoseCallbacks);

      g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    // publishers
#ifdef NITE_FX
    effect_collection.init();
    ROS_WARN("NitePrimitive: rate:%i Hz, "
                "camera_frame_id:'%s', "
                "display_images_flag:%i, "
                "publish_images_flag:%i, "
                "publish_skeletons_flag:%i, "
                "publish_transforms_flag:%i",
                rate,
                camera_frame_id.c_str(),
                display_images_flag,
                publish_images_flag,
                publish_skeletons_flag,
                publish_transforms_flag);
#else // not NITE_FX
    if (publish_images_flag) {
      transport = new image_transport::ImageTransport(nh_public);
      rgb_img_pub = transport->advertise("rgb/image", QUEUE_SIZE);
      depth_img_pub = transport->advertise("depth/image", QUEUE_SIZE);
      user_img_pub = transport->advertise("user/image", QUEUE_SIZE);

      // try to get the serial
      //    std::string kinect_serial_nb = nite_utils::get_kinect_serial_number(g_Context);
      //    ROS_WARN("serial_nb:'%s'", kinect_serial_nb.c_str());
      // set serial number as a param
      //nh_public.setParam("kinect_serial_number", kinect_serial_nb);
      std::string kinect_serial = "", kinect_serial_param_name = "kinect_serial";
      nh_private.param("kinect_serial_param_name", kinect_serial_param_name, kinect_serial_param_name);
      kinect_serial_param_name = nh_public.resolveName(kinect_serial_param_name);
      int ntry = 0, maxtries = 5;
      can_publish_caminfos = false;
      while(ntry++ < maxtries) {
        if (nh_public.getParam(kinect_serial_param_name, kinect_serial))
          break;
        ROS_INFO("Could not get kinect_serial_number param on '%s'! Waiting one sec",
                 kinect_serial_param_name.c_str());
        sleep(1);
      }
      if (kinect_serial == "") {
        ROS_WARN("Could not get kinect_serial_number param on '%s'! Not publishing cam info",
                 kinect_serial_param_name.c_str());
      }
      else if (kinect_openni_utils::read_camera_info_binary_files
               (kinect_serial, depth_camera_info, rgb_camera_info)) {
        rgb_caminfo_pub = nh_public.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);
        depth_caminfo_pub = nh_public.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
        can_publish_caminfos = true;
      }
      else ROS_WARN("Could not read camera info files for Kinect (serial:%s)! "
                    "Not publishing cam info", kinect_serial.c_str());
    } // end if (publish_images_flag)

    if (publish_skeletons_flag)
      skeletons_pub = nh_public.advertise<kinect::NiteSkeletonList>("skeletons", 1);

    ROS_WARN("NitePrimitive: rate:%i Hz, "
                "camera_frame_id:'%s', "
                "display_images_flag:%i "
                "publish_images_flag:%i, "
                "(topics:rgb:'%s', depth:'%s', user:'%s') "
                "publish_caminfo:%i (topics:rgb:'%s', depth:'%s'), "
                "publish_skeletons_flag:%i (topic:'%s'), "
                "publish_transforms_flag:%i",
                rate,
                camera_frame_id.c_str(),
                display_images_flag,
                publish_images_flag,
                (publish_images_flag ? rgb_img_pub.getTopic().c_str() : ""),
                (publish_images_flag ? depth_img_pub.getTopic().c_str() : ""),
                (publish_images_flag ? user_img_pub.getTopic().c_str() : ""),
                can_publish_caminfos,
                (can_publish_caminfos ? rgb_caminfo_pub.getTopic().c_str() : ""),
                (can_publish_caminfos ? depth_caminfo_pub.getTopic().c_str() : ""),
                publish_skeletons_flag,
                (publish_skeletons_flag ? skeletons_pub.getTopic().c_str() : ""),
                publish_transforms_flag);
#endif //not  NITE_FX

    // no failure
    // return 0;
  } // end init_nite();

  //////////////////////////////////////////////////////////////////////////////

  //! dtor
  ~NitePrimitiveClass() {
    g_Context.Shutdown();
#ifndef NITE_FX
    delete transport;
#endif // NITE_FX not defined
  }

  //////////////////////////////////////////////////////////////////////////////

  void run() {
    ROS_WARN("run()");
    skeleton_utils::Rate r(rate);

    while (ros::ok()) {
      DEBUG_PRINT("run loop");
      DEBUG_TIMER_INIT;
      g_Context.WaitAndUpdateAll();
      DEBUG_TIMER_PRINT("WaitAndUpdateAll()");

#ifdef NITE_FX
      get_userjoint_data();
      generate_cv_images();
      if (display_images_flag)
        display_images();
      effect_collection.fn(bgr8, depth32f, user8, skeleton_list_msg);
#else // not NITE_FX
      bool need_publish_images_now = (publish_images_flag &&
                                      (depth_img_pub.getNumSubscribers() > 0
                                       || rgb_img_pub.getNumSubscribers() > 0
                                       || user_img_pub.getNumSubscribers() > 0));
      bool need_publish_skeletons_now =
          (publish_skeletons_flag && skeletons_pub.getNumSubscribers() > 0);
      bool need_publish_caminfos_now =
          (publish_images_flag &&
           (depth_caminfo_pub.getNumSubscribers() > 0
            || rgb_caminfo_pub.getNumSubscribers() > 0));
      messages_timestamp = ros::Time::now();
      //  ROS_WARN("need_publish_skeletons_now:%i, need_publish_images_now:%i",
      //           need_publish_skeletons_now, need_publish_images_now);

      if (need_publish_skeletons_now || publish_transforms_flag)
        get_userjoint_data();

      if (publish_transforms_flag)
        publish_transforms();
      DEBUG_TIMER_PRINT("publish_transforms()");

      if (need_publish_skeletons_now)
        publish_skeleton();
      DEBUG_TIMER_PRINT("publish_skeleton()");

      if (display_images_flag || need_publish_images_now)
        generate_cv_images();
      DEBUG_TIMER_PRINT("generate_cv_images()");

      if (display_images_flag)
        display_images();
      DEBUG_TIMER_PRINT("display_images()");

      if (need_publish_images_now)
        publish_images();
      DEBUG_TIMER_PRINT("publish_images()");

      if (need_publish_caminfos_now)
        publish_caminfos();
      DEBUG_TIMER_PRINT("publish_caminfos()");
#endif // not NITE_FX

      r.sleep();
    }
  } // end run();

  //////////////////////////////////////////////////////////////////////////////

  // #define COPY_DATA // comment to share data between NITE and CV matrices (faster)

  inline void generate_cv_images() {
    DEBUG_PRINT("generate_cv_images()");

    // paint rgb
    // cf http://nma.web.nitech.ac.jp/fukushima/openni/SampleMultiKinect.cpp
    g_ImageGenerator.GetMetaData(rgbMD);
    XnUInt16 rgb_cols = rgbMD.XRes(), rgb_rows = rgbMD.YRes();
    //    ROS_WARN("rgb:(%ix%i), PixelFormat:%i, BytesPerPixel:%i, DataSize:%i ",
    //           rgb_cols, rgb_rows,
    //           g_ImageGenerator.GetPixelFormat(),
    //           rgbMD.BytesPerPixel(),
    //           rgbMD.DataSize());
#ifdef COPY_DATA
    rgb8.create(rgb_rows,rgb_cols);
    // it seems calling WritableData() generates a segfault
    // cf http://openni-discussions.979934.n3.nabble.com/OpenNI-dev-Periodic-partial-disappearance-of-depth-image-as-a-result-of-SetViewpoint-amp-making-depts-td2270962.html
    memcpy(rgb8.data, rgbMD.WritableData(), rgb_rows * rgb_cols * rgbMD.BytesPerPixel());
#else
    // int rows, int cols, int type, void* data, size_t step=AUTO_STEP
    rgb8 = cv::Mat(rgb_rows, rgb_cols, CV_8UC3,
                   rgbMD.WritableData());
    //    (uchar*) rgbMD.WritableRGB24Data());
    //    (uchar*) g_ImageGenerator.GetRGB24ImageMap());
#endif
    cv::cvtColor(rgb8,bgr8,CV_RGB2BGR);

    // get depth metadata
    g_DepthGenerator.GetMetaData(depthMD);
    XnUInt16 depth_cols = depthMD.XRes(), depth_rows = depthMD.YRes();
    // ROS_WARN("depth:(%ix%i)", depth_cols, depth_rows);
#ifdef COPY_DATA
    depth16.create(depth_rows, depth_cols);
    // it seems calling WritableData() generates a segfault
    // cf http://openni-discussions.979934.n3.nabble.com/OpenNI-dev-Periodic-partial-disappearance-of-depth-image-as-a-result-of-SetViewpoint-amp-making-depts-td2270962.html
    memcpy(depth16.data, depthMD.WritableData(), depth_rows * depth_cols * depthMD.BytesPerPixel());
    // memcpy(depth16.data, depthMD.Data(), rows * cols * depthMD.BytesPerPixel());
    depth16.convertTo(depth32f, CV_32FC1, 1.0 / 1000.0);
#else
    // int rows, int cols, int type, void* data, size_t step=AUTO_STEP
    depth16 = cv::Mat(depth_rows, depth_cols, CV_16UC1, depthMD.WritableData());
    depth16.convertTo(depth32f, CV_32FC1, 1.0 / 1000.0);
#endif

    // user image
    g_UserGenerator.GetUserPixels(0, userMD);
    XnUInt16 user_cols = userMD.XRes(), user_rows = userMD.YRes();
    // ROS_WARN("user:(%ix%i)", user_cols, user_rows);
#ifdef COPY_DATA
    user16.create(user_rows,user_cols); //,CV_16UC1);
    // it seems calling WritableData() generates a segfault
    // cf http://openni-discussions.979934.n3.nabble.com/OpenNI-dev-Periodic-partial-disappearance-of-user-image-as-a-result-of-SetViewpoint-amp-making-depts-td2270962.html
    memcpy(user16.data, userMD.WritableData(), user_rows * user_cols * userMD.BytesPerPixel());
    // memcpy(user16.data, userMD.Data(), user_rows * user_cols * userMD.BytesPerPixel());
    // convert to uchar (8 bits)
    user8.create(user_rows,user_cols); //,CV_8UC1);
#else
    // int rows, int cols, int type, void* data, size_t step=AUTO_STEP
    user16 = cv::Mat(user_rows, user_cols, CV_16UC1, userMD.WritableData());
#endif
    user16.convertTo(user8, CV_8UC1);
  } // end generate_cv_images();

  //////////////////////////////////////////////////////////////////////////////

  //! Requires calling generate_cv_images() before
  inline void display_images() {
    DEBUG_PRINT("display_images()");

    // ROS_WARN(1, "cols:%i, rows:%i", cols, rows);

    // paint the depth
    double minVal, maxVal;
    cv::minMaxLoc(depth16, &minVal, &maxVal);
    // ROS_WARN("min:%g, max:%g", minVal, maxVal);
    depth16.convertTo(depth8_illus, CV_8U, 32.f / 1000);

    // paint the user
    user_image_to_rgb(user8, user_illus, 8);
    skeleton_utils::draw_skeleton_list(user_illus, skeleton_list_msg, 2);

    cv::imshow("depth8_illus", depth8_illus);
    cv::imshow("user_illus", user_illus);
    cv::imshow("bgr8", bgr8);
    cv::waitKey(5);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, UserId nId, void* pCookie) {
    ROS_WARN("New User %d", nId);
    NitePrimitiveClass* this_ptr = (NitePrimitiveClass*) pCookie;

    if (this_ptr->g_bNeedPose)
      this_ptr->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection
          (this_ptr->g_strPose, nId);
    else
      this_ptr->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE User_LostUser
  (xn::UserGenerator& generator, UserId nId, void* pCookie) {
    ROS_WARN("Lost user %d", nId);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE UserCalibration_CalibrationStart
  (xn::SkeletonCapability& capability, UserId nId, void* pCookie) {
    ROS_WARN("Calibration started for user %d", nId);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd
  (xn::SkeletonCapability& capability, UserId nId, XnBool bSuccess, void* pCookie) {
    NitePrimitiveClass* this_ptr = (NitePrimitiveClass*) pCookie;
    if (bSuccess) {
      ROS_WARN("Calibration complete, start tracking user %d", nId);
      this_ptr->g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else {
      ROS_WARN("Calibration failed for user %d", nId);
      if (this_ptr->g_bNeedPose)
        this_ptr->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection
            (this_ptr->g_strPose, nId);
      else
        this_ptr->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE UserPose_PoseDetected
  (xn::PoseDetectionCapability& capability, XnChar const* strPose, UserId nId, void* pCookie) {
    ROS_WARN("Pose %s detected for user %d", strPose, nId);
    NitePrimitiveClass* this_ptr = (NitePrimitiveClass*) pCookie;
    this_ptr->g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    this_ptr->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool add_userjoint_data(const UserId & user_id,
                                 const JointId & joint_id_xn)
  {
    UserJointData out;
    // build child_frame_id
    std::ostringstream frame_str;
    frame_str << joint_id_converter.direct_search(joint_id_xn) << "_" << user_id;
    out.child_frame_id = frame_str.str();

    // get position
    XnSkeletonJointPosition joint_position;
    if (g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition
        (user_id, joint_id_xn, joint_position) != XN_STATUS_OK)
      return false;
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    // get orientation (quaternion)
    XnSkeletonJointOrientation joint_orientation;
    if (g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation
        (user_id, joint_id_xn, joint_orientation) != XN_STATUS_OK)
      return false;

    // confidence as average of position and orientation
    out.position_confidence = joint_position.fConfidence;
    out.orientation_confidence = joint_orientation.fConfidence;

#ifdef NITE_FX // skip computing
    transform.translation.x = -x;
    transform.translation.y = -y;
    transform.translation.z = z;
    transform.rotation.x = 0;
    transform.rotation.y = 0;
    transform.rotation.z = 0;
    transform.rotation.w = 1;
#else // not NITE_FX
    out.transform.setOrigin(tf::Vector3(x, y, z));
    out.transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
        m[3], m[4], m[5],
        m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    out.transform.setOrigin(tf::Vector3(x, y, z));
    out.transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
#if 0 // change_frame, original transform
    //frame_rotation.setEulerZYX(M_PI_2, 0, M_PI_2); // deprecated setEulerZYX()
    frame_rotation.setRPY(M_PI_2, 0, M_PI_2);
#endif // change_frame
    frame_rotation.setRPY(0, 0, M_PI);
    change_frame.setRotation(frame_rotation);

    out.transform = change_frame * out.transform;
#endif // not NITE_FX
    _userjoint_data[user_id][joint_id_xn]= out;
    return true;
  } // end get_userjoint_data_joint_transform();

  //////////////////////////////////////////////////////////////////////////////

  void get_userjoint_data() {
    DEBUG_PRINT("get_userjoint_data()");
    UserId users[15];
    XnUInt16 nusers = 15;
    g_UserGenerator.GetUsers(users, nusers);
    _userjoint_data.clear();
    std::string j_name;
    JointId j_id;

    for (int user_counter = 0; user_counter < nusers; ++user_counter) {
      UserId curr_user_id = users[user_counter];
      if (!g_UserGenerator.GetSkeletonCap().IsTracking(curr_user_id))
        continue;

      for (int i = 0; i < _published_joints.size(); i++){
          // Get joint name and translate them to ids
          j_name = static_cast<std::string>(_published_joints[i]);
          j_id = static_cast<JointId>(joint_id_converter.reverse_search(j_name));

          add_userjoint_data(curr_user_id, j_id);
      }

//      add_userjoint_data(curr_user_id, XN_SKEL_HEAD);
//      add_userjoint_data(curr_user_id, XN_SKEL_NECK);
//      add_userjoint_data(curr_user_id, XN_SKEL_TORSO);
//      // get_userjoint_data(curr_user_id, XN_SKEL_WAIST);

//      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_COLLAR);
//      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_SHOULDER);
//      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_ELBOW);
//      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_WRIST);
//      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_HAND);
//      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_FINGERTIP);

//      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_COLLAR);
//      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_SHOULDER);
//      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_ELBOW);
//      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_WRIST);
//      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_HAND);
//      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_FINGERTIP);

//      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_HIP);
//      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_KNEE);
//      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_ANKLE);
//      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_FOOT);

//      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_HIP);
//      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_KNEE);
//      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_ANKLE);
//      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_FOOT);
    } // end loop user_counter
  } // end get_userjoint_data();

  //////////////////////////////////////////////////////////////////////////////

#ifndef NITE_FX

  //! Requires calling get_userjoint_data() before
  void publish_skeleton() {
    DEBUG_PRINT("get_userjoint_data()");
    int nusers = _userjoint_data.size();
    skeleton_list_msg.header.stamp = messages_timestamp;
    skeleton_list_msg.header.frame_id = camera_frame_id;
    skeleton_list_msg.skeletons.clear();
    skeleton_list_msg.skeletons.reserve(nusers);

    // iterate on all users
    std::map<UserId, std::map<JointId, UserJointData> >::const_iterator user_it
        = _userjoint_data.begin();
    while (user_it != _userjoint_data.end()) {
      kinect::NiteSkeleton curr_skeleton;
      curr_skeleton.header = skeleton_list_msg.header; // copy header
      curr_skeleton.joints.reserve(kinect::NiteSkeleton::SKEL_MAX_JOINTS);
      // iterate on all joints
      std::map<JointId, UserJointData>::const_iterator joint_it = user_it->second.begin();
      while (joint_it != user_it->second.end()) {
        // fill the current joint
        kinect::NiteSkeletonJoint joint;
        const UserJointData* joint_data = &(joint_it->second);

        // project position
        XnPoint3D pt3D, projected;
        pt3D.X = joint_data->transform.getOrigin().getX();
        pt3D.Y = joint_data->transform.getOrigin().getY();
        pt3D.Z = joint_data->transform.getOrigin().getZ();
        g_DepthGenerator.ConvertRealWorldToProjective(1, &pt3D, &projected);

        joint.header = skeleton_list_msg.header; // copy header
        joint.joint_id = joint_it->first;
        joint.child_frame_id = joint_data->child_frame_id;
        joint.pose3D.position.x = joint_data->transform.getOrigin().getX();
        joint.pose3D.position.y = joint_data->transform.getOrigin().getY();
        joint.pose3D.position.z = joint_data->transform.getOrigin().getZ();
        joint.pose3D.orientation.x = joint_data->transform.getRotation().getX();
        joint.pose3D.orientation.y = joint_data->transform.getRotation().getY();
        joint.pose3D.orientation.z = joint_data->transform.getRotation().getZ();
        joint.pose3D.orientation.w = joint_data->transform.getRotation().getW();
        joint.pose2D.x = projected.X / depth16.cols;
        joint.pose2D.y = 1. - projected.Y / depth16.rows;
        joint.pose2D.theta = 0;
        joint.position_confidence = joint_data->position_confidence;
        joint.orientation_confidence = joint_data->orientation_confidence;
        // add the new joint
        curr_skeleton.joints.push_back(joint);
        ++joint_it;
      } // end while (joint_it != _userjoint_data.end())
      // add the current skeleton
      skeleton_list_msg.skeletons.push_back(curr_skeleton);

      ++user_it;
    } // end while (user_it != _userjoint_data.end())
    skeletons_pub.publish(skeleton_list_msg);
  } // end publish_skeleton()

  //////////////////////////////////////////////////////////////////////////////

  //! Requires calling get_userjoint_data() before
  void publish_transforms() {
    DEBUG_PRINT("publish_transforms()");
    static tf::TransformBroadcaster br;
    // iterate on all users
    std::map<UserId, std::map<JointId, UserJointData> >::const_iterator user_it
        = _userjoint_data.begin();
    while (user_it != _userjoint_data.end()) {
      // iterate on all joints
      std::map<JointId, UserJointData>::const_iterator joint_it = user_it->second.begin();
      while (joint_it != user_it->second.end()) {
        const UserJointData* joint_data = &(joint_it->second);
        br.sendTransform(tf::StampedTransform(joint_data->transform, messages_timestamp,
                                              camera_frame_id, joint_data->child_frame_id));
        ++joint_it;
      } // end while (joint_it != _userjoint_data.end())
      ++user_it;
    } // end while (user_it != _userjoint_data.end())
  } // end publish_transforms();

  //////////////////////////////////////////////////////////////////////////////

  //! Requires calling generate_cv_images() before
  inline void publish_images() {
    DEBUG_PRINT("publish_images()");
#if 1 // old impl
    // build depth image
    // publish depth image
    depth_img_bridge.header.frame_id = camera_frame_id;
    depth_img_bridge.header.stamp = messages_timestamp;
    depth_img_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
#ifdef COPY_DATA
    depth32f.copyTo(depth_img_bridge.image);
    depth_img_pub.publish(depth_img_bridge.toImageMsg());
#else // COPY_DATA
    depth_img_bridge.image = depth32f;
    depth_img_bridge.toImageMsg(depth_msg);
    depth_img_pub.publish(depth_msg);
#endif // COPY_DATA

    // publish rgb image
    rgb_img_bridge.header = depth_img_bridge.header;
    //rgb_img_bridge.image = rgb8;
    // rgb_img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_img_bridge.encoding = sensor_msgs::image_encodings::BGR8;
#ifdef COPY_DATA
    bgr8.copyTo(rgb_img_bridge.image);
    rgb_img_pub.publish(rgb_img_bridge.toImageMsg());
#else // COPY_DATA
    rgb_img_bridge.image = bgr8;
    rgb_img_bridge.toImageMsg(rgb_msg);
    rgb_img_pub.publish(rgb_msg);
#endif // COPY_DATA


    // build user image - 16-bit unsigned integer
    // cv::Mat* user_img = &(user_img_bridge.image);
#if 0 // keep as unsigned short
    user_img->create(rows,cols,CV_16UC1);
    memcpy(user_img->data, userMD.WritableData(), rows * cols * userMD.BytesPerPixel());
    //cv::cvtuser(*user_img, *user_img, CV_RGB2BGR);
    user_img_bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
#else
    // get as unsigned short
    //    user16.create(rows,cols,CV_16UC1);
    //    memcpy(user16.data, userMD.WritableData(), rows * cols * userMD.BytesPerPixel());
    //    // convert to uchar (8 bits)
    //    user_img->create(rows,cols,CV_8UC1);
    //    user16.convertTo(*user_img, CV_8UC1);
    user_img_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
#endif
    // publish user image
    user_img_bridge.header = depth_img_bridge.header;
#ifdef COPY_DATA
    user8.copyTo(user_img_bridge.image);
    user_img_pub.publish(user_img_bridge.toImageMsg());
#else // COPY_DATA
    user_img_bridge.image = user8;
    user_img_bridge.toImageMsg(user_msg);
    user_img_pub.publish(user_msg);
#endif // COPY_DATA

#else // new impl
    // cf https://github.com/ros-drivers/openni_camera/blob/groovy-devel/src/nodelets/driver.cpp#L459
    std_msgs::Header header;
    header.frame_id = camera_frame_id;
    header.stamp = messages_timestamp;

    cv_bridge::CvImage depth_bridge(header, sensor_msgs::image_encodings::TYPE_32FC1, depth32f);
    depth_img_pub.publish(depth_bridge.toImageMsg());

    cv_bridge::CvImage rgb_bridge(header, sensor_msgs::image_encodings::BGR8, bgr8);
    rgb_img_pub.publish(rgb_bridge.toImageMsg());

    cv_bridge::CvImage user_bridge(header, sensor_msgs::image_encodings::TYPE_8UC1, user8);
    user_img_pub.publish(user_bridge.toImageMsg());
#endif // new impl
  } // end publish_images();

  //////////////////////////////////////////////////////////////////////////////

  inline void publish_caminfos() {
    rgb_caminfo_pub.publish(rgb_camera_info);
    depth_caminfo_pub.publish(depth_camera_info);
  } // end publish_caminfos();

  //////////////////////////////////////////////////////////////////////////////

#endif // not NITE_FX

  //////////////////////////////////////////////////////////////////////////////

private:
  int rate;
  std::string camera_frame_id;
  // images stuff
  bool publish_images_flag;
#ifndef NITE_FX
  ros::NodeHandle nh_public, nh_private;
  image_transport::ImageTransport* transport;
  image_transport::Publisher depth_img_pub;
  image_transport::Publisher user_img_pub;
  image_transport::Publisher rgb_img_pub;
  sensor_msgs::CameraInfo rgb_camera_info;
  sensor_msgs::CameraInfo depth_camera_info;
  bool can_publish_caminfos;
  ros::Publisher rgb_caminfo_pub;
  ros::Publisher depth_caminfo_pub;

  cv_bridge::CvImage depth_img_bridge;
  cv_bridge::CvImage user_img_bridge;
  cv_bridge::CvImage rgb_img_bridge;

  sensor_msgs::Image depth_msg;
  sensor_msgs::Image user_msg;
  sensor_msgs::Image rgb_msg;
#endif // NITE_FX not defined

  xn::SceneMetaData userMD;
  xn::DepthMetaData depthMD;
  xn::ImageMetaData rgbMD;

  //! dp16: unsigned short -> unsigned: 0 to 65535 - CV_16U
  cv::Mat1w depth16;
  cv::Mat1f depth32f;
  cv::Mat1b depth8_illus;

  cv::Mat3b rgb8;
  cv::Mat3b bgr8;

  cv::Mat3b user_illus;
  cv::Mat1w user16;
  cv::Mat1b user8;

  xn::Context        g_Context;
  xn::DepthGenerator g_DepthGenerator;
  xn::ImageGenerator g_ImageGenerator;
  xn::UserGenerator  g_UserGenerator;
  XnBool g_bNeedPose;
  XnChar g_strPose[20];

  // ros::Time
  skeleton_utils::Time messages_timestamp;

  //! true for displaying input
  bool display_images_flag;

  // skeletons and TF stuff
  struct UserJointData {
    tf::Transform transform;
    double position_confidence;
    double orientation_confidence;
    std::string child_frame_id;
  };
  std::map<UserId, std::map<JointId, UserJointData> > _userjoint_data;
  //! true for publishing skeleton
  bool publish_transforms_flag;
  //! true for publishing skeleton TF (transforms)
  bool publish_skeletons_flag;
  //! the message that will be filled with skeleton
  kinect::NiteSkeletonList skeleton_list_msg;
  //! convert joint ID to string
  skeleton_utils::JointId2StringConverter joint_id_converter;
  //! the skeleton publisher
#ifndef NITE_FX
  ros::Publisher skeletons_pub;
#endif // not NITE_FX
   XmlRpc::XmlRpcValue _published_joints;    // to retrieve from the param server

#ifdef NITE_FX
  EffectCollection effect_collection;
#endif // NITE_FX
}; // end class NitePrimitiveClass

#endif // NITE_PRIMITIVE_H

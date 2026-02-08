/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

class ImageGrabber
{
public:
  ImageGrabber(const bool bOnlineRectify, const string &paras) : mbOnlineRectify(bOnlineRectify)
  {
    count = 0;
    cv::FileStorage fSettings(paras, cv::FileStorage::READ);

    //===================times delay=================
    cv::FileNode td_node = fSettings["TimeDelay"]; // t_imu = t_img+td
    if (td_node.empty())
    {
      mtd = 0.0;
    }
    else
    {
      mtd = td_node.real();
    }
    cout << "calibrated time shift = " << mtd << endl;

    if (mbOnlineRectify)
    {
      // cvKl = cv::Mat::eye(3, 3, CV_64F);
      // cvKr = cv::Mat::eye(3, 3, CV_64F);
      //===================camera parameters=================
      ImageSize.width = fSettings["Camera.width"].operator int();
      ImageSize.height = fSettings["Camera.height"].operator int();

      // Left Camera parameters
      float fx = fSettings["Camera1.fx"].real();
      float fy = fSettings["Camera1.fy"].real();
      float cx = fSettings["Camera1.cx"].real();
      float cy = fSettings["Camera1.cy"].real();
      cvKl = (cv::Mat_<float>(3, 3) << fx, 0, cx,
              0, fy, cy,
              0, 0, 1);

      cvK = cvKl.clone();

      float k1 = fSettings["Camera1.k1"].real();
      float k2 = fSettings["Camera1.k2"].real();
      float p1 = fSettings["Camera1.p1"].real();
      float p2 = fSettings["Camera1.p2"].real();
      vDistortionCoefsl = {k1, k2, p1, p2};

      // Right Camera parameters
      float fx2 = fSettings["Camera2.fx"].real();
      float fy2 = fSettings["Camera2.fy"].real();
      float cx2 = fSettings["Camera2.cx"].real();
      float cy2 = fSettings["Camera2.cy"].real();
      cvKr = (cv::Mat_<float>(3, 3) << fx2, 0, cx2,
              0, fy2, cy2,
              0, 0, 1);

      float k12 = fSettings["Camera2.k1"].real();
      float k22 = fSettings["Camera2.k2"].real();
      float p12 = fSettings["Camera2.p1"].real();
      float p22 = fSettings["Camera2.p2"].real();
      vDistortionCoefsr = {k12, k22, p12, p22};

      // Transformation from right camera to left camera, Tlr
      cvTlr = fSettings["Stereo.T_c1_c2"].mat();
      cv::Mat inv_cvTlr  = cvTlr.inv();
      cvTlr = inv_cvTlr.clone();

      cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
      R12.convertTo(R12, CV_64F);
      cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
      t12.convertTo(t12, CV_64F);

      cv::Mat R_r1_u1, R_r2_u2;
      cv::Mat P1, P2, Q;

      cvDistortionCoefsl = cv::Mat(vDistortionCoefsl.size(), 1, CV_32F, vDistortionCoefsl.data());
      cvDistortionCoefsr = cv::Mat(vDistortionCoefsr.size(), 1, CV_32F, vDistortionCoefsr.data());

      cv::stereoRectify(cvKl, cvDistortionCoefsl, cvKr, cvDistortionCoefsr, ImageSize,
                        R12, t12,
                        R_r1_u1, R_r2_u2, P1, P2, Q,
                        cv::CALIB_ZERO_DISPARITY, -1, ImageSize);
      cv::initUndistortRectifyMap(cvKl, cvDistortionCoefsl, R_r1_u1, P1.rowRange(0, 3).colRange(0, 3),
                                  ImageSize, CV_32F, M1l, M2l);
      cv::initUndistortRectifyMap(cvKl, cvDistortionCoefsr, R_r2_u2, P2.rowRange(0, 3).colRange(0, 3),
                                  ImageSize, CV_32F, M1r, M2r);
      // Update calibration
      cvK = (cv::Mat_<float>(3, 3) << P1.at<double>(0, 0), 0, P1.at<double>(0, 2),
             0, P1.at<double>(1, 1), P1.at<double>(1, 2),
             0, 0, 1);
      b = abs(P1.at<double>(0, 3) / P1.at<double>(0, 0) - P2.at<double>(0, 3) / P2.at<double>(0, 0));
      cout << "K left = \n"
           << cvKl << endl
           << "K right = \n"
           << cvKr << endl;
      cout << "Tlr = \n"
           << cvTlr << endl;
      cout << "original b = " << cv::norm(t12) << endl;
      cout << "rectified K = \n"
           << cvK << endl;
      cout << "rectified b = " << b << endl;
    }
  }

  //===========Camera Intrinsic and Extrinsic Parameters==============
  cv::Size ImageSize;
  cv::Mat cvK;
  cv::Mat cvKl;
  cv::Mat cvKr;
  cv::Mat cvDistortionCoefsl;
  cv::Mat cvDistortionCoefsr;
  vector<float> vDistortionCoefsl;
  vector<float> vDistortionCoefsr;
  cv::Mat cvTlr;
  const bool mbOnlineRectify;
  cv::Mat M1l, M2l, M1r, M2r;
  float b;

  //=============Get Data from Rosbag===============
  void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);
  void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);
  cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
  void SyncStereo();
  queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;
  double mtd;
  int count;
  double t_initial;

  //==========Save Path========
  string save_path = "/home/zhiyu/DataSet/SelfCollected/LYR_AGV/12_27/files";
  string left_imgs_save_path = save_path + "/image_2";
  string right_imgs_save_path = save_path + "/image_3";
  string timestamps_save_path = save_path + "/times.txt";
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  ImageGrabber igb(true, argv[1]);

  ros::Subscriber sub_img_left = n.subscribe("/camera_array/cam0/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
  ros::Subscriber sub_img_right = n.subscribe("/camera_array/cam1/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

  std::thread sync_thread(&ImageGrabber::SyncStereo, &igb);

  ros::spin();

  return 0;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    //std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncStereo()
{
  fstream f;
  f.open(timestamps_save_path, ios::out);
  t_initial = -1.0;

  const double maxTimeDiff = 0.01;
  cv::namedWindow("left image");
  while (1)
  {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty() && !imgRightBuf.empty())
    {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec() + mtd;
      tImRight = imgRightBuf.front()->header.stamp.toSec() + mtd;

      this->mBufMutexRight.lock();
      while ((tImLeft - tImRight) > maxTimeDiff)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec() + mtd;
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while ((tImRight - tImLeft) > maxTimeDiff)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec() + mtd;
      }
      this->mBufMutexLeft.unlock();

      if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      cv::Mat imLefttoSave;
      cv::Mat imRighttoSave;
      if (mbOnlineRectify)
      {
        cv::remap(imLeft, imLefttoSave, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRighttoSave, M1r, M2r, cv::INTER_LINEAR);
      }
      else
      {
        imLefttoSave = imLeft.clone();
        imRighttoSave = imRight.clone();
      }

      stringstream filenametream;
      // filenametream <<  setw(6) << setfill('0')<<to_string(kitti_id);
      filenametream << setw(6) << setfill('0') << to_string(int(count));
      string filename;
      filenametream >> filename;
      cv::imwrite(left_imgs_save_path + "/" + filename + ".png", imLefttoSave);
      cv::imwrite(right_imgs_save_path + "/" + filename + ".png", imRighttoSave);
      if (t_initial < 0.0)
      {
        t_initial = tImLeft;
      }

      double ts = tImLeft - t_initial;
      //f << setprecision(9) << ts << endl;
      f << to_string(cout*0.1) <<endl;
      count++;

      if (count > 500)
      {
        cout << "get "<<count<<" images, stop."<<endl;
        f.close();
        break;
      }
        

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

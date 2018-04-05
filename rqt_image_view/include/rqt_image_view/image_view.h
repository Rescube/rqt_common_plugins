/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rqt_image_view__ImageView_H
#define rqt_image_view__ImageView_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_view.h>

#include <image_transport/image_transport.h>
#include <rescube_msgs/image_view_rotation.h>
#include <ros/macros.h>
#include <ros/types.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>

#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QAction>
#include <QImage>

#include <vector>

namespace rqt_image_view {

class ImageView
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  ImageView();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

  virtual void updateTopicList();
  virtual void toggleCompressedTopicFilter(bool compressedonly);

protected:

  // deprecated function for backward compatibility only, use getTopics() instead
  ROS_DEPRECATED virtual QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);

  virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);
  virtual void selectOverlayTopic(const QString& topic);

protected slots:

  virtual void onTopicChanged(int index);
  virtual void onOverlayChanged(int index);
  virtual void onZoom1(bool checked);
  virtual void onDynamicRange(bool checked);
  virtual void saveImage();
  virtual void set_controls_visiblity(bool show);
  virtual void onMousePublish(bool checked);
  virtual void onMouseLeft(int x, int y);
  virtual void onPubTopicChanged();

protected:

  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
  virtual void callbackOverlay(const sensor_msgs::Image::ConstPtr& msg);
  void callbackRotationChanged(const rescube_msgs::image_view_rotation::ConstPtr& msg);
  void updateImage(QImage &image);

  Ui::ImageViewWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;
  image_transport::Subscriber overlay_subscriber_;
  ros::Subscriber rotation_subscriber_;
  cv::Mat conversion_mat_;
  cv::Mat conversion_mat_overlay;

  QImage main_image;
  QImage overlay_image;
  ros::Time main_image_header_stamp;
  ros::Time overlay_image_header_stamp;
private slots:
  void onComboBoxOrientation_currentIndexChanged(int index);
  void on_doubleSpinBoxRotation_valueChanged(double arg1);
  void on_doubleSpinBoxDy_valueChanged(double arg1);
  void on_doubleSpinBoxDx_valueChanged(double arg1);

private:
  ros::Publisher pub_mouse_left_;
  ros::NodeHandle nodeHandle;

  bool pub_topic_custom_;

  QString arg_topic_name, arg_overlay_name;
  QAction *tools_hide_action;

  void autoSelectOverLay(const QString &topicName);
  QList<QString> getSupportedTransports();
};

}

#endif // rqt_image_view__ImageView_H

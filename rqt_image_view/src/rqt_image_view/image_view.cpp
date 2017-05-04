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

#include <rqt_image_view/image_view.h>
#include <rqt_image_view/ratio_layouted_frame.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <rescube_msgs/image_view_rotation.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QDebug>

namespace rqt_image_view {

ImageView::ImageView()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("ImageView");
}

void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
{
    widget_ = new QWidget();
    ui_.setupUi(widget_);

    if (context.serialNumber() > 1)
    {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    updateTopicList();
    ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
    connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));
    connect(ui_.overlay_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onOverlayChanged(int)));

    connect(ui_.refresh_topics_push_button, SIGNAL(toggled(bool)), this, SLOT(updateTopicList()));

    connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));

    connect(ui_.dynamic_range_check_box, SIGNAL(toggled(bool)), this, SLOT(onDynamicRange(bool)));
    connect(ui_.save_as_image_push_button, SIGNAL(pressed()), this, SLOT(saveImage()));
    connect(ui_.comboBoxOrientation, SIGNAL(currentIndexChanged(int)), this, SLOT(onComboBoxOrientation_currentIndexChanged(int)));
    connect(ui_.doubleSpinBoxRotation, SIGNAL(valueChanged(double)), this, SLOT(on_doubleSpinBoxRotation_valueChanged(double)));
    connect(ui_.doubleSpinBoxDx, SIGNAL(valueChanged(double)), this, SLOT(on_doubleSpinBoxDx_valueChanged(double)));
    connect(ui_.doubleSpinBoxDy, SIGNAL(valueChanged(double)), this, SLOT(on_doubleSpinBoxDy_valueChanged(double)));
    connect(ui_.toolButtonFilterCompressed, SIGNAL(toggled(bool)), this, SLOT(updateTopicList()));

    // set topic name if passed in as argument
    const QStringList& argv = context.argv();
    if (!argv.empty()) {
        arg_topic_name = argv[0];
        // add topic name to list if not yet in
        int index = ui_.topics_combo_box->findText(arg_topic_name);
        if (index == -1) {
            QString label(arg_topic_name);
            label.replace(" ", "/");
            ui_.topics_combo_box->addItem(label, QVariant(arg_topic_name));
            ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(arg_topic_name));
        }
    }

    // second argument is the overlay topic's name
    if (context.argv().count() > 1) {
        arg_overlay_name = argv[1];
        int index = ui_.overlay_combo_box->findText(arg_overlay_name);
        if (index == -1) {
            QString label(arg_overlay_name);
            label.replace(" ", "/");
            ui_.overlay_combo_box->addItem(label, QVariant(arg_overlay_name));
            ui_.overlay_combo_box->setCurrentIndex(ui_.overlay_combo_box->findText(arg_overlay_name));
        }
    }

    tools_hide_action = new QAction(tr("Hide toolbar"), this);
    tools_hide_action->setCheckable(true);
    ui_.image_frame->addAction(tools_hide_action);

    connect(tools_hide_action, SIGNAL(toggled(bool)), this, SLOT(set_controls_visiblity(bool)));

    pub_topic_custom_ = false;

    QRegExp rx("([a-zA-Z/][a-zA-Z0-9_/]*)?"); //see http://www.ros.org/wiki/ROS/Concepts#Names.Valid_Names (but also accept an empty field)
    ui_.publish_click_location_topic_line_edit->setValidator(new QRegExpValidator(rx, this));
    connect(ui_.publish_click_location_check_box, SIGNAL(toggled(bool)), this, SLOT(onMousePublish(bool)));
    connect(ui_.image_frame, SIGNAL(mouseLeft(int, int)), this, SLOT(onMouseLeft(int, int)));
    connect(ui_.publish_click_location_topic_line_edit, SIGNAL(editingFinished()), this, SLOT(onPubTopicChanged()));

}

void ImageView::shutdownPlugin()
{
    subscriber_.shutdown();
    pub_mouse_left_.shutdown();
    rotation_subscriber_.shutdown();
}

void ImageView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    QString topic = ui_.topics_combo_box->currentText();
    QString overlay_topic = ui_.overlay_combo_box->currentText();
    //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
    instance_settings.setValue("topic", topic);
    instance_settings.setValue("overlay_topic", overlay_topic);
    instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
    instance_settings.setValue("dynamic_range", ui_.dynamic_range_check_box->isChecked());
    instance_settings.setValue("controls_hidden", tools_hide_action->isChecked());
    instance_settings.setValue("max_range", ui_.max_range_double_spin_box->value());
    instance_settings.setValue("display_latency", ui_.display_latency_check_box->isChecked());
    instance_settings.setValue("full_scale_latency", ui_.full_scale_latency_spin_box->value());
    instance_settings.setValue("publish_click_location", ui_.publish_click_location_check_box->isChecked());
    instance_settings.setValue("mouse_pub_topic", ui_.publish_click_location_topic_line_edit->text());
    instance_settings.setValue("orientation", ui_.comboBoxOrientation->currentIndex());
    instance_settings.setValue("lineEditName", ui_.lineEditName->text());
    instance_settings.setValue("showCompressedOnly", ui_.toolButtonFilterCompressed->isChecked());
}

void ImageView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
    ui_.zoom_1_push_button->setChecked(zoom1_checked);

    bool dynamic_range_checked = instance_settings.value("dynamic_range", false).toBool();
    ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);

    double max_range = instance_settings.value("max_range", ui_.max_range_double_spin_box->value()).toDouble();
    ui_.max_range_double_spin_box->setValue(max_range);

    bool display_latency = instance_settings.value("display_latency", false).toBool();
    ui_.display_latency_check_box->setChecked(display_latency);

    if (!display_latency) {
        ui_.max_latency_label->setVisible(false);
        ui_.full_scale_latency_spin_box->setVisible(false);
    }

    int full_scale_latency = instance_settings.value("full_scale_latency", ui_.full_scale_latency_spin_box->value()).toInt();
    ui_.full_scale_latency_spin_box->setValue(full_scale_latency);

    QString topic = instance_settings.value("topic", "").toString();
    // don't overwrite topic name passed as command line argument
    if (!arg_topic_name.isEmpty())
    {
        arg_topic_name = "";
    }
    else
    {
        //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
        selectTopic(topic);
    }

    QString overlay_topic = instance_settings.value("overlay_topic", "").toString();
    if (!arg_overlay_name.isEmpty())
    {
        arg_overlay_name = "";
    }
    else
    {
        //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
        selectOverlayTopic(overlay_topic);
    }

    bool controls_hidden = instance_settings.value("controls_hidden", false).toBool();
    tools_hide_action->setChecked(controls_hidden);

    bool publish_click_location = instance_settings.value("publish_click_location", false).toBool();
    ui_.publish_click_location_check_box->setChecked(publish_click_location);

    QString pub_topic = instance_settings.value("mouse_pub_topic", "").toString();
    ui_.publish_click_location_topic_line_edit->setText(pub_topic);

    ui_.comboBoxOrientation->setCurrentIndex(instance_settings.value("orientation").toInt());

    ui_.lineEditName->setText(instance_settings.value("lineEditName").toString());
    ui_.toolButtonFilterCompressed->setChecked(instance_settings.value("showCompressedOnly").toBool());
}

void ImageView::updateTopicList()
{
    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");
    QSet<QString> message_sub_types;
    message_sub_types.insert("sensor_msgs/CompressedImage");

    // get declared transports
    QList<QString> transports = getSupportedTransports();

    QString selected = ui_.topics_combo_box->currentText();
    QString selectedOverlay = ui_.overlay_combo_box->currentText();

    // fill combo box
    QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
    topics.append("");
    qSort(topics);
    ui_.topics_combo_box->clear();
    ui_.overlay_combo_box->clear();
    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
    {
        QString label(*it);
        label.replace(" ", "/");
        ui_.topics_combo_box->addItem(label, QVariant(*it));
        ui_.overlay_combo_box->addItem(label, QVariant(*it));
    }

    // restore previous selection
    selectTopic(selected);
    selectOverlayTopic(selectedOverlay);
}

QList<QString> ImageView::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
    QSet<QString> message_sub_types;
    return getTopics(message_types, message_sub_types, transports).values();
}

QSet<QString> ImageView::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    QSet<QString> all_topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
        all_topics.insert(it->name.c_str());
    }

    QSet<QString> topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
        if (message_types.contains(it->datatype.c_str()))
        {
            QString topic = it->name.c_str();

            // if compressed filtering is set and the topic is uncompressed skip it!
            if (ui_.toolButtonFilterCompressed->isChecked() && !topic.endsWith("compressed"))
                continue;

            topics.insert(topic);
            //qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());

            // add transport specific sub-topics
            for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
            {
                if (all_topics.contains(topic + "/" + *jt))
                {
                    QString sub = topic + " " + *jt;
                    if (!topic.endsWith("compressed"))
                        continue;
                    topics.insert(sub);
                    //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
                }
            }
        }
        if (message_sub_types.contains(it->datatype.c_str()))
        {
            QString topic = it->name.c_str();
            int index = topic.lastIndexOf("/");
            if (index != -1)
            {
                if (!topic.endsWith("compressed"))
                    continue;
                topic.replace(index, 1, " ");
                topics.insert(topic);
                //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
            }
        }
    }
    return topics;
}

void ImageView::selectTopic(const QString& topic)
{
    int index = ui_.topics_combo_box->findText(topic);
    if (index == -1)
    {
        index = ui_.topics_combo_box->findText("");
    }
    ui_.topics_combo_box->setCurrentIndex(index);
}

void ImageView::selectOverlayTopic(const QString& topic)
{
    int index = ui_.overlay_combo_box->findText(topic);
    if (index == -1)
    {
        index = ui_.overlay_combo_box->findText("");
    }
    ui_.overlay_combo_box->setCurrentIndex(index);
}

void ImageView::onTopicChanged(int index)
{
    subscriber_.shutdown();
    rotation_subscriber_.shutdown();

    // reset image on topic change
    ui_.image_frame->setImage(QImage());

    QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";
    bool success = true;
    if (!topic.isEmpty())
    {
        image_transport::ImageTransport it(getNodeHandle());
        image_transport::TransportHints hints(transport.toStdString());
        try {
            subscriber_ = it.subscribe(topic.toStdString(), 1, &ImageView::callbackImage, this, hints);
            rotation_subscriber_ = nodeHandle.subscribe(topic.toStdString() + "_rotation", 1, &ImageView::callbackRotationChanged, this);
            //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
            qWarning() << QString::fromStdString(topic.toStdString() + "_rotation");
        } catch (image_transport::TransportLoadException& e) {
            QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
            success = false;
        }
    }

    onMousePublish(ui_.publish_click_location_check_box->isChecked());

    // try to find topic suffixed with _overlay and select that as overlay
    if (success)
        autoSelectOverLay(topic);
}

void ImageView::onOverlayChanged(int index)
{
    overlay_subscriber_.shutdown();

    QStringList parts = ui_.overlay_combo_box->itemData(index).toString().split(" ");
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";

    if (!topic.isEmpty())
    {
        image_transport::ImageTransport it(getNodeHandle());
        image_transport::TransportHints hints(transport.toStdString());
        try {
            overlay_subscriber_ = it.subscribe(topic.toStdString(), 1, &ImageView::callbackOverlay, this, hints);
            //qDebug("ImageView::onOverlayChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), overlay_subscriber_.getTransport().c_str());
        } catch (image_transport::TransportLoadException& e) {
            QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
        }
    }
}

void ImageView::onZoom1(bool checked)
{
    if (checked)
    {
        if (ui_.image_frame->getImage().isNull())
        {
            return;
        }
        ui_.image_frame->setInnerFrameFixedSizeToImage();
        widget_->resize(ui_.image_frame->size());
        widget_->setMinimumSize(widget_->sizeHint());
        widget_->setMaximumSize(widget_->sizeHint());
    } else {
        QSize minSize = QSize(80, 60);
        if (ui_.comboBoxOrientation->currentIndex() == RatioLayoutedFrame::Rotated_90 ||
                ui_.comboBoxOrientation->currentIndex() == RatioLayoutedFrame::Rotated_270) {
            minSize = QSize(60, 80);
        }
        ui_.image_frame->setInnerFrameMinimumSize(minSize);
        widget_->setMinimumSize(minSize);
        ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
        widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    }
}

void ImageView::onDynamicRange(bool checked)
{
    ui_.max_range_double_spin_box->setEnabled(!checked);
}

void ImageView::saveImage()
{
    // take a snapshot before asking for the filename
    QImage img = ui_.image_frame->getImageCopy();

    QString file_name = QFileDialog::getSaveFileName(widget_, tr("Save as image"), "image.png", tr("Image (*.bmp *.jpg *.png *.tiff)"));
    if (file_name.isEmpty())
    {
        return;
    }

    img.save(file_name);
}

void ImageView::onMousePublish(bool checked)
{
    std::string topicName;
    if(pub_topic_custom_)
    {
        topicName = ui_.publish_click_location_topic_line_edit->text().toStdString();
    } else {
        if(!subscriber_.getTopic().empty())
        {
            topicName = subscriber_.getTopic()+"_mouse_left";
        } else {
            topicName = "mouse_left";
        }
        ui_.publish_click_location_topic_line_edit->setText(QString::fromStdString(topicName));
    }

    if(checked)
    {
        pub_mouse_left_ = getNodeHandle().advertise<geometry_msgs::Point>(topicName, 1000);
    } else {
        pub_mouse_left_.shutdown();
    }
}

void ImageView::onMouseLeft(int x, int y)
{
    if(ui_.publish_click_location_check_box->isChecked() && !ui_.image_frame->getImage().isNull())
    {
        geometry_msgs::Point clickLocation;
        // Publish click location in pixel coordinates
        clickLocation.x = round((double)x/(double)ui_.image_frame->width()*(double)ui_.image_frame->getImage().width());
        clickLocation.y = round((double)y/(double)ui_.image_frame->height()*(double)ui_.image_frame->getImage().height());
        clickLocation.z = 0;
        pub_mouse_left_.publish(clickLocation);
    }
}

void ImageView::onPubTopicChanged()
{
    pub_topic_custom_ = !(ui_.publish_click_location_topic_line_edit->text().isEmpty());
    onMousePublish(ui_.publish_click_location_check_box->isChecked());
}

void ImageView::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        conversion_mat_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        try
        {
            // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            if (msg->encoding == "CV_8UC3")
            {
                // assuming it is rgb
                conversion_mat_ = cv_ptr->image;
            } else if (msg->encoding == "8UC1") {
                // convert gray to rgb
                cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
            } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
                // scale / quantify
                double min = 0;
                double max = ui_.max_range_double_spin_box->value();
                if (msg->encoding == "16UC1") max *= 1000;
                if (ui_.dynamic_range_check_box->isChecked())
                {
                    // dynamically adjust range based on min/max in image
                    cv::minMaxLoc(cv_ptr->image, &min, &max);
                    if (min == max) {
                        // completely homogeneous images are displayed in gray
                        min = 0;
                        max = 2;
                    }
                }
                cv::Mat img_scaled_8u;
                cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
                cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
            } else {
                qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
                ui_.image_frame->setImage(QImage());
                return;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
            ui_.image_frame->setImage(QImage());
            return;
        }
    }

    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);

    // if overlay is selected paint the image onto the overlayimage
    // technically we are painting it over
    QPainter overlay_painter(&image);
    if (!overlay_image.isNull()) {
        overlay_painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
        overlay_painter.drawImage(0, 0, overlay_image.scaled(image.width(), image.height(), Qt::IgnoreAspectRatio));
    }
    QPen pen;

    if (ui_.display_latency_check_box->isChecked()) {
        int latency_in_ms = (ros::Time::now() - msg->header.stamp).toNSec() / 1000000;
        // latency bar color scheme:
        //  0%-33%  of full scale: green
        // 33%-66%  of full scale: yellow
        // 66%-100% of full scale: darkyellow
        // >100%    of full scale: red
        if (latency_in_ms > ui_.full_scale_latency_spin_box->value()) {
            pen.setColor(Qt::red);
        } else if (latency_in_ms <= ui_.full_scale_latency_spin_box->value() &&
                   latency_in_ms > (ui_.full_scale_latency_spin_box->value()/3)*2) {
            pen.setColor(QColor::fromRgb(0xFF, 0xA5, 0x00));
        } else if (latency_in_ms <= (ui_.full_scale_latency_spin_box->value()/3)*2 &&
                   latency_in_ms > (ui_.full_scale_latency_spin_box->value()/3)) {
            pen.setColor(Qt::yellow);
        } else {
            pen.setColor(Qt::green);
        }

        pen.setWidth(4);
        overlay_painter.setPen(pen);
        double latency_bar_width = (image.width() / ui_.full_scale_latency_spin_box->value()) * latency_in_ms;
        if (latency_bar_width > image.width())
            latency_bar_width = image.width();
        overlay_painter.drawLine(0, 0, latency_bar_width, 0);
    }

    if (!ui_.lineEditName->text().isEmpty()) {
        pen.setColor(Qt::red);
        pen.setBrush(Qt::red);
        overlay_painter.setPen(pen);
        overlay_painter.drawText(QRect(3,3, image.width(), image.height()), ui_.lineEditName->text());
    }

    ui_.image_frame->setImage(image);

    if (!ui_.zoom_1_push_button->isEnabled())
    {
        ui_.zoom_1_push_button->setEnabled(true);
        onZoom1(ui_.zoom_1_push_button->isChecked());
    }
}


void ImageView::callbackOverlay(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
        conversion_mat_overlay = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        qWarning("ImageView.callback_image() could not convert image to 'bgra8' (%s)", e.what());
        return;
    }
    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    overlay_image = QImage(conversion_mat_overlay.data,
                           conversion_mat_overlay.cols,
                           conversion_mat_overlay.rows,
                           conversion_mat_overlay.step[0],
            QImage::Format_ARGB32).copy();
}

void ImageView::callbackRotationChanged(const rescube_msgs::image_view_rotation::ConstPtr& msg)
{
    ui_.image_frame->setCustomRotation(msg->rotation);
    ui_.image_frame->setDx(msg->dx);
    ui_.image_frame->setDy(msg->dy);
    qWarning() << msg->rotation;
    qWarning() << msg->dx;
    qWarning() << msg->dy;
}


void rqt_image_view::ImageView::set_controls_visiblity(bool hide)
{
    ui_.controlsWidget->setVisible(!hide);
}

void rqt_image_view::ImageView::onComboBoxOrientation_currentIndexChanged(int index)
{
    ui_.image_frame->setOrientation((rqt_image_view::RatioLayoutedFrame::Orientation)index);
}

void rqt_image_view::ImageView::on_doubleSpinBoxRotation_valueChanged(double arg1)
{
    ui_.image_frame->setCustomRotation(arg1);
}


void rqt_image_view::ImageView::on_doubleSpinBoxDy_valueChanged(double arg1)
{
    ui_.image_frame->setDx(arg1);
}

void rqt_image_view::ImageView::on_doubleSpinBoxDx_valueChanged(double arg1)
{
    ui_.image_frame->setDy(arg1);
}

QList<QString> ImageView::getSupportedTransports()
{
    QList<QString> transports;
    image_transport::ImageTransport it(getNodeHandle());
    std::vector<std::string> declared = it.getDeclaredTransports();
    for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
    {
        //qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
        QString transport = it->c_str();

        // strip prefix from transport name
        QString prefix = "image_transport/";
        if (transport.startsWith(prefix))
        {
            transport = transport.mid(prefix.length());
        }
        transports.append(transport);
    }

    return transports;
}

/**
 * @brief ImageView::autoSelectOverLay
 * @param topicName the name of the selected image topic without the "compressed" suffix
 * @details This method will automatically select the
 * overlay topic if there is an existing topic with the
 * name of the topicName argument suffixed with _overlay.
 * It will also handle the overlay comboBox selection.
 */
void ImageView::autoSelectOverLay(const QString & topicName)
{
    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");
    QSet<QString> message_sub_types;
    message_sub_types.insert("sensor_msgs/CompressedImage");

    QStringList parts = topicName.split("/", QString::SkipEmptyParts);
    QString imageNode = parts.first();

    // get declared transports
    QList<QString> transports = getSupportedTransports();

    // fill combo box
    QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
    {
        QString currentTopicName = QVariant(*it).toString();
        parts = currentTopicName.split('/', QString::SkipEmptyParts);
        QString currentNodeName = parts.first();
        if (currentNodeName == imageNode + "_overlay") {
            selectOverlayTopic(*it);
            break;
        }
    }
}

} // end namespace rqt_image_view

PLUGINLIB_EXPORT_CLASS(rqt_image_view::ImageView, rqt_gui_cpp::Plugin)



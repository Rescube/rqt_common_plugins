#ifndef SENDFILEWIDGET_H
#define SENDFILEWIDGET_H

#include <QWidget>

#include <ros/node_handle.h>

#include "ui_send_file_widget.h"

namespace Ui {
class SendFileWidget;
}

class SendFileWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SendFileWidget(QWidget *parent = 0);
    ~SendFileWidget();

    QString getLastDir() const;
    void setLastDir(const QString &value);

    QString getSelectedFile() const;
    void setSelectedFile(const QString &value);

    QString getSelectedTopic() const;
    void setSelectedTopic(const QString &value);

private slots:
    void on_toolButtonBrowse_clicked();

    void on_pushButtonSend_clicked();

    void on_pushButtonRefresh_clicked();

private:
    Ui::SendFileWidget *ui;
    QString lastDir;
    QString selectedFile;
    QString selectedTopic;
    ros::NodeHandle nodeHandle;
};

#endif // SENDFILEWIDGET_H

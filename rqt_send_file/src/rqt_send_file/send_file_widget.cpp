#include <rqt_send_file/send_file_widget.h>

#include <std_msgs/String.h>
#include <ros/publisher.h>
#include <ros/master.h>

#include <QFileDialog>
#include <QFile>
#include <QMessageBox>

SendFileWidget::SendFileWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SendFileWidget)
{
    ui->setupUi(this);
    on_pushButtonRefresh_clicked(); // this will populate the topics combobox
}

SendFileWidget::~SendFileWidget()
{
    delete ui;
}

void SendFileWidget::on_toolButtonBrowse_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this,
                                                    tr("Select the file to send"),
                                                    lastDir);
    if (!filePath.isEmpty()) {
        setSelectedFile(filePath);
    }
}

void SendFileWidget::on_pushButtonSend_clicked()
{
    if (ui->comboBoxTopicName->currentText() == "") {
        QMessageBox::warning(this,
                             tr("Error"),
                             tr("No topics selected to publish!"));
        return;
    }

    if (ui->textEditFileContents->toPlainText().isEmpty()) {
        QMessageBox::warning(this,
                             tr("Error"),
                             tr("Nothing to send!"));
        return;
    }

    ros::Publisher file_publisher = nodeHandle.advertise<std_msgs::String>(ui->comboBoxTopicName->currentText().toStdString(), 1000);
    std_msgs::String msg;

    std::stringstream ss;
    ss << ui->textEditFileContents->toPlainText().toStdString();
    msg.data = ss.str();

    file_publisher.publish(msg);
}

QString SendFileWidget::getSelectedFile() const
{
    return ui->lineEditFilePath->text();
}

void SendFileWidget::setSelectedFile(const QString &value)
{
    if (value.isEmpty())
        return;
    QFile file(value);
    if (!file.open(QFile::ReadOnly)) {
        QMessageBox::warning(
                    this,
                    tr("Error"),
                    tr("Unable to open the %1 file for reading!").arg(value)
                    );
        return;
    }

    ui->textEditFileContents->setText(file.readAll());
    file.close();
    ui->lineEditFilePath->setText(value);
}

QString SendFileWidget::getLastDir() const
{
    return lastDir;
}

void SendFileWidget::setLastDir(const QString &value)
{
    lastDir = value;
}


void SendFileWidget::on_pushButtonRefresh_clicked()
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    selectedTopic = ui->comboBoxTopicName->currentText();
    ui->comboBoxTopicName->clear();

    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
        ui->comboBoxTopicName->addItem(it->name.c_str());
    }

    int selectedIndex = ui->comboBoxTopicName->findText(selectedTopic);
    if (selectedIndex != -1) {
        ui->comboBoxTopicName->setCurrentIndex(selectedIndex);
    }
}
QString SendFileWidget::getSelectedTopic() const
{
    return selectedTopic;
}

void SendFileWidget::setSelectedTopic(const QString &value)
{
    int selectedIndex = ui->comboBoxTopicName->findText(value);
    if (selectedIndex != -1) {
        ui->comboBoxTopicName->setCurrentIndex(selectedIndex);
        selectedTopic = value;
    }
}


#include <rqt_ui_loader/ui_loader.h>

#include <ros/publisher.h>
#include <ros/master.h>

#include <QDebug>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QSlider>
#include <QSpinBox>

UILoaderWidget::UILoaderWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UILoaderWidget),
    widget(NULL)
{
    ui->setupUi(this);
    loadUIFile("/home/mm/test.ui");
}

UILoaderWidget::~UILoaderWidget()
{
    delete ui;
}

void UILoaderWidget::loadUIFile(QString filePath)
{
    if (widget != NULL) {
        ui->gridLayout->removeWidget(widget);
        widget->deleteLater();
    }

    QFile file(filePath);
    file.open(QFile::ReadOnly);
    widget = loader.load(&file, this);
    file.close();

    ui->gridLayout->addWidget(widget);
    setWindowTitle(widget->windowTitle());

    processWidget(widget);
}

void UILoaderWidget::on_toolButtonBrowse_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this,
                                                    tr("Select the UI file"),
                                                    QDir::homePath(),
                                                    tr("Qt UI files (*.ui)")
                                                   );
    if (!filePath.isEmpty()) {
        ui->lineEditFilePath->setText(filePath);

        loadUIFile(filePath);
    }
}

void UILoaderWidget::processWidget(QObject *widget)
{
    // recursive function call to go through the widgets

    if (widget->property("publish_topic") != QVariant::Invalid &&
        widget->property("publish_type") != QVariant::Invalid) {

        if (strcmp(widget->metaObject()->className(), "QSlider") == 0 ||
            strcmp(widget->metaObject()->className(), "QSpinBox") == 0) {
            publishers << new Int32Publisher(&nodeHandle, widget);
        }
        // QAbstractButton

        // QLineEdit
    }

    if (widget->property("subscribe_topic") != QVariant::Invalid &&
        widget->property("subscribe_type") != QVariant::Invalid) {
        qWarning() << widget->property("publish_topic");
        qWarning() << widget->metaObject()->className();
    }

    foreach (QObject *childWidget, widget->children()) {
        processWidget(childWidget);
    }
}

WidgetPublisher::WidgetPublisher(ros::NodeHandle *nodeHandle, QObject *widget_a) :
    QObject(widget_a),
    nh(nodeHandle),
    widget(widget_a)
{

}

void WidgetPublisher::createPublisher(QString typeName)
{
    if (typeName.toLower() == "int32") {
        publisher = nh->advertise<std_msgs::Int32>(widget->property("publish_topic").toString().toStdString(), 1);
    } else if (typeName.toLower() == "string") {
        publisher = nh->advertise<std_msgs::String>(widget->property("publish_topic").toString().toStdString(), 1);
    } else if (typeName.toLower() == "bool" || typeName.toLower() == "boolean") {
        publisher = nh->advertise<std_msgs::Bool>(widget->property("publish_topic").toString().toStdString(), 1);
    }
}

Int32Publisher::Int32Publisher(ros::NodeHandle *nodeHandle, QObject *widget) :
    WidgetPublisher(nodeHandle, widget)
{
    qWarning() << widget->property("publish_topic");
    qWarning() << widget->metaObject()->className();
    if (strcmp(widget->metaObject()->className(), "QSpinBox") == 0) {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(widget);
        connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
        qWarning() << "spinbox";
    } else if (strcmp(widget->metaObject()->className(), "QSlider") == 0) {
        QSlider *slider = qobject_cast<QSlider*>(widget);
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
        qWarning() << "QSlider";
    }

    createPublisher(widget->property("publish_type").toString());
}

Int32Publisher::~Int32Publisher()
{
    publisher.shutdown();
}

void Int32Publisher::valueChanged(int value)
{
    msg.data = value;
    publisher.publish(msg);
}




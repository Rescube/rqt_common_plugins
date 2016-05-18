#include <rqt_ui_loader/ui_loader.h>
#include <rqt_ui_loader/ui_loader_widget.h>


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
}

UILoaderWidget::~UILoaderWidget()
{
    while (subscribers.size()) {
        WidgetSubscriber *subscriber = subscribers.takeFirst();
        delete subscriber;
    }

    while (publishers.size()) {
        WidgetPublisher *publisher = publishers.takeFirst();
        delete publisher;
    }

    delete ui;
}

void UILoaderWidget::loadUIFile(QString filePath)
{
    if (filePath.isEmpty())
        return;

    QFile file(filePath);
    if (file.open(QFile::ReadOnly)) {
        if (widget != NULL) {
            ui->gridLayoutMain->removeWidget(widget);
            widget->deleteLater();
        }

        widget = loader.load(&file, this);
        file.close();

        if (widget == NULL) {
            QMessageBox::critical(
                        this,
                        tr("Unable to load the UI file"),
                        tr("Unable to parse the %1 file!").arg(filePath)
                        );
            return;
        }

        // clean existing contents of the layout!
        if (ui->verticalSpacerMain != NULL) {
            ui->gridLayoutMain->removeItem(ui->verticalSpacerMain);
            delete ui->verticalSpacerMain;
            ui->verticalSpacerMain = NULL;
        }

        ui->gridLayoutMain->addWidget(widget);
        setWindowTitle(widget->windowTitle());

        processWidget(widget);

        ui->lineEditFilePath->setText(filePath);
        m_filePath = filePath;
    } else {
        QMessageBox::critical(
                    this,
                    tr("Unable to open the UI file"),
                    tr("Unable to open the %1 file for reading!").arg(filePath)
                    );
    }
}

void UILoaderWidget::on_toolButtonBrowse_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this,
                                                    tr("Select the UI file"),
                                                    QDir::homePath(),
                                                    tr("Qt UI files (*.ui)")
                                                   );
    if (!filePath.isEmpty()) {
        loadUIFile(filePath);
    }
}

void UILoaderWidget::processWidget(QObject *widget)
{
    // recursive function call to go through the widgets

    if (widget->property("publish_topic") != QVariant::Invalid) {

        if (strcmp(widget->metaObject()->className(), "QSlider") == 0 ||
            strcmp(widget->metaObject()->className(), "QDial") == 0 ||
            strcmp(widget->metaObject()->className(), "QSpinBox") == 0) {
            publishers << new Int32Publisher(&nodeHandle, widget);
        }
        // QAbstractButton

        // QLineEdit
    }

    if (widget->property("subscribe_topic") != QVariant::Invalid) {
        if (strcmp(widget->metaObject()->className(), "QSlider") == 0 ||
            strcmp(widget->metaObject()->className(), "QDial") == 0 ||
            strcmp(widget->metaObject()->className(), "QSpinBox") == 0) {
            subscribers << new Int32Subscriber(&nodeHandle, widget);
        }
    }

    foreach (QObject *childWidget, widget->children()) {
        processWidget(childWidget);
    }
}
QString UILoaderWidget::getFilePath() const
{
    return m_filePath;
}

void UILoaderWidget::setFilePath(const QString &value)
{
    m_filePath = value;
}


WidgetPublisher::WidgetPublisher(ros::NodeHandle *nodeHandle, QObject *widget_a) :
    QObject(widget_a),
    nh(nodeHandle),
    m_widget(widget_a)
{

}

WidgetPublisher::~WidgetPublisher()
{
    publisher.shutdown();
}

void WidgetPublisher::createPublisher(QString typeName)
{
    if (typeName.toLower() == "int32") {
        publisher = nh->advertise<std_msgs::Int32>(m_widget->property("publish_topic").toString().toStdString(), 1);
    } else if (typeName.toLower() == "string") {
        publisher = nh->advertise<std_msgs::String>(m_widget->property("publish_topic").toString().toStdString(), 1);
    } else if (typeName.toLower() == "bool" || typeName.toLower() == "boolean") {
        publisher = nh->advertise<std_msgs::Bool>(m_widget->property("publish_topic").toString().toStdString(), 1);
    }
}

Int32Publisher::Int32Publisher(ros::NodeHandle *nodeHandle, QObject *widget) :
    WidgetPublisher(nodeHandle, widget)
{
    if (strcmp(widget->metaObject()->className(), "QSpinBox") == 0) {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(widget);
        connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
    } else if (strcmp(widget->metaObject()->className(), "QSlider") == 0 ||
               strcmp(widget->metaObject()->className(), "QDial") == 0) {
        QAbstractSlider *slider = qobject_cast<QAbstractSlider*>(widget);
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
    }

    createPublisher("Int32");
}

void Int32Publisher::valueChanged(int value)
{
    msg.data = value;
    publisher.publish(msg);
}


WidgetSubscriber::WidgetSubscriber(ros::NodeHandle *nodeHandle, QObject *widget) :
    QObject(widget)
{
    m_widget = widget;
    m_nodeHandle = nodeHandle;
}

WidgetSubscriber::~WidgetSubscriber()
{
    m_subscriber.shutdown();
}


Int32Subscriber::Int32Subscriber(ros::NodeHandle *nodeHandle, QObject *widget) :
    WidgetSubscriber(nodeHandle, widget)
{
    m_subscriber = nodeHandle->subscribe(widget->property("subscribe_topic").toString().toStdString(), 1, &Int32Subscriber::valueChanged, this);

    if (strcmp(widget->metaObject()->className(), "QSpinBox") == 0) {
        m_type = SpinBox;
        m_spinBox = qobject_cast<QSpinBox*>(widget);
    } else if (strcmp(widget->metaObject()->className(), "QSlider") == 0 ||
               strcmp(widget->metaObject()->className(), "QDial") == 0) {
        m_type = Slider;
        m_slider = qobject_cast<QAbstractSlider*>(widget);
    }
}

void Int32Subscriber::valueChanged(const std_msgs::Int32::ConstPtr& msg)
{
    switch (m_type) {
    case Slider:
        m_slider->setValue(msg->data);
        break;
    case SpinBox:
        m_spinBox->setValue(msg->data);
        break;
    }
}

#include <rqt_ui_loader/ui_loader.h>

#include <ros/publisher.h>
#include <ros/master.h>

#include <QDebug>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QAbstractSlider>


UILoaderWidget::UILoaderWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UILoaderWidget),
    widget(NULL)
{
    ui->setupUi(this);
}

UILoaderWidget::~UILoaderWidget()
{
    delete ui;
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

        if (widget != NULL) {
            ui->gridLayout->removeWidget(widget);
            widget->deleteLater();
        }

        QFile file(filePath);
        file.open(QFile::ReadOnly);
        widget = loader.load(&file, this);
        file.close();

        ui->gridLayout->addWidget(widget);

        processWidget(widget);
    }
}

void UILoaderWidget::processWidget(QObject *widget)
{
    // recursive function call to go through the widgets

    if (widget->property("publish_topic") != QVariant::Invalid &&
        widget->property("publish_type") != QVariant::Invalid) {
        qWarning() << widget->property("publish_topic");
        qWarning() << widget->metaObject()->className();

        // QAbstractSlider
        if (widget->metaObject()->className() == "QSlider") {
            QAbstractSlider *slider = qobject_cast<QAbstractSlider*>(widget);
            connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderPublish));
        }

        // QAbstractSpinBox

        // QAbstractButton
    }

    if (widget->property("subscribe_topic") != QVariant::Invalid) {
        qWarning() << widget->property("publish_topic");
        qWarning() << widget->metaObject()->className();
    }

    foreach (QObject *childWidget, widget->children()) {
        processWidget(childWidget);
    }
}

void UILoaderWidget::sliderPublish()
{
    QObject *sender = this->sender();
    QAbstractSlider *slider = qobject_cast<QAbstractSlider*>(sender);


}

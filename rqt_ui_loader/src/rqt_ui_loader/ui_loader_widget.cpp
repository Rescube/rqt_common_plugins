#include <rqt_ui_loader/ui_loader.h>

#include <ros/publisher.h>
#include <ros/master.h>

#include <QDebug>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>

UILoaderWidget::UILoaderWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UILoaderWidget)
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
    }
}

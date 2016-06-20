#include <rqt_send_file/send_file.h>


#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <QDir>

namespace rqt_send_file {

SendFile::SendFile() :
    rqt_gui_cpp::Plugin(),
    widget(NULL)
{
    setObjectName("SendFile");
}

void SendFile::initPlugin(qt_gui_cpp::PluginContext &context)
{
    widget = new SendFileWidget();
    context.addWidget(widget);
}

void SendFile::shutdownPlugin()
{

}

void SendFile::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
{
    if (widget != NULL) {
        instance_settings.setValue("lastDir", widget->getLastDir());
        instance_settings.setValue("selectedFile", widget->getSelectedFile());
        instance_settings.setValue("selectedTopic", widget->getSelectedTopic());
    }
}

void SendFile::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
{
    if (widget != NULL) {
        QString lastDir = instance_settings.value("lastDir", QDir::homePath()).toString();
        widget->setLastDir(lastDir);

        QString selectedFile = instance_settings.value("selectedFile").toString();
        widget->setSelectedFile(selectedFile);

        QString selectedTopic = instance_settings.value("selectedTopic").toString();
        widget->setSelectedTopic(selectedTopic);
    }
}

SendFile::~SendFile()
{

}

} // end namespace rqt_send_file

PLUGINLIB_EXPORT_CLASS(rqt_send_file::SendFile, rqt_gui_cpp::Plugin)

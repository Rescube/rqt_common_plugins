#include <rqt_ui_loader/ui_loader.h>


#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <QDir>

namespace rqt_ui_loader {

UILoader::UILoader() :
    rqt_gui_cpp::Plugin(),
    widget(NULL)
{
    setObjectName("UILoader");
}

void UILoader::initPlugin(qt_gui_cpp::PluginContext &context)
{
    widget = new UILoaderWidget();
    context.addWidget(widget);
}

void UILoader::shutdownPlugin()
{

}

void UILoader::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
{
    if (widget != NULL) {
        instance_settings.setValue("filePath", widget->getFilePath());
    }
}

void UILoader::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
{
    if (widget != NULL) {
        QString filePath = instance_settings.value("filePath").toString();
        widget->loadUIFile(filePath);
    }
}

UILoader::~UILoader()
{
    // no need to delete the widget here because:
    // PluginContext::addWidget(QWidget* widgets)
    // "The ownership of the widget pointer is transferred to the callee which will delete it when the plugin is shut down."
}

} // end namespace rqt_ui_loader

PLUGINLIB_EXPORT_CLASS(rqt_ui_loader::UILoader, rqt_gui_cpp::Plugin)

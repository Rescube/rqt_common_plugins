#ifndef SENDFILE_H
#define SENDFILE_H


#include <QWidget>
#include <rqt_gui_cpp/plugin.h>

#include <rqt_send_file/send_file_widget.h>

namespace rqt_send_file {

class SendFile : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    SendFile();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    ~SendFile();
private:
    SendFileWidget *widget;
};

} // end namespace rqt_send_file

#endif // SENDFILE_H

// SwarmFlow robot dashboard for the Gazebo GUI (T022).
//
// Subscribes to the gz topic /swarmflow/dashboard (gz.msgs.StringMsg holding JSON, published by swarmflow_viz) and
// shows one row per robot: status colour, robot colour, mode, goal, order, load, speed and the age of its last
// update. The topic can be changed with <topic> in the plugin's config.
#ifndef SWARMFLOW_GZ_PANEL_SWARMFLOWDASHBOARD_HH_
#define SWARMFLOW_GZ_PANEL_SWARMFLOWDASHBOARD_HH_

#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <QString>
#include <QVariantList>

namespace swarmflow
{
class SwarmFlowDashboard : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QVariantList robots READ Robots NOTIFY BoardChanged)
  Q_PROPERTY(QString header READ Header NOTIFY BoardChanged)

public:
  SwarmFlowDashboard();
  ~SwarmFlowDashboard() override;
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  QVariantList Robots() const;
  QString Header() const;

signals:
  void BoardChanged();

private slots:
  void OnJson(const QString & _json);

private:
  void OnMsg(const gz::msgs::StringMsg & _msg);

  gz::transport::Node node;
  std::string topic{"/swarmflow/dashboard"};
  QVariantList robots;
  QString header{"waiting for /swarmflow/dashboard"};
};
}  // namespace swarmflow

#endif  // SWARMFLOW_GZ_PANEL_SWARMFLOWDASHBOARD_HH_

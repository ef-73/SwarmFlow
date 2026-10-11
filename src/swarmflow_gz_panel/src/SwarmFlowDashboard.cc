#include "SwarmFlowDashboard.hh"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace swarmflow
{
SwarmFlowDashboard::SwarmFlowDashboard() = default;

SwarmFlowDashboard::~SwarmFlowDashboard() = default;

void SwarmFlowDashboard::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "SwarmFlow robots";
  }
  if (_pluginElem) {
    if (auto * t = _pluginElem->FirstChildElement("topic"); t && t->GetText()) {
      this->topic = t->GetText();
    }
  }
  if (!this->node.Subscribe(this->topic, &SwarmFlowDashboard::OnMsg, this)) {
    gzerr << "SwarmFlowDashboard: cannot subscribe to [" << this->topic << "]\n";
  }
}

void SwarmFlowDashboard::OnMsg(const gz::msgs::StringMsg & _msg)
{
  // gz-transport thread: hand the text to the Qt thread
  QMetaObject::invokeMethod(
    this, "OnJson", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(_msg.data())));
}

void SwarmFlowDashboard::OnJson(const QString & _json)
{
  const QJsonDocument doc = QJsonDocument::fromJson(_json.toUtf8());
  if (!doc.isObject()) {
    return;
  }
  const QJsonObject root = doc.object();
  QVariantList rows;
  for (const auto & r : root.value("robots").toArray()) {
    rows.append(r.toObject().toVariantMap());
  }
  this->robots = rows;
  this->header = QString("t = %1 s (sim)").arg(root.value("t").toDouble(), 0, 'f', 0);
  emit this->BoardChanged();
}

QVariantList SwarmFlowDashboard::Robots() const
{
  return this->robots;
}

QString SwarmFlowDashboard::Header() const
{
  return this->header;
}
}  // namespace swarmflow

GZ_ADD_PLUGIN(swarmflow::SwarmFlowDashboard, gz::gui::Plugin)

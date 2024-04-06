#include "properties/cyber_channel_property.h"

#include <QDebug>

#include <cyber/service_discovery/topology_manager.h>

using namespace rviz_common;
using namespace rviz_common::properties;

CyberChannelProperty::CyberChannelProperty(const QString &name,
                                           const QString &default_value,
                                           const QString &description,
                                           Property *parent,
                                           const char *changed_slot,
                                           QObject *receiver)
    : EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver) {
  connect(this, SIGNAL(requestOptions(EditableEnumProperty *)), this, SLOT(ListCurrentChannel()));
}

void CyberChannelProperty::ListCurrentChannel() {
  using namespace apollo::cyber::service_discovery;
  std::vector<std::string> channels;
  auto channel_manager = TopologyManager::Instance()->channel_manager();
  channel_manager->GetChannelNames(&channels);

  this->clearOptions();
  for (auto const &name : channels) {
    std::string msg_type = "";
    channel_manager->GetMsgType(name, &msg_type);

    if (message_type_ == msg_type) { this->addOptionStd(name); }

    // qDebug() << "name = " << QString::fromStdString(name) << "type = " << QString::fromStdString(msg_type);
  }

  // qDebug() << QString("MatrixDisplay::ListCurrentChannel() called ,size= %1 ").arg(names.size());
}

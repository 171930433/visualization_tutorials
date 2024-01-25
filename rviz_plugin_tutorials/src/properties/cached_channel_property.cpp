#include "properties/cached_channel_property.h"

#include <QDebug>

#include "cacher/cacher.h"
#include "protobuf_helper.h"
namespace rviz {

CachedChannelProperty::CachedChannelProperty(const QString &name,
                                             const QString &default_value,
                                             const QString &description,
                                             Property *parent,
                                             const char *changed_slot,
                                             QObject *receiver)
    : EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver) {
  connect(this, SIGNAL(requestOptions(EditableEnumProperty *)), this, SLOT(ListCurrentChannel()));
}

void CachedChannelProperty::ListCurrentChannel() {
  auto const names = g_cacher_->GetChannelNames();
  this->clearOptions();
  for (auto const &name : names) {
    this->addOptionStd(name);
    // qDebug() << "name = " << QString::fromStdString(name);
  }

  qDebug() << QString("MatrixDisplay::ListCurrentChannel() called ,size= %1 ").arg(names.size());
}

FieldListProperty::FieldListProperty(const QString &name,
                                     const QString &default_value,
                                     const QString &description,
                                     Property *parent,
                                     const char *changed_slot,
                                     QObject *receiver)
    : EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver) {
  connect(this, SIGNAL(requestOptions(EditableEnumProperty *)), this, SLOT(ListFieldNames()));
}

void FieldListProperty::setChannelProperty(CachedChannelProperty *channel) {
  channel_ = channel;
  connect(channel, SIGNAL(changed()), this, SIGNAL(changed()));
}

QString FieldListProperty::getChannelName() const {
  if (!channel_) { return ""; }
  return channel_->getString();
}

void FieldListProperty::ListFieldNames() {
  // 如果未设定通道名,则退化成通道名
  // ! 需要统一string 和 qstring
  if (!channel_) {
    auto const names = g_cacher_->GetChannelNames();
    this->clearOptions();
    this->addOption("");
    for (auto const &name : names) {
      this->addOptionStd(name);
    }
    return;
  }

  auto channel_name = channel_->getStdString();
  if (channel_name.empty()) { return; }
  auto type_name = g_cacher_->GetTypeNameWithChannelName(channel_name);
  auto msg = CreateMessageByName(type_name);
  auto options = GetFildNames(*msg);

  this->clearOptions();
  this->addOption("");
  for (auto const &name : options) {
    this->addOption(name);
  }
}
} // namespace rviz
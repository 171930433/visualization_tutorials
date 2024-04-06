#pragma once

#include <eigen3/Eigen/Dense>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>

class CyberChannelProperty : public rviz_common::properties::EditableEnumProperty {
  Q_OBJECT
public:
  CyberChannelProperty(const QString &name = QString(),
                       const QString &default_value = QString(),
                       const QString &description = QString(),
                       Property *parent = nullptr,
                       const char *changed_slot = nullptr,
                       QObject *receiver = nullptr);

  void setMessageType(std::string const &message_type) {
    message_type_ = message_type;
    this->setDescription(QString("message_type is %1").arg(QString::fromStdString(message_type_)));
  }
  QString getMessageType() const { return QString::fromStdString(message_type_); }

  QString getTopic() const { return getValue().toString(); }
  std::string getTopicStd() const { return getValue().toString().toStdString(); }

private Q_SLOTS:
  void ListCurrentChannel(); // 列举当前cacher中的所有通道到combox中
private:
  std::string message_type_;
};

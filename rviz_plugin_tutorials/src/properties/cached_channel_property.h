#pragma once
#include <eigen3/Eigen/Dense>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>

namespace rviz_common {
namespace properties {

class CachedChannelProperty : public EditableEnumProperty {
  Q_OBJECT
public:
  CachedChannelProperty(const QString &name = QString(),
                        const QString &default_value = QString(),
                        const QString &description = QString(),
                        Property *parent = nullptr,
                        const char *changed_slot = nullptr,
                        QObject *receiver = nullptr);
private Q_SLOTS:
  void ListCurrentChannel(); // 列举当前cacher中的所有通道到combox中
};

class FieldListProperty : public EditableEnumProperty {
  Q_OBJECT
public:
  FieldListProperty(const QString &name = QString(),
                    const QString &default_value = QString(),
                    const QString &description = QString(),
                    Property *parent = nullptr,
                    const char *changed_slot = nullptr,
                    QObject *receiver = nullptr);
  void setChannelProperty(CachedChannelProperty *channel);
  QString getChannelName() const;
protected Q_SLOTS:
  void ListFieldNames(); // 列举当前通道的类型的所有字段名称
protected:
  CachedChannelProperty *channel_;
};


using VectorXChannel = std::vector<std::shared_ptr<rviz_common::properties::CachedChannelProperty>>;


} // namespace properties
} // namespace rviz_common


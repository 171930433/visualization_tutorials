#pragma once
#include <eigen3/Eigen/Dense>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/property.h>

namespace rviz {
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
private Q_SLOTS:
  void ListFieldNames(); // 列举当前通道的类型的所有字段名称
protected:
  CachedChannelProperty *channel_;
};

using MatrixXFieldList = Eigen::Matrix<std::shared_ptr<rviz::FieldListProperty>, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXChannel = Eigen::Matrix<std::shared_ptr<rviz::CachedChannelProperty>, Eigen::Dynamic, Eigen::Dynamic>;
} // namespace rviz

#include "properties/longitude_latitude_property.h"
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>

#include <QDebug>

namespace rviz_common {
namespace properties {

LatLonProperty::LatLonProperty(const QString &name,
                               Eigen::Vector3d const &default_value,
                               const QString &description,
                               Property *parent,
                               const char *changed_slot,
                               QObject *receiver)
    : Property(name, QVariant(), description, parent, changed_slot, receiver), vector_(default_value),
      ignore_child_updates_(false) {
  // 单位
  units_ = new EnumProperty("units", "rad,rad,meter", "units of llh", this, SLOT(UpdateUnits()));
  units_->addOption("rad,rad,meter", LlhUnits::RadRadMeter);
  units_->addOption("deg,deg,meter", LlhUnits::DegDegMeter);
  vector_with_units_ = vector_.array() * units_vector_.array();
  //
  x_ = new Property("Latitude", vector_.x(), "X coordinate", this);
  y_ = new Property("Longitude", vector_.y(), "Y coordinate", this);
  z_ = new Property("Height", vector_.z(), "Z coordinate", this);
  updateString();
  connect(x_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(y_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(z_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(x_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
  connect(y_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
  connect(z_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
}

void LatLonProperty::UpdateUnits() {
  switch ((LlhUnits)units_->getOptionInt()) {
  case LlhUnits::RadRadMeter:
    units_vector_.setOnes();
    break;
  case LlhUnits::DegDegMeter:
    units_vector_ = Eigen::Vector3d(180.0 / M_PI, 180.0 / M_PI, 1);

    break;
  default:
    units_vector_.setOnes();
    break;
  }
  // 不会触发changed消息
  vector_with_units_ = vector_.array() * units_vector_.array();
  ignore_child_updates_ = true;
  x_->setValue(vector_with_units_.x());
  y_->setValue(vector_with_units_.y());
  z_->setValue(vector_with_units_.z());
  ignore_child_updates_ = false;
  updateString();

  // qDebug() << QString("%1; %2; %3").arg(vector_.x(), 0, 'f', 8).arg(vector_.y(), 0, 'f', 8).arg(vector_.z(), 0, 'f', 8);
}

// new_vector rad rad m
bool LatLonProperty::setVector(Eigen::Vector3d const &new_vector) {
  if (new_vector != vector_) {
    Q_EMIT aboutToChange();
    vector_ = new_vector; // 实际存储 rad rad m
    vector_with_units_ = vector_.array() * units_vector_.array();
    ignore_child_updates_ = true;
    x_->setValue(vector_with_units_.x());
    y_->setValue(vector_with_units_.y());
    z_->setValue(vector_with_units_.z());
    ignore_child_updates_ = false;
    updateString();
    Q_EMIT changed();
    if (model_) model_->emitDataChanged(this);
    return true;
  }
  return false;
}

bool LatLonProperty::setValue(const QVariant &new_value) {
  QStringList strings = new_value.toString().split(';');
  if (strings.size() >= 3) {
    bool x_ok = true;
    double x = strings[0].toDouble(&x_ok);
    bool y_ok = true;
    double y = strings[1].toDouble(&y_ok);
    bool z_ok = true;
    double z = strings[2].toDouble(&z_ok);
    if (x_ok && y_ok && z_ok) {
      Eigen::Vector3d new_value_with_units = {x, y, z};
      if (!new_value_with_units.isApprox(vector_with_units_, 1e-8)) {
        return setVector(new_value_with_units.array() / units_vector_.array());
      }
    }
  }
  return false;
}

void LatLonProperty::updateFromChildren() {
  if (!ignore_child_updates_) {
    vector_with_units_ =
        Eigen::Vector3d{x_->getValue().toDouble(), y_->getValue().toDouble(), z_->getValue().toDouble()};
    vector_ = vector_with_units_.array() / units_vector_.array();
    updateString();
    Q_EMIT changed();
  }
}

void LatLonProperty::emitAboutToChange() {
  if (!ignore_child_updates_) { Q_EMIT aboutToChange(); }
}

void LatLonProperty::updateString() {
  value_ = QString("%1; %2; %3")
               .arg(vector_with_units_.x(), 0, 'f', 8)
               .arg(vector_with_units_.y(), 0, 'f', 8)
               .arg(vector_with_units_.z(), 0, 'f', 3);
}

void LatLonProperty::load(const Config &config) {
  QString unit = "";
  config.mapGetString("units", &unit);
  units_->setValue(unit);

  float x, y, z;
  if (config.mapGetFloat("Latitude", &x) && config.mapGetFloat("Longitude", &y) && config.mapGetFloat("Height", &z)) {
    // Calling setVector() once explicitly is better than letting the
    // Property class load the X, Y, and Z children independently,
    // which would result in at least 3 calls to setVector().
    Eigen::Vector3d new_value_with_units{x, y, z};
    setVector(new_value_with_units.array() / units_vector_.array());
  }
}

void LatLonProperty::save(Config config) const {
  // Saving the child values explicitly avoids having Property::save()
  // save the summary string version of the property.
  // ! 保存时,以rad rad m为单位保存,否则默认的float可能有精度损失
  config.mapSetValue("units", units_->getValue());
  config.mapSetValue("Latitude", x_->getValue());
  config.mapSetValue("Longitude", y_->getValue());
  config.mapSetValue("Height", z_->getValue());
}

void LatLonProperty::setReadOnly(bool read_only) {
  Property::setReadOnly(read_only);
  x_->setReadOnly(read_only);
  y_->setReadOnly(read_only);
  z_->setReadOnly(read_only);
}

} // namespace properties
} // namespace rviz_common

namespace rviz {}
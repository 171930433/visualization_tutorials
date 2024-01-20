#pragma once
#include <rviz/properties/property.h>
#include <eigen3/Eigen/Dense>

namespace rviz
{
  class EnumProperty;

  enum LlhUnits
  {
    RadRadMeter = 0, //
    DegDegMeter      //
  };

  class LatLonProperty : public Property
  {

    Q_OBJECT
  public:
    LatLonProperty(const QString &name = QString(),
                   Eigen::Vector3d const &default_value = Eigen::Vector3d::Zero(),
                   const QString &description = QString(),
                   Property *parent = nullptr,
                   const char *changed_slot = nullptr,
                   QObject *receiver = nullptr);

    virtual bool setVector(Eigen::Vector3d const &vector);
    virtual Eigen::Vector3d getVector() const
    {
      return vector_;
    }
    bool add(Eigen::Vector3d const &offset)
    {
      return setVector(getVector() + offset);
    }

    bool setValue(const QVariant &new_value) override;

    void load(const Config &config) override;
    void save(Config config) const override;

    /** @brief Overridden from Property to propagate read-only-ness to children. */
    void setReadOnly(bool read_only) override;

  private Q_SLOTS:
    void updateFromChildren();
    void emitAboutToChange();
    void UpdateUnits();

  private:
    void updateString();

    Eigen::Vector3d vector_ = Eigen::Vector3d::Zero(); // 实际存储 rad rad m, 修改单位仅仅修改显示
    Eigen::Vector3d vector_with_units_;                // 显示的数值
    // units
    EnumProperty *units_;
    Eigen::Vector3d units_vector_ = Eigen::Vector3d::Ones();

    // 实际存储类型,单位为 [rad,rad,m]
    Property *x_;
    Property *y_;
    Property *z_;

    bool ignore_child_updates_;
  };

}
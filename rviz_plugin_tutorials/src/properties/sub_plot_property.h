#pragma once
#include "plot/qcustomplot.h"
#include "properties/cached_channel_property.h"
#include <eigen3/Eigen/Dense>

namespace rviz {

template <typename _T> class SubPlotProperty : public FieldListProperty {
public:
  SubPlotProperty(const QString &name = QString(),
                  const QString &default_value = QString(),
                  const QString &description = QString(),
                  Property *parent = nullptr,
                  const char *changed_slot = nullptr,
                  QObject *receiver = nullptr)
      : FieldListProperty(name, default_value, description, parent, changed_slot, receiver) {}

public:
  std::shared_ptr<_T> &graph() { return graph_; };

protected:
  std::shared_ptr<_T> graph_ = nullptr;
};

using SubGraphProperty = SubPlotProperty<QCPGraph>;
using SubCurveProperty = SubPlotProperty<QCPCurve>;

using MatrixXSubGraph = Eigen::Matrix<std::shared_ptr<SubGraphProperty>, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXSubCurve = Eigen::Matrix<std::shared_ptr<SubCurveProperty>, Eigen::Dynamic, Eigen::Dynamic>;

} // namespace rviz

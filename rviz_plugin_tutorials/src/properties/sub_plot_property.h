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

template <typename T> class MatrixX {
public:
  using spPointer = std::shared_ptr<T>;
  MatrixX(Property *parent = nullptr) : parent_(parent) {}

public:
  void resize(int const row, int const col) {
    qDebug() << QString(" resize begin from [%1,%2] to [%3,%4]").arg(rows()).arg(cols()).arg(row).arg(col);
    elems_.resize(row);
    for (auto &one_col : elems_) {
      one_col.resize(col);
    }
    qDebug() << QString(" resize end from [%1,%2] to [%3,%4]").arg(rows()).arg(cols()).arg(row).arg(col);
  }
  int size() const { return rows() * cols(); }
  int rows() const { return elems_.size(); }
  int cols() const { return elems_.empty() ? 0 : elems_[0].size(); }
  spPointer &operator()(int const row, int const col) { return elems_[row][col]; }

protected:
  Property *parent_;
  std::vector<std::vector<spPointer>> elems_;
};

using MatrixXSubGraph = MatrixX<SubGraphProperty>;
using VectorXSubGraph = std::vector<std::shared_ptr<SubGraphProperty>>;
// using MatrixXSubGraph = Eigen::Matrix<std::shared_ptr<SubGraphProperty>, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXSubCurve = Eigen::Matrix<std::shared_ptr<SubCurveProperty>, Eigen::Dynamic, Eigen::Dynamic>;

} // namespace rviz

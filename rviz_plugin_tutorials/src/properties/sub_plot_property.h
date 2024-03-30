#pragma once
#include "plot/qcustomplot.h"
#include "properties/cached_channel_property.h"
#include <eigen3/Eigen/Dense>

namespace rviz_common {
namespace properties {
class EnumProperty;
class ColorProperty;
class IntProperty;

class PlotableProperty : public QObject {
  Q_OBJECT
public:
  PlotableProperty(Property *parent = nullptr);

  QPen getLinePen() const { return ls_.pen_; }
  QCPScatterStyle getScatterStyle() const { return ss_; }

  struct LineStyle {
    int style_;
    QPen pen_;
  };

Q_SIGNALS:
  void ScatterTypeChanged(QCPScatterStyle);
  void LineTypeChanged(LineStyle);
private Q_SLOTS:
  void UpdateScatter();
  void UpdateLine();

protected:
  EnumProperty *scatter_type_ = nullptr;
  ColorProperty *scatter_color_ = nullptr; // scatter color
  IntProperty *scatter_size_ = nullptr;    // scatter size
  // line
  EnumProperty *line_type_ = nullptr;   // line type
  IntProperty *line_width_ = nullptr;   // line width
  ColorProperty *line_color_ = nullptr; // line color
  //
  LineStyle ls_;
  QCPScatterStyle ss_;
};
template <typename _T> class SubPlotProperty : public FieldListProperty {
public:
  SubPlotProperty(const QString &name = QString(),
                  const QString &default_value = QString(),
                  const QString &description = QString(),
                  Property *parent = nullptr,
                  const char *changed_slot = nullptr,
                  QObject *receiver = nullptr)
      : FieldListProperty(name, default_value, description, parent, changed_slot, receiver) {
    style_prop_ = new PlotableProperty(this);
    connect(style_prop_, &PlotableProperty::ScatterTypeChanged, this, &SubPlotProperty::onScatterTypeChanged);
    connect(style_prop_, &PlotableProperty::LineTypeChanged, this, &SubPlotProperty::onLineTypeChanged);
  }

public:
  std::shared_ptr<_T> &graph() { return graph_; };
  QPen getLinePen() const { return style_prop_->getLinePen(); }
  QCPScatterStyle getScatterStyle() const { return style_prop_->getScatterStyle(); }

protected:
  void onScatterTypeChanged(QCPScatterStyle const &ss) {
    if (graph_) {
      graph_->setScatterStyle(ss);
      graph_->parentPlot()->replot();
    }
  }
  void onLineTypeChanged(PlotableProperty::LineStyle const ls) {
    if (graph_) {
      graph_->setPen(ls.pen_);
      graph_->setLineStyle(typename _T::LineStyle(ls.style_));
      graph_->parentPlot()->replot();
    }
  }

protected:
  std::shared_ptr<_T> graph_ = nullptr;
  PlotableProperty *style_prop_;
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
using MatrixXSubCurve = MatrixX<SubCurveProperty>;
using VectorXSubGraph = std::vector<std::shared_ptr<SubGraphProperty>>;
using VectorXSubCurve = std::vector<std::shared_ptr<SubCurveProperty>>;

} // namespace properties

} // namespace rviz_common


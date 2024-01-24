#include <QWidget>

#include "display_sync_base.h"
#include "plot/plot_base.h"
#include <rviz/display_context.h>

void FocusByIndex(QCPAbstractPlottable *single_graph, int const dataIndex) {
  double const x = single_graph->interface1D()->dataMainKey(dataIndex);
  double const y = single_graph->interface1D()->dataMainValue(dataIndex);

  QCPAxis *x_axis = single_graph->keyAxis();
  QCPAxis *y_axis = single_graph->valueAxis();

  x_axis->setRange(x, x_axis->range().size(), Qt::AlignCenter);
  y_axis->setRange(y, y_axis->range().size(), Qt::AlignCenter);
}

void SelectByT0(QCPAbstractPlottable *single_graph, double const t0) {
  auto const dataIndex = single_graph->interface1D()->findBegin(t0, true);
  // 选中点
  QCPDataRange index_range{dataIndex, dataIndex + 1};
  single_graph->setSelection(QCPDataSelection{index_range});
}

// 选中区间
void SelectByT0sT0e(QCPAbstractPlottable *single_graph, double const t0_s, double const t0_e) {
  auto const dataIndex_s = single_graph->interface1D()->findBegin(t0_s, true);
  auto const dataIndex_e = single_graph->interface1D()->findBegin(t0_e, true);
  // 选中点
  QCPDataRange index_range{dataIndex_s, dataIndex_e + 1};
  single_graph->setSelection(QCPDataSelection{index_range});
}

PlotBase::PlotBase(QWidget *parent) : QCustomPlot(parent) {
  //
  this->setVisible(false);
  dateTicker_ = QSharedPointer<QCPAxisTickerDateTime>(new QCPAxisTickerDateTime);
  dateTicker_->setDateTimeFormat("HH:mm:ss");

  connect(this, SIGNAL(selectionChangedByUser()), this, SLOT(onSelectionChangedByUser()));
}

void PlotBase::resizeEvent(QResizeEvent *event) {
  QCustomPlot::resizeEvent(event);
  this->replot();
}

void PlotBase::keyPressEvent(QKeyEvent *event) {
  QCustomPlot::keyPressEvent(event);

  if (event->key() == Qt::Key_S) {
    if (this->selectionRectMode() == QCP::SelectionRectMode::srmSelect) {
      this->setSelectionRectMode(QCP::SelectionRectMode::srmNone);
      QWidget::setCursor(Qt::ArrowCursor);
    } else {
      this->setSelectionRectMode(QCP::SelectionRectMode::srmSelect);
      QWidget::setCursor(Qt::CrossCursor);
    }
  }
}

void PlotBase::mouseMoveEvent(QMouseEvent *event) {
  QCustomPlot::mouseMoveEvent(event);

  QString str;

  for (auto *single_rect : this->axisRects()) {
    if (single_rect->rect().contains(event->pos())) {
      double const x = single_rect->axis(QCPAxis::atBottom)->pixelToCoord(event->pos().x());
      double const y = single_rect->axis(QCPAxis::atLeft)->pixelToCoord(event->pos().y());
      str = QString("mouse pos = [%1,%2]").arg(x, 0, 'f', 3).arg(y, 0, 'f', 8);
      getDisplaySync()->getContext()->setStatus(str);
    }
  }
}

void PlotBase::FouseRange(QCPRange const &time_range) {
  for (auto *single_rect : this->axisRects()) {
    for (auto *single_plotable : single_rect->plottables()) {
      SelectByT0sT0e(single_plotable, time_range.lower, time_range.upper);
    }
  }

  this->replot();
}

void PlotBase::FocusPoint(double const t0) {
  for (auto *single_rect : this->axisRects()) {
    for (auto *single_plotable : single_rect->plottables()) {
      SelectByT0(single_plotable, t0);
    }
  }

  this->replot();
}

void PlotBase::onSelectionChangedByUser() {
  qDebug() << "PlotBase onSelectionChangedByUser begin";

  QList<QCPAbstractPlottable *> selected_plotable = this->selectedPlottables();

  if (selected_plotable.size() <= 0) {
    qDebug() << " no serials select";
    return;
  }

  if (selected_plotable.size() > 1) {
    qDebug() << " select more than one serials";
    return;
  }

  QCPAbstractPlottable *single_plot = selected_plotable.first();
  QCPDataRange index_range = single_plot->selection().dataRange(0);
  double t0_s = single_plot->interface1D()->dataSortKey(index_range.begin());
  double t0_e = single_plot->interface1D()->dataSortKey(index_range.end());

  for (auto *single_rect : this->axisRects()) {
    for (auto *single_plotable : single_rect->plottables()) {
      if (index_range.size() == 1) {
        SelectByT0(single_plotable, t0_s);
      } else if (index_range.size() > 1) {
        SelectByT0sT0e(single_plotable, t0_s, t0_e);
      }
    }
  }

  if (index_range.size() == 1) {
    this->onFocusPoint(t0_s, false, true);
  } else if (index_range.size() > 1) {
    QCPRange time_range{t0_s, t0_e};
    this->onFouseRange(time_range, false, true);
  }

  this->replot();
  qDebug() << "PlotBase onSelectionChangedByUser end";
}

std::shared_ptr<QCPGraph> PlotBase::CreateDefaultGraph(QCPAxisRect *rect) {
  auto *curve = this->addGraph(rect->axis(QCPAxis::atBottom), rect->axis(QCPAxis::atLeft));

  curve->setSelectable(QCP::stDataRange);

  // 设置散点样式和颜色
  curve->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ScatterShape::ssCross, Qt::blue));

  // 设置直线样式
  curve->setLineStyle(QCPGraph::LineStyle::lsLine);
  QPen lp;
  lp.setWidthF(2);
  lp.setColor(Qt::gray);
  curve->setPen(lp);

  // 定制选中样式
  QCPSelectionDecorator *decorator = curve->selectionDecorator();
  QCPScatterStyle selectedScatterStyle = decorator->scatterStyle();
  selectedScatterStyle.setSize(10); // 选中点的大小
  decorator->setScatterStyle(selectedScatterStyle,
                             QCPScatterStyle::ScatterProperty::spSize); // 只有size使用设定值，其他的用plot的继承值

  // 需要考虑资源回收
  auto when_delete = [this](QCPGraph *elem) { this->removeGraph(elem); };
  std::shared_ptr<QCPGraph> result(curve);

  return result;
}
QCPAxisRect *PlotBase::CreateDefaultRect() {
  QCPAxisRect *rect = new QCPAxisRect(this);
  qDebug() << QString("begin CreateDefaultRect done");

  rect->axis(QCPAxis::atLeft)->setLabel(QString("rect-%1").arg(this->axisRectCount()));
  // 默认缩放y轴
  rect->setRangeZoom(Qt::Vertical);
  // x轴样式
  rect->axis(QCPAxis::atBottom)->setTicker(dateTicker_);
  // y轴label在axis内侧
  rect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  rect->axis(QCPAxis::atLeft)->setSubTicks(false);
  // 共x轴 (0,0)->所有 所有->(0,0)
  if (this->graphCount() != 0) {
    connect(rect->axis(QCPAxis::atBottom),
            SIGNAL(rangeChanged(QCPRange)),
            this->axisRect(0)->axis(QCPAxis::atBottom),
            SLOT(setRange(QCPRange)));
    connect(this->axisRect(0)->axis(QCPAxis::atBottom),
            SIGNAL(rangeChanged(QCPRange)),
            rect->axis(QCPAxis::atBottom),
            SLOT(setRange(QCPRange)));
  }
  foreach (QCPAxis *axis, rect->axes()) {
    axis->setLayer("axes");
    axis->grid()->setLayer("grid");
  }
  //
  qDebug() << QString("CreateDefaultRect done");
  // 创建默认序列
  // auto *curve = CreateDefaultGraph(rect);
  return rect;
}

void PlotBase::setupMatrixDemo(int row, int col) {
  this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iMultiSelect | QCP::iSelectAxes |
                        QCP::iSelectPlottables);
  this->setMultiSelectModifier(Qt::KeyboardModifier::ControlModifier);

  this->plotLayout()->clear(); // let's start from scratch and remove the default axis rect
  this->clearPlottables();

  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      QCPAxisRect *current_rect = CreateDefaultRect();
      this->plotLayout()->addElement(i, j, current_rect);
      //
      // x轴不显示
      if (i != row - 1) {
        current_rect->axis(QCPAxis::atBottom)->setTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setSubTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setTickLabels(false);
      }
      current_rect->setAutoMargins(QCP::MarginSide::msAll);
      current_rect->setMinimumMargins(QMargins{0, 0, 0, 0});
    }
  }
  this->replot();
}
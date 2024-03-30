#include <QWidget>

#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

void SyncCachedData(QCPGraph *curve, std::map<size_t, sp_cPbMsg> const &new_data) {
  for (auto const &kv : new_data) {
    auto const &message = *kv.second;
    double const time_index = kv.first / 1e3;
    double const y = GetValueByHeaderName(message, curve->name()).toDouble();
    curve->addData(time_index, y);
  }
}

MatrixWidget::MatrixWidget(QWidget *parent) : PlotBase(parent) {
  type_ = Type::Matrix;

  setupMatrixDemo(0, 0);
  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel()));
}

void MatrixWidget::ShowSubplot(int const index) {
  // auto rect_show = this->plotLayout()->take(all_rects_[index]);
  // // qDebug() <<  "all_rects_.rows() = " << all_rects_.rows();
  // if (rect_show) {
  //   all_rects_[index]->setVisible(false);
  //   this->plotLayout()->simplify();
  // } else {
  //   all_rects_[index]->setVisible(true);
  //   int current_index = -1; // 因为每次删除元素后，剩余元素会重新编号，所以需要重新计算当前rect的实际index
  //   for (int i = 0; i <= index; ++i) {
  //     if (all_rects_[i]->visible()) { current_index++; }
  //   }

  //   // qDebug() << "current_index == " << current_index;

  //   this->plotLayout()->insertRow(current_index);
  //   this->plotLayout()->addElement(current_index, 0, all_rects_[index]);
  // }

  this->replot();
}

void MatrixWidget::contextMenuRequest(QPoint pos) {
  QMenu *menu = new QMenu(this);
  menu->setAttribute(Qt::WA_DeleteOnClose);

  menu->addAction("X plot", this, [this] { ShowSubplot(0); });
  menu->addAction("Y plot", this, [this] { ShowSubplot(1); });
  menu->addAction("Z plot", this, [this] { ShowSubplot(2); });

  menu->popup(this->mapToGlobal(pos));
}

void MatrixWidget::mouseWheel() {
  // bool x_selected = false;

  // for (auto rect : this->axisRects()) {
  //   if (rect->axis(QCPAxis::atBottom)->selectedParts().testFlag(QCPAxis::spAxis)) {
  //     x_selected = true;
  //     break;
  //   }
  // }

  // for (auto rect : this->axisRects()) {
  //   if (x_selected) {
  //     rect->setRangeZoom(Qt::Horizontal);
  //   } else {
  //     rect->setRangeZoom(Qt::Vertical);
  //   }
  // }

  // qDebug() <<" x_selected = " << x_selected;
}

void MatrixWidget::CreatePlot(QString const &name, MatrixXQString const &field_names) {
  //
  int const row = field_names.rows(), col = field_names.cols();
  qDebug() << QString("start row=%1 col=%2").arg(row).arg(col);
  setupMatrixDemo(row, col);
  qDebug() << QString("end row=%1 col=%2").arg(row).arg(col);

  // this->rescaleAxes();
  // for (int i = 0; i < row; i++) {
  //   for (int j = 0; j < col; j++) {
  //     CreateGraphByFieldName(i, j, field_names(i, j));
  //   }
  // }

  qDebug() << QString("end filed size =%1").arg(field_names.size());
}

std::shared_ptr<QCPGraph> MatrixWidget::CreateGraphByFieldName(int const row,
                                                               int const col,
                                                               QString const &channel_name,
                                                               QString const &field_name) {
  qDebug() << QString("CreateGraphByFieldName start, channel = %1,field = %2, row=%3, col=%4")
                  .arg(channel_name)
                  .arg(field_name)
                  .arg(row)
                  .arg(col);

  auto *rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(row, col));

  std::shared_ptr<QCPGraph> curve = CreateDefaultGraph(rect, channel_name);

  rect->axis(QCPAxis::atLeft)->setLabel(field_name);
  curve->setName(field_name);

  qDebug() << QString("CreateGraphByFieldName end %1").arg(field_name);
  // 检查已经缓存的数据,是否需要更新

  if (channel_msgs_.empty()) { return curve; }

  for (auto const &kv : channel_msgs_[channel_name.toStdString()]) {
    auto const &message = *kv.second;
    double const time_index = kv.first / 1e3;
    double const y = GetValueByHeaderName(message, field_name).toDouble();
    curve->addData(time_index, y);
  }
  qDebug() << QString("CreateGraphByFieldName fist time %1, size = %2")
                  .arg(field_name)
                  .arg(curve->interface1D()->dataCount());

  this->rescaleAxes();
  // this->replot();

  return curve;
}

void MatrixWidget::UpdatePlotLayout(int const new_row, int const new_col) {
  int const old_row = all_rects_.rows();
  int const old_col = all_rects_.cols();

  RowColChanged(new_row, new_col);
  qDebug() << QString("from %1*%2 ------------> %3*%4 rects_ size=%5*%6")
                  .arg(old_row)
                  .arg(old_col)
                  .arg(new_row)
                  .arg(new_col)
                  .arg(all_rects_.rows())
                  .arg(all_rects_.cols());

  for (int i = 0; i < new_row; i++) {
    for (int j = 0; j < new_col; j++) {
      // QCPAxisRect *current_rect = rects_(i, j);
      QCPAxisRect *current_rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(i, j));
      //
      // x轴不显示
      if (i != new_row - 1) {
        current_rect->axis(QCPAxis::atBottom)->setTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setSubTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setTickLabels(false);
      }
      current_rect->setAutoMargins(QCP::MarginSide::msAll);
      current_rect->setMinimumMargins(QMargins{0, 0, 0, 0});
    }
  }

  this->plotLayout()->simplify();
  this->replot();
}

double MatrixWidget::getLastDataTime(std::string const &channel_name) const {
  if (channel_msgs_.find(channel_name) == channel_msgs_.end()) { return 0; }
  return channel_msgs_.at(channel_name).rbegin()->first / 1e3;
}

void MatrixWidget::AddNewData(std::string const &channel_name,
                              std::map<size_t, sp_cPbMsg> const &new_data,
                              int const channel_index) {
  qDebug() << QString("begin MatrixWidget::AddNewData channel_name=%1").arg(QString::fromStdString(channel_name));

  for (auto const &kv : new_data) {
    auto const &message = *kv.second;
    double const time_index = kv.first / 1e3;

    auto single_channel_graphs = channel_graph_[QString::fromStdString(channel_name)];
    for (auto *single_graph : single_channel_graphs) {
      double const y = GetValueByHeaderName(message, single_graph->name()).toDouble();
      single_graph->addData(time_index, y);
    }
  }
  if (new_data.size()) {
    channel_msgs_[channel_name].insert(new_data.begin(), new_data.end());
    this->rescaleAxes();
  }
  qDebug() << QString("MatrixWidget::AddNewData added %1 points").arg(new_data.size());
}

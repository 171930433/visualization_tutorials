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
  auto rect_show = this->plotLayout()->take(all_rects_[index]);
  // qDebug() <<  "this->plotLayout()->rowCount() = " << this->plotLayout()->rowCount();
  if (rect_show) {
    all_rects_[index]->setVisible(false);
    this->plotLayout()->simplify();
  } else {
    all_rects_[index]->setVisible(true);
    int current_index = -1; // 因为每次删除元素后，剩余元素会重新编号，所以需要重新计算当前rect的实际index
    for (int i = 0; i <= index; ++i) {
      if (all_rects_[i]->visible()) { current_index++; }
    }

    // qDebug() << "current_index == " << current_index;

    this->plotLayout()->insertRow(current_index);
    this->plotLayout()->addElement(current_index, 0, all_rects_[index]);
  }
  // for (auto [key, rect] : all_rects_)
  // {
  //   qDebug() << "rect" << key << " " << rect->visible();
  // }
  // qDebug() <<  "end rowCount() = " << this->plotLayout()->rowCount();

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
  bool x_selected = false;

  for (auto rect : this->axisRects()) {
    if (rect->axis(QCPAxis::atBottom)->selectedParts().testFlag(QCPAxis::spAxis)) {
      x_selected = true;
      break;
    }
  }

  for (auto rect : this->axisRects()) {
    if (x_selected) {
      rect->setRangeZoom(Qt::Horizontal);
    } else {
      rect->setRangeZoom(Qt::Vertical);
    }
  }

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

void MatrixWidget::CreateGraphByFieldName(int const row, int const col, QString const &field_name) {
  qDebug() << QString("CreateGraphByFieldName start %1, row=%2, col=%3").arg(field_name).arg(row).arg(col);

  auto *rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(row, col));

  QCPGraph *curve = CreateDefaultGraph(rect);

  rect->axis(QCPAxis::atLeft)->setLabel(field_name);
  curve->setName(field_name);
  this->rescaleAxes();

  qDebug() << QString("CreateGraphByFieldName end %1").arg(field_name);
  // 检查已经缓存的数据,是否需要更新

  if (channel_msgs_.empty()) { return; }

  for (auto const &kv : channel_msgs_.begin()->second) {
    auto const &message = *kv.second;
    double const time_index = kv.first / 1e3;
    double const y = GetValueByHeaderName(message, curve->name()).toDouble();
    curve->addData(time_index, y);
  }
  qDebug() << QString("CreateGraphByFieldName fist time %1, size = %2").arg(field_name).arg(curve->interface1D()->dataCount());

  this->replot();
}

void MatrixWidget::RowChanged(int const new_row) {
  int const old_row = this->plotLayout()->rowCount();
  int const col = this->plotLayout()->columnCount();
  qDebug() << QString("begin RowChanged, col=%3, %1--->%2").arg(old_row).arg(new_row).arg(col);

  // 增加
  if (old_row < new_row) {
    // rects_.conservativeResize(new_row, col);
    for (int i = old_row; i < new_row; i++) {
      for (int j = 0; j < col; j++) {
        QCPAxisRect *current_rect = CreateDefaultRect();
        bool re = this->plotLayout()->addElement(i, j, current_rect);
      }
    }
  }
  // 减少
  else if (old_row > new_row) {
    for (int i = new_row; i < old_row; i++) {
      for (int j = 0; j < col; j++) {
        QCPAxisRect *current_rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(i, j));
        for (auto *one_graph : current_rect->graphs()) {
          this->removeGraph(one_graph);
        }
        this->plotLayout()->remove(current_rect);
      }
    }
    // rects_.conservativeResize(new_row, col);
  }
  qDebug() << QString("end RowChanged %1--->%2").arg(old_row).arg(new_row);
}
void MatrixWidget::ColChanged(int const new_col) {
  //! 因为0,0 --> 1,1 是行和列单独变化的,
  int const row = (this->plotLayout()->rowCount() < 1 ? 1 : this->plotLayout()->rowCount());
  int const old_col = this->plotLayout()->columnCount();
  qDebug() << QString("begin ColChanged, row=%3, %1--->%2").arg(old_col).arg(new_col).arg(row);

  // 增加
  if (old_col < new_col) {
    // rects_.conservativeResize(row, new_col);
    // qDebug() << QString("rects_ %1*%2").arg(this->plotLayout()->rowCount()).arg(this->plotLayout()->columnCount());
    for (int i = 0; i < row; i++) {
      for (int j = old_col; j < new_col; j++) {
        QCPAxisRect *current_rect = CreateDefaultRect();
        bool re = this->plotLayout()->addElement(i, j, current_rect);
      }
    }
  }
  // 减少
  else if (old_col > new_col) {
    for (int i = 0; i < row; i++) {
      for (int j = new_col; j < old_col; j++) {
        QCPAxisRect *current_rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(i, j));
        for (auto *one_graph : current_rect->graphs()) {
          this->removeGraph(one_graph);
        }
        this->plotLayout()->remove(current_rect);
      }
    }
    // rects_.conservativeResize(row, new_col);
  }
  qDebug() << QString("end ColChanged %1--->%2").arg(old_col).arg(new_col);
}

void MatrixWidget::UpdatePlotLayout(int const new_row, int const new_col) {
  int const old_row = this->plotLayout()->rowCount();
  int const old_col = this->plotLayout()->columnCount();
  if (new_row < 1 || new_col < 1) { return; }
  // int const old_row = this->plotLayout()->rowCount();
  // int const old_col = rects_.cols();
  if (new_row != old_row) { RowChanged(new_row); }
  if (old_col != new_col) { ColChanged(new_col); }

  qDebug() << QString("from %1*%2 ------------> %3*%4 rects_ size=%5*%6")
                  .arg(old_row)
                  .arg(old_col)
                  .arg(new_row)
                  .arg(new_col)
                  .arg(this->plotLayout()->rowCount())
                  .arg(this->plotLayout()->columnCount());
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

std::deque<std::shared_ptr<QCPGraph>> MatrixWidget::AddGraphInRect(int const row, int const col, int count) {
  std::deque<std::shared_ptr<QCPGraph>> result(count);
  QCPAxisRect *current_rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(row, col));

  for (int i = 0; i < count; ++i) {
    std::shared_ptr<QCPGraph> one_graph;
    auto new_graph = CreateDefaultGraph(current_rect);
    one_graph.reset(new_graph);
    result[i] = one_graph;
  }
  return result;
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

    for (QCPAxisRect *rect : axisRects()) {
      auto all_graphs = rect->graphs();
      if (all_graphs.empty()) { continue; } // 尚未添加任何字段
      QCPGraph *single_graph = all_graphs[channel_index];
      double const y = GetValueByHeaderName(message, single_graph->name()).toDouble();
      single_graph->addData(time_index, y);
    }
  }
  channel_msgs_[channel_name].insert(new_data.begin(), new_data.end());
  qDebug() << QString("MatrixWidget::AddNewData added %1 points").arg(new_data.size());
}

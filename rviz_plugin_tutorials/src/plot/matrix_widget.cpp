#include <QWidget>

#include "plot/matrix_widget.h"

MatrixWidget::MatrixWidget(QWidget *parent) : PlotBase(parent) {
  //
  type_ = Type::Matrix;

  setupVector3Demo();
}

DisplaySyncBase *MatrixWidget::getDisplaySync()
{
  return sync_display_;
}

void MatrixWidget::ShowSubplot(int const index) {
  auto rect_show = this->plotLayout()->take(all_rects_[index]);
  // qDebug() <<  "this->plotLayout()->rowCount() = " << this->plotLayout()->rowCount();
  if (rect_show) {
    all_rects_[index]->setVisible(false);
    this->plotLayout()->simplify();
  } else {
    all_rects_[index]->setVisible(true);
    int current_index = -1;  // 因为每次删除元素后，剩余元素会重新编号，所以需要重新计算当前rect的实际index
    for (int i = 0; i <= index; ++i) {
      if (all_rects_[i]->visible()) {
        current_index++;
      }
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
  for (auto [key, rect] : all_rects_) {
    if (rect->axis(QCPAxis::atBottom)->selectedParts().testFlag(QCPAxis::spAxis)) {
      x_selected = true;
      break;
    }
  }

  for (auto [key, rect] : all_rects_) {
    if (x_selected) {
      rect->setRangeZoom(Qt::Horizontal);
    } else {
      rect->setRangeZoom(Qt::Vertical);
    }
  }
  // qDebug() <<" x_selected = " << x_selected;
}

void MatrixWidget::setupVector3Demo() {
  // demoName = "Vector3 Demo";
  this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes);

  this->plotLayout()->clear();  // let's start from scratch and remove the default axis rect
  // add the first axis rect in second row (row index 1):
  QCPAxisRect *topAxisRect = new QCPAxisRect(this);
  QCPAxisRect *middleAxisRect = new QCPAxisRect(this);
  QCPAxisRect *bottomAxisRect = new QCPAxisRect(this);
  all_rects_[0] = topAxisRect;
  all_rects_[1] = middleAxisRect;
  all_rects_[2] = bottomAxisRect;
  this->plotLayout()->addElement(0, 0, topAxisRect);
  this->plotLayout()->addElement(1, 0, middleAxisRect);
  this->plotLayout()->addElement(2, 0, bottomAxisRect);
  // 默认缩放y轴
  topAxisRect->setRangeZoom(Qt::Vertical);
  middleAxisRect->setRangeZoom(Qt::Vertical);
  bottomAxisRect->setRangeZoom(Qt::Vertical);
  topAxisRect->axis(QCPAxis::atLeft)->setLabel("y0 Axis");
  middleAxisRect->axis(QCPAxis::atLeft)->setLabel("y1 Axis");
  bottomAxisRect->axis(QCPAxis::atLeft)->setLabel("y2 Axis");

  // x轴的padding为0
  // topAxisRect->axis(QCPAxis::atBottom)->setPadding(0);
  // topAxisRect->setMargins(QMargins{0,0,0,0});
  // middleAxisRect->setMargins(QMargins{0,0,0,0});
  topAxisRect->setAutoMargins(QCP::msLeft | QCP::msRight);
  topAxisRect->setMargins(QMargins(0, 0, 0, 0));
  middleAxisRect->setAutoMargins(QCP::msLeft | QCP::msRight);
  middleAxisRect->setMargins(QMargins(0, 0, 0, 0));
  bottomAxisRect->setAutoMargins(QCP::msLeft | QCP::msRight | QCP::msBottom);
  bottomAxisRect->setMargins(QMargins(0, 0, 0, 0));
  // this->plotLayout()->setRowSpacing(0);
  // this->plotLayout()->setRowStretchFactor(0, 1);
  // 共x轴
  connect(bottomAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), middleAxisRect->axis(QCPAxis::atBottom),
          SLOT(setRange(QCPRange)));
  connect(middleAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), topAxisRect->axis(QCPAxis::atBottom),
          SLOT(setRange(QCPRange)));
  connect(topAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), bottomAxisRect->axis(QCPAxis::atBottom),
          SLOT(setRange(QCPRange)));
  // y轴label在axis内侧
  topAxisRect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  middleAxisRect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  bottomAxisRect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  topAxisRect->axis(QCPAxis::atLeft)->setSubTicks(false);
  middleAxisRect->axis(QCPAxis::atLeft)->setSubTicks(false);
  bottomAxisRect->axis(QCPAxis::atLeft)->setSubTicks(false);
  // top和middle的x轴不显示
  topAxisRect->axis(QCPAxis::atBottom)->setTicks(false);
  topAxisRect->axis(QCPAxis::atBottom)->setSubTicks(false);
  topAxisRect->axis(QCPAxis::atBottom)->setTickLabels(false);
  middleAxisRect->axis(QCPAxis::atBottom)->setTicks(false);
  middleAxisRect->axis(QCPAxis::atBottom)->setSubTicks(false);
  middleAxisRect->axis(QCPAxis::atBottom)->setTickLabels(false);

  QList<QCPAxis *> allAxes;
  allAxes << topAxisRect->axes() << middleAxisRect->axes() << bottomAxisRect->axes();
  foreach (QCPAxis *axis, allAxes) {
    axis->setLayer("axes");
    axis->grid()->setLayer("grid");
  }

  addRandomGraph();
  this->rescaleAxes();

  this->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel()));
}

void MatrixWidget::addRandomGraph() {
  int n = 500000;  // number of points in graph
  double xScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double yScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double xOffset = (std::rand() / (double)RAND_MAX - 0.5) * 4;
  double yOffset = (std::rand() / (double)RAND_MAX - 0.5) * 10;
  double r1 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r2 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r3 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r4 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  QVector<double> x(n), y(n);
  for (int i = 0; i < n; i++) {
    x[i] = (i / (double)n - 0.5) * 10.0 * xScale + xOffset;
    y[i] = (qSin(x[i] * r1 * 5) * qSin(qCos(x[i] * r2) * r4 * 3) + r3 * qCos(qSin(x[i]) * r4 * 2)) * yScale + yOffset;
  }

  for (auto [key, rect] : all_rects_) {
    QCPAxis *left = rect->axis(QCPAxis::atLeft);
    QCPAxis *bottom = rect->axis(QCPAxis::atBottom);

    this->addGraph(bottom, left);
    this->graph()->setName(QString("New graph %1").arg(this->graphCount() - 1));
    this->graph()->setData(x, y);
    this->graph()->setLineStyle((QCPGraph::LineStyle)(std::rand() % 5 + 1));
    if (std::rand() % 100 > 50) this->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(std::rand() % 14 + 1)));
    QPen graphPen;
    graphPen.setColor(QColor(std::rand() % 245 + 10, std::rand() % 245 + 10, std::rand() % 245 + 10));
    graphPen.setWidthF(std::rand() / (double)RAND_MAX * 2 + 1);
    this->graph()->setPen(graphPen);
  }

  this->replot();
}
#include "data_table/data_table_widget.h"
#include "data_table_display.h"
#include <rviz/visualization_manager.h>
#include <rviz/display_group.h>

#include "display_sync_base.h"
#include "plot/matrix_display.h"

DisplaySyncBase *DataTableWidget::getDisplaySync()
{
  return sync_display_;
}

DataTableWidget::DataTableWidget(QWidget *parent) : QWidget(parent)
{
  // qRegisterMetaType<Eigen::Vector3d>("Vector3d");

  mainTableView_ = new QTableView(this);
  subTableView_ = new QTableView(this);

  mainTableView_->horizontalHeader()->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(mainTableView_->horizontalHeader(), &QHeaderView::customContextMenuRequested,
          this, &DataTableWidget::showHeaderMenu);

  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(mainTableView_);
  layout->addWidget(subTableView_);

  model_ = new MyTableModel(this);

  main_proxy_ = new MainProxyModel(this);
  main_proxy_->setSourceModel(model_);
  mainTableView_->setModel(main_proxy_);

  sub_proxy_ = new SubProxyModel(this);
  sub_proxy_->setSourceModel(model_);
  subTableView_->setModel(sub_proxy_);

  // 连接信号和槽以同步主副表格
  connect(mainTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
          { this->OnMainSelectionChanged(selected, deselected); });
  connect(subTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
          { this->OnSubSelectionChanged(selected, deselected); });
}

void DataTableWidget::setMainInterval(int interval)
{
  main_proxy_->setInterval(interval);
}

void DataTableWidget::setSubRange(int range)
{
  sub_proxy_->setRange(range);

  Scrol2SubMiddle();
}

void DataTableWidget::OnMainSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
  if (selected.indexes().isEmpty())
    return;

  int currentRow = selected.indexes().last().row();

  sub_proxy_->setMiddleRow(main_proxy_->mapToSource(selected.indexes().last()).row());
  Scrol2SubMiddle();

  // 获得时间索引
  // double t0_s = model_->data(main_proxy_->mapToSource(mainTableView_->currentIndex()), 0), Qt::DisplayRole).toDouble();
  // this->setPoint(t0_s);
  QCPRange time_range;
  time_range.lower = model_->data(main_proxy_->mapToSource(selected.indexes().front()), Qt::DisplayRole).toDouble();
  QModelIndex next_select_index = main_proxy_->index(selected.indexes().back().row() + 1, 0);
  auto const end_variant = model_->data(main_proxy_->mapToSource(next_select_index), Qt::DisplayRole);
  auto const last_variant = model_->LastData(0);
  time_range.upper = end_variant.isNull() ? last_variant.toDouble() : end_variant.toDouble();

  // qDebug() << QString(" time_range = [%1,%2]").arg(time_range.lower, 0, 'f', 3).arg(time_range.upper, 0, 'f', 3);
  // qDebug() << end_variant << QString(" last = %1").arg(last_variant.toDouble(), 0, 'f', 3);
  this->onFouseRange(time_range, false, true);
}

void DataTableWidget::OnSubSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
  if (selected.indexes().isEmpty())
    return;

  // 获得时间索引
  double t0_s = model_->data(sub_proxy_->mapToSource(subTableView_->currentIndex()), Qt::DisplayRole).toDouble();
  this->onFocusPoint(t0_s, false, true);
}

void DataTableWidget::Scrol2SubMiddle()
{
  // 选中中间行
  auto *selectionModel = subTableView_->selectionModel();
  QModelIndex indexToSelect = sub_proxy_->mapFromSource(main_proxy_->mapToSource(mainTableView_->currentIndex()));
  // selectionModel->select(indexToSelect, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
  // 将选中的行滚动到视图的中间
  subTableView_->scrollTo(indexToSelect, QAbstractItemView::PositionAtTop);
}

void DataTableWidget::FocusPoint(double const t0)
{
  // qDebug() << QString(" DataTableWidget::FocusPoint called ,t0 = %1").arg(t0, 0, 'f', 3);

  QModelIndex const raw_index = model_->getIndexByt0(t0);
  int const main_index_row = raw_index.row() / main_proxy_->getInterval();
  auto const main_proxy_index = main_proxy_->index(main_index_row, 0);
  // auto const main_index = main_proxy_->mapFromSource(main_proxy_index);
  mainTableView_->scrollTo(main_proxy_index, QAbstractItemView::PositionAtTop);
  // 选中
  auto *selectionModel = mainTableView_->selectionModel();
  disconnect(mainTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, 0, 0);

  selectionModel->select(main_proxy_index, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

  connect(mainTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
          { this->OnMainSelectionChanged(selected, deselected); });
  // 附表区域更新
  sub_proxy_->setMiddleRow(main_index_row * main_proxy_->getInterval());

  // 副表focuss
  auto const sub_index = sub_proxy_->mapFromSource(raw_index);
  subTableView_->scrollTo(sub_index, QAbstractItemView::PositionAtTop);
  // 选中
  selectionModel = subTableView_->selectionModel();

  disconnect(subTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, 0, 0);

  selectionModel->select(sub_index, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

  connect(subTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
          { this->OnSubSelectionChanged(selected, deselected); });
  // qDebug() << QString(" main to %1, sub to %2 ").arg(main_index.row()).arg(sub_index.row());
  // this->onFocusPoint(t0, true, false);
}

void DataTableWidget::FouseRange(QCPRange const &time_range)
{
  qDebug() << QString(" DataTableWidget::FouseRange called ,time_range = [%1,%2]").arg(time_range.lower, 0, 'f', 3).arg(time_range.upper, 0, 'f', 3);
  return;
  // // 1. 根据t0计算index
  // double const t0 = time_range.lower;
  // auto const main_index = model_->getIndexByt0(t0);
  // mainTableView_->scrollTo(main_index, QAbstractItemView::PositionAtTop);
  // // 选中
  // auto *selectionModel = mainTableView_->selectionModel();
  // disconnect(mainTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, 0, 0);

  // selectionModel->select(main_index, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

  // connect(mainTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
  //         { this->OnMainSelectionChanged(selected, deselected); });
  // // 附表区域更新
  // subModel_->UpdateStart(main_index.row() * model_->gettDisplayInterval());

  // // 副表focuss
  // auto const sub_index = subModel_->getIndexByt0(t0);
  // subTableView_->scrollTo(sub_index, QAbstractItemView::PositionAtTop);

  // qDebug() << QString(" main to %1, sub to %2 ").arg(main_focus_index).arg(sub_focus_index);
  // this->onFocusPoint(t0, true, false);

  return;
}

void DataTableWidget::showHeaderMenu(const QPoint &pos)
{
  // auto const old = mainTableView_->selectionBehavior();
  // mainTableView_->setSelectionBehavior(QAbstractItemView::SelectColumns);
  QStringList filed_names;
  // 获取每个选中列的表头字符串
  for (auto const &col : mainTableView_->selectionModel()->selectedColumns())
  {
    QString header = main_proxy_->headerData(col.column(), Qt::Horizontal).toString();
    filed_names.append(header);
  }
  qDebug() << " showHeaderMenu " << filed_names.join('.');

  QMenu *menu = new QMenu(this);
  menu->addAction("create row plot", this, [this, filed_names]()
                  { this->CreateRowVectorPlot("", filed_names); });
  menu->addAction("create col plot", this, [this, filed_names]()
                  { this->CreateVectorPlot("", filed_names); });
  // 添加更多操作...
  QMenu *sub_menu = new QMenu("create matrix plot",menu);
  menu->addMenu(sub_menu);
  sub_menu->addAction("row = 2");
  sub_menu->addAction("row = 3");
  sub_menu->addAction("row = 4");
  sub_menu->addAction("col = 2");
  sub_menu->addAction("col = 3");
  sub_menu->addAction("col = 4");

  menu->popup(mainTableView_->horizontalHeader()->mapToGlobal(pos));

  // mainTableView_->setSelectionBehavior(old);
}

MatrixDisplay *DataTableWidget::CreateMatrixDisplay()
{
  qDebug() << " CreateMatrixPlot ";
  if (!sync_display_)
  {
    return nullptr;
  }
  // qobject_cast<rviz::VisualizationManager*>(sync_display_->context_)->createDisplay()
  auto *matrix_display = new MatrixDisplay();
  sync_display_->getContext()->getRootDisplayGroup()->addDisplay(matrix_display);
  matrix_display->initialize(sync_display_->getContext());
  matrix_display->setName(MatrixDisplay::generateName());

  return matrix_display;
}

void DataTableWidget::CreateMatrixPlot(QString const &name, QStringList const &field_names)
{
  MatrixDisplay *matrix_display = CreateMatrixDisplay();
}

void DataTableWidget::CreateRowVectorPlot(QString const &name, QStringList const &field_names)
{
  MatrixDisplay *matrix_display = CreateMatrixDisplay();
  MatrixXQString fields = MatrixXQString(1, field_names.size());
  for (int i = 0; i < field_names.size(); ++i)
  {
    fields(0, i) = field_names[i];
  }
  matrix_display->CreateMatrixPlot(name, fields);
}
void DataTableWidget::CreateVectorPlot(QString const &name, QStringList const &field_names)
{
  MatrixDisplay *matrix_display = CreateMatrixDisplay();
  MatrixXQString fields = MatrixXQString(field_names.size(), 1);
  for (int i = 0; i < field_names.size(); ++i)
  {
    fields(i, 0) = field_names[i];
  }
  matrix_display->CreateMatrixPlot(name, fields);
}
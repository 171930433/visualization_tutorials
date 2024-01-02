#include "data_table/data_table_widget.h"

DisplaySyncBase *DataTableWidget::getDisplaySync()
{
  return sync_display_;
}

DataTableWidget::DataTableWidget(QWidget *parent) : QWidget(parent)
{
  // qRegisterMetaType<Eigen::Vector3d>("Vector3d");

  mainTableView_ = new QTableView(this);
  subTableView_ = new QTableView(this);

  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(mainTableView_);
  layout->addWidget(subTableView_);

  // 存在两份表格数据,需要优化
  mainModel_ = new MyTableModel(this);
  subModel_ = new MyTableModel(this);

  mainTableView_->setModel(mainModel_);
  subTableView_->setModel(subModel_);

  // 连接信号和槽以同步主副表格
  connect(mainTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
          { this->OnMainSelectionChanged(selected, deselected); });
  connect(subTableView_->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
          { this->OnSubSelectionChanged(selected, deselected); });
}

void DataTableWidget::setMainInterval(int interval)
{
  mainModel_->setDisplayInterval(interval);
  // mainTableView_->resizeRowsToContents();  // 自动行高
}

void DataTableWidget::setSubRange(int range)
{
  subModel_->setSubTableRange(range);
  // subTableView_->resizeRowsToContents();
  Scrol2SubMiddle();
}

void DataTableWidget::OnMainSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
  if (selected.indexes().isEmpty())
    return;

  int currentRow = selected.indexes().last().row();
  subModel_->UpdateStart(currentRow * mainModel_->gettDisplayInterval());
  Scrol2SubMiddle();
  // 获得时间索引
  double t0_s = view_data_[currentRow * mainModel_->gettDisplayInterval()][0].toDouble();
  // this->setPoint(t0_s);
  QCPRange time_range;
  time_range.lower = mainModel_->data(selected.indexes().first(), Qt::DisplayRole).toDouble();
  auto const end_variant = mainModel_->data(mainModel_->index(selected.indexes().last().row() + 1, 0), Qt::DisplayRole);
  time_range.upper = end_variant.isNull() ? view_data_.last()[0].toDouble() : end_variant.toDouble();

  this->onFouseRange(time_range, false, true);
}

void DataTableWidget::OnSubSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
  if (selected.indexes().isEmpty())
    return;

  // int currentRow = selected.indexes().first().row();
  // auto indexToSelect = subModel_->data(currentRow, Qt::DisplayRole);

  // 获得时间索引
  double t0_s = subModel_->data(selected.indexes().first(), Qt::DisplayRole).toDouble();
  // this->setPoint(t0_s);
  this->onFocusPoint(t0_s, false, true);
}

void DataTableWidget::Scrol2SubMiddle()
{
  // 选中中间行
  auto *selectionModel = subTableView_->selectionModel();
  auto indexToSelect = subModel_->index(subModel_->getSubTableRange() / 2, 0);
  // selectionModel->select(indexToSelect, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
  // 将选中的行滚动到视图的中间
  subTableView_->scrollTo(indexToSelect, QAbstractItemView::PositionAtTop);
}

void DataTableWidget::FocusPoint(double const t0)
{
  qDebug() << QString(" DataTableWidget::FocusPoint called ,t0 = %1").arg(t0, 0, 'f', 3);
  // 1. 根据t0计算index
  auto it = std::lower_bound(view_data_.constBegin(), view_data_.constEnd(), QVariant(t0), [](QVector<QVariant> const &v1, QVariant const &v2)
                             { return v1[0] < v2; });
  // 主表focus
  double const index = std::distance(view_data_.constBegin(), it) - 1;
  int const main_focus_index = std::round(index / mainModel_->gettDisplayInterval());
  auto const main_index = mainModel_->index(main_focus_index, 0);
  mainTableView_->scrollTo(main_index, QAbstractItemView::PositionAtTop);
  // 选中
  auto *selectionModel = mainTableView_->selectionModel();
  selectionModel->select(main_index, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

  // 附表区域更新
  subModel_->UpdateStart(main_focus_index * mainModel_->gettDisplayInterval());

  // 副表focuss
  int const sub_focus_index = index - (main_focus_index * mainModel_->gettDisplayInterval() - subModel_->getSubTableRange() / 2);
  auto const sub_index = subModel_->index(sub_focus_index, 0);
  subTableView_->scrollTo(sub_index, QAbstractItemView::PositionAtTop);
  // 选中
  selectionModel = subTableView_->selectionModel();
  selectionModel->select(sub_index, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

  qDebug() << QString(" main to %1, sub to %2 ").arg(main_focus_index).arg(sub_focus_index);
  // this->onFocusPoint(t0, true, false);
}

void DataTableWidget::FouseRange(QCPRange const &time_range)
{
  return;
}
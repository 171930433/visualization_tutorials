#include "data_table/data_table_widget.h"
#include "data_table/filterwidget.h"
#include "data_table_display.h"
#include <rviz/visualization_manager.h>
#include <rviz/display_group.h>

#include "display_sync_base.h"
#include "plot/matrix_display.h"

#include <QStyledItemDelegate>
#include <QString>

class DoublePrecisionDelegate : public QStyledItemDelegate
{
public:
  DoublePrecisionDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}

  QString displayText(const QVariant &value, const QLocale &locale) const override
  {
    if (value.type() == QVariant::Double)
    {
      double const number = value.toDouble();
      return locale.toString(number, 'f', 6); // 小数点后保留3位
    }
    return QStyledItemDelegate::displayText(value, locale);
  }
};

double DataTableWidget::getLastDataTime() const
{
  return model_->LastData(0).toDouble();
}

void DataTableWidget::setDataTypeName(std::string const &type_name)
{
  qDebug() << " headers = " << QString::fromStdString(type_name);

  auto msg = CreateMessageByName(type_name);
  qDebug() << " msg = " << (msg == nullptr);
  qDebug() << " msg = " << QString::fromStdString(msg->DebugString());

  QStringList headers = GetFildNames(*msg);
  qDebug() << " headers = " << headers;

  // column_->addItems(headers);
  QStringList options = headers;
  options.prepend("all columns");
  column_model_->setStringList(options);

  // 设置表头
  model_->setHeaders(headers);
}

void DataTableWidget::UpdateData(const std::map<size_t, sp_cPbMsg> &newData)
{
  // 转换数据到适合模型的格式
  model_->AddData(newData);
}

DataTableWidget::DataTableWidget(QWidget *parent) : QWidget(parent)
{
  // qRegisterMetaType<Eigen::Vector3d>("Vector3d");
  DoublePrecisionDelegate *delegate = new DoublePrecisionDelegate();

  mainTableView_ = new QTableView(this);
  subTableView_ = new QTableView(this);

  mainTableView_->setItemDelegate(delegate); // 应用于第二列
  subTableView_->setItemDelegate(delegate);  // 应用于第二列

  mainTableView_->horizontalHeader()->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(mainTableView_->horizontalHeader(), &QHeaderView::customContextMenuRequested,
          this, &DataTableWidget::showHeaderMenu);
  // combobox
  column_ = new QComboBox();
  column_->setEditable(true);
  column_model_ = new QStringListModel();
  QCompleter *completer = new QCompleter(column_model_, column_);
  completer->setCaseSensitivity(Qt::CaseInsensitive);
  column_->setCompleter(completer);
  column_->setModel(column_model_);
  // filter
  filterWidget_ = new FilterWidget;
  // column_->addItem("all columns");
  // filterWidget_->setText("Grace|Sports");
  connect(filterWidget_, &FilterWidget::filterChanged, this, &DataTableWidget::textFilterChanged);
  connect(column_, &QComboBox::currentTextChanged, this, &DataTableWidget::textFilterChanged);

  filterPatternLabel_ = new QLabel(tr("Filter &columns:"));
  filterPatternLabel_->setBuddy(filterWidget_);
  QHBoxLayout *h_layout = new QHBoxLayout();
  h_layout->addWidget(filterPatternLabel_);
  h_layout->addWidget(column_);
  h_layout->addWidget(filterWidget_, 1);

  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(mainTableView_, 1);
  layout->addLayout(h_layout);
  layout->addWidget(subTableView_, 1);

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
  QMenu *sub_menu = new QMenu("create matrix plot", menu);
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

void DataTableWidget::textFilterChanged()
{
  QRegExp regExp(filterWidget_->text(), filterWidget_->caseSensitivity(), filterWidget_->patternSyntax());
  int index = column_->currentIndex() - 1;
  sub_proxy_->setColumnIndex(index);
  qDebug() << QString("index=%1").arg(index);
  sub_proxy_->setFilterRegExp(regExp);
}
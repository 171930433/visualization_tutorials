#include "data_table/data_table_widget.h"

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

    int currentRow = selected.indexes().first().row();
    subModel_->UpdateStart(currentRow * mainModel_->gettDisplayInterval());
    Scrol2SubMiddle();
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

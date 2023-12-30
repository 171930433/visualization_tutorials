#ifndef DATATABLEWIDGET_H
#define DATATABLEWIDGET_H

#include <QWidget>
#include <QTableView>
#include <QHBoxLayout>
#include <vector>
#include <boost/hana.hpp>
#include <qdebug.h>
#include "data_table_model.h"

#include <string>

// 示例结构体
struct MyStruct
{
    int a;
    double b;
    std::string c;
};

BOOST_HANA_ADAPT_STRUCT(MyStruct, a, b, c);

class DataTableWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DataTableWidget(QWidget *parent = nullptr) : QWidget(parent)
    {
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

    template <typename T>
    void setData(const std::vector<T> &newData)
    {

        QStringList headers;

        // 使用 Boost.Hana 获取字段名
        boost::hana::for_each(boost::hana::keys(T{}), [&](auto key)
                              { headers << QString::fromStdString(boost::hana::to<char const *>(key)); });

        // 设置表头
        mainModel_->setHeaders(headers);
        subModel_->setHeaders(headers);

        // 数据转换
        view_data_.clear();
        view_data_.reserve(newData.size());

        for (const auto &item : newData)
        {
            QVector<QVariant> rowData;
            hana::for_each(hana::members(item), [&](const auto &field)
                           { rowData.push_back(convertToVariant(field)); });

            view_data_.push_back(rowData);
        }
        // 转换数据到适合模型的格式
        mainModel_->setData(view_data_);
        subModel_->setData(view_data_);
    }

    void OnMainSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
    {
        if (selected.indexes().isEmpty())
            return;

        int currentRow = selected.indexes().first().row();
        subModel_->UpdateStart(currentRow * mainModel_->gettDisplayInterval());
        // 选中中间行
        auto *selectionModel = subTableView_->selectionModel();
        auto indexToSelect = subModel_->index(subModel_->getSubTableRange() / 2, 0);
        selectionModel->select(indexToSelect, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
        // 将选中的行滚动到视图的中间
        subTableView_->scrollTo(indexToSelect, QAbstractItemView::PositionAtCenter);
    }
    void setMainInterval(int interval)
    {
        mainModel_->setDisplayInterval(interval);
    }
    void setSubRange(int range) { subModel_->setSubTableRange(range); }

private:
    QTableView *mainTableView_;
    QTableView *subTableView_;
    MyTableModel *mainModel_;
    MyTableModel *subModel_;
    QVector<QVector<QVariant>> view_data_; // 存储数据的二维数组,持有显示数据的资源
};

#endif // DATATABLEWIDGET_H

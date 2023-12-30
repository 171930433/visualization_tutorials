#ifndef DATATABLEWIDGET_H
#define DATATABLEWIDGET_H

#include <QWidget>
#include <QTableView>
#include <QHBoxLayout>
#include <vector>
#include <boost/hana.hpp>
#include <qdebug.h>
#include "my_table_model.h"

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
        mainTableView = new QTableView(this);
        subTableView = new QTableView(this);

        QVBoxLayout *layout = new QVBoxLayout(this);
        layout->addWidget(mainTableView);
        layout->addWidget(subTableView);

        // 存在两份表格数据,需要优化
        mainModel = new MyTableModel(this);
        subModel = new MyTableModel(this, true);

        mainTableView->setModel(mainModel);
        subTableView->setModel(subModel);

        // 连接信号和槽以同步主副表格
        connect(mainTableView->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection &selected, const QItemSelection &deselected)
                { this->OnMainSelectionChanged(selected, deselected); });
    }

    template <typename T>
    void setData(const std::vector<T> &data)
    {

        QStringList headers;

        // 使用 Boost.Hana 获取字段名
        boost::hana::for_each(boost::hana::keys(T{}), [&](auto key)
                              { headers << QString::fromStdString(boost::hana::to<char const *>(key)); });

        // 设置表头
        mainModel->setHeaders(headers);
        subModel->setHeaders(headers);

        // 转换数据到适合模型的格式
        mainModel->setData(data);
        subModel->setData(data);
    }

    void OnMainSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
    {
        if (selected.indexes().isEmpty())
            return;

        int currentRow = selected.indexes().first().row();
        subModel->UpdateStart(currentRow);
        qDebug() << QString("start = %1").arg(currentRow);
    }
    void setMainInterval(int interval)
    {
        mainModel->setDisplayInterval(interval);
        subModel->setDisplayInterval(interval);
    }
    void setSubRange(int range) { subModel->setSubTableRange(range); }

private:
    QTableView *mainTableView;
    QTableView *subTableView;
    MyTableModel *mainModel;
    MyTableModel *subModel;
};

#endif // DATATABLEWIDGET_H

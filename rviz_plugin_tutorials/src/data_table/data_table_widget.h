#pragma once

#include <QWidget>
#include <QTableView>
#include <QHBoxLayout>
#include <vector>
#include <qdebug.h>
#include <string>
#include <eigen3/Eigen/Dense>

#include "data_table/data_table_model.h"
#include "time_sync.h"

class DataTableDisplay;
class DisplaySyncBase;
class DataTableWidget : public QWidget, public ITimeSync
{
    Q_OBJECT

public:
    explicit DataTableWidget(QWidget *parent = nullptr);

    void setDisplaySync(DisplaySyncBase *sync_display) { sync_display_ = sync_display; }
    DisplaySyncBase *getDisplaySync() override;

    void setData(const std::map<size_t, spMessage> &newData)
    {

        QStringList headers = GetFildNames(*newData.begin()->second);
        qDebug() << " headers = " << headers;
        // 设置表头
        model_->setHeaders(headers);
        // mainModel_->setHeaders(headers);
        // subModel_->setHeaders(headers);

        // 转换数据到适合模型的格式
        model_->setData(newData);
        // mainModel_->setData(newData);
        // subModel_->setData(newData);
    }

public slots:
    void setMainInterval(int interval);
    void setSubRange(int range);
    void OnMainSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);
    void OnSubSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);

protected:
    void FocusPoint(double const t0) override;
    void FouseRange(QCPRange const &time_range) override;

private:
    void Scrol2SubMiddle();

private:
    QTableView *mainTableView_;
    QTableView *subTableView_;
    // MyTableModel *mainModel_;
    // MyTableModel *subModel_;
    MyTableModel *model_;
    MainProxyModel *main_proxy_;
    SubProxyModel *sub_proxy_;

    friend DataTableDisplay;
    DisplaySyncBase *sync_display_ = nullptr;
};

#ifndef MYTABLEMODEL_H
#define MYTABLEMODEL_H

#include <QAbstractTableModel>
#include <boost/hana.hpp>
#include <vector>
#include <QStringList>
#include <QVariant>
#include <qdebug.h>

namespace hana = boost::hana;

class MyTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    MyTableModel(QObject *parent = nullptr)
        : QAbstractTableModel(parent)
    {
    }

    // 设置数据
    void setData(QVector<QVector<QVariant>> &newData)
    {
        beginResetModel();
        data_ = &newData;
        // 主表的范围在设定数据时才可确定
        if (!end_)
        {
            end_ = data_->size() - 1;
        }
        // 副表在设置range时即可确定
        endResetModel();
    }

    // 设置表头
    void setHeaders(const QStringList &headers)
    {
        headers_ = headers;
    }

    // 设置主表显示间隔
    void setDisplayInterval(int interval)
    {
        interval_ = interval;
        emit dataChanged(index(0, 0), index(rowCount(), columnCount()));
        emit layoutChanged();
    }
    int gettDisplayInterval() const { return interval_; }

    // 设置子表显示范围
    void setSubTableRange(int range)
    {
        range_ = range;
        end_ = start_ + range_; // 子表设置完range后,显示行数即确定
        emit dataChanged(index(0, 0), index(rowCount(), columnCount()));
        emit layoutChanged();
    }
    int getSubTableRange() const { return (end_ - start_); }
    int rowCount(const QModelIndex &parent = QModelIndex()) const override
    {
        if (!data_)
        {
            return 0;
        }
        return (end_ - start_) / interval_;
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override
    {
        if (!data_)
        {
            return 0;
        }
        return headers_.size();
    }

    void UpdateStart(int start)
    {
        int s0 = start * interval_ - range_ / 2;
        start_ = (s0 < 0 ? 0 : s0);
        int end = start * interval_ + range_ / 2;
        end_ = (end > data_->size() - 1 ? data_->size() - 1 : end);
        int dynamic_range = end_ - start_;
        emit dataChanged(index(0, 0), index(start_ - end_, columnCount()));
        qDebug() << QString("start = %1 end = %2").arg(start_).arg(end_);
    }

    QVariant data(const QModelIndex &index, int role) const override
    {
        if (!data_)
        {
            return QVariant();
        }
        if (!index.isValid() || role != Qt::DisplayRole)
        {
            return QVariant();
        }

        int rowIndex = index.row() * interval_ + start_;
        // qDebug() << QString("rowIndex = %1").arg(rowIndex);
        if (rowIndex >= data_->size())
        {
            return QVariant();
        }

        return data_->operator[](rowIndex)[index.column()];
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role) const override
    {
        if (!data_)
        {
            return QVariant();
        }
        if (role != Qt::DisplayRole || orientation != Qt::Horizontal)
        {
            return QVariant();
        }

        if (section >= headers_.size())
        {
            return QVariant();
        }

        return headers_[section];
    }

private:
    QVector<QVector<QVariant>> *data_; // 存储数据的二维数组
    QStringList headers_;              // 存储表头
    int interval_ = 1;                 // 主表显示间隔
    int range_ = 1;                    // 子表显示范围
    int start_ = 0;                    // 开始行数
    int end_ = 0;                      // 结束行数
};

#endif // MYTABLEMODEL_H

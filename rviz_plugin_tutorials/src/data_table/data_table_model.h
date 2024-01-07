#ifndef MYTABLEMODEL_H
#define MYTABLEMODEL_H

#include <QAbstractTableModel>
#include <boost/hana.hpp>
#include <vector>
#include <QStringList>
#include <QVariant>
#include <qdebug.h>
#include "protobuf_helper.h"
#include <map>
#include <utility>

namespace hana = boost::hana;

#include <vector>
#include <map>
#include <iostream>

template <typename K, typename V>
class IndexedMap
{
public:
    std::map<K, size_t> key_to_index_;
    std::vector<V> values_;

public:
    void setMap(std::map<K, V> const &raw_map)
    {
        for (auto kv : raw_map)
        {
            insert(kv.first, kv.second);
        }
    }
    void insert(const K &key, const V &value)
    {
        if (key_to_index_.find(key) == key_to_index_.end())
        {
            key_to_index_[key] = values_.size();
            values_.push_back(value);
        }
        else
        {
            // Handle key already exists scenario
        }
    }
    size_t size() const { return values_.size(); }
    bool empty() const { return values_.empty(); }
    V const &back() const { return values_.front(); }
    V const &front() const { return values_.back(); }

    const V &getByKey(const K &key) const
    {
        return values_[key_to_index_.at(key)];
    }

    const V &getByIndex(size_t index) const
    {
        return values_.at(index);
    }

    size_t getIndexByKey(const K &key) const
    {
        return key_to_index_.at(key);
    }
};

class MyTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    MyTableModel(QObject *parent = nullptr)
        : QAbstractTableModel(parent)
    {
    }

    // 设置数据
    void setData(const std::map<size_t, spMessage> &newData)
    {
        beginResetModel();
        index_map_.setMap(newData);
        // map_datas_ = &newData;
        // datas_.reserve(newData.size());
        // for (auto kv : newData)
        // {
        //     datas_.push_back(kv.second);
        // }
        // 主表的范围在设定数据时才可确定
        if (!end_)
        {
            end_ = index_map_.size() - 1;
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
        if (index_map_.empty())
        {
            return 0;
        }
        return (end_ - start_) / interval_;
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override
    {
        if (index_map_.empty())
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
        end_ = (end > index_map_.size() - 1 ? index_map_.size() - 1 : end);
        int dynamic_range = end_ - start_;
        emit dataChanged(index(0, 0), index(start_ - end_, columnCount()));
        qDebug() << QString("start = %1 end = %2").arg(start_).arg(end_);
    }

    QVariant data(const QModelIndex &index, int role) const override
    {
        if (index_map_.empty())
        {
            return QVariant();
        }
        if (!index.isValid() || role != Qt::DisplayRole)
        {
            return QVariant();
        }

        int rowIndex = index.row() * interval_ + start_;
        // qDebug() << QString("rowIndex = %1").arg(rowIndex);
        if (rowIndex >= index_map_.size())
        {
            return QVariant();
        }
        // return QVariant();
        return GetValueByHeaderName(*index_map_.getByIndex(rowIndex), headers_[index.column()]);
        // return data_->operator[](rowIndex)[index.column()];
    }
    QVariant LastData(int col) const
    {
        if (index_map_.empty())
        {
            return QVariant();
        }
        return GetValueByHeaderName(*index_map_.back(), headers_[col]);
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role) const override
    {
        if (index_map_.empty())
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

    // std::vector<spMessage> const &AllDatas() const { return datas_; }
    QModelIndex getIndexByt0(double const t0) const
    {
        auto index = index_map_.getIndexByKey(t0 * 1e3);
        int index2 = (index - start_) / interval_;
        return this->index(index2, 0);
    }

private:
    // std::vector<spMessage> datas_;
    // std::map<size_t, spMessage> *map_datas_ = nullptr;
    IndexedMap<size_t, spMessage> index_map_;
    QStringList headers_; // 存储表头
    int interval_ = 1;    // 主表显示间隔
    int range_ = 1;       // 子表显示范围
    int start_ = 0;       // 开始行数
    int end_ = 0;         // 结束行数
};

#endif // MYTABLEMODEL_H

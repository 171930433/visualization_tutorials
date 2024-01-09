#pragma once

#include <QAbstractTableModel>
#include <QSortFilterProxyModel>
#include <vector>
#include <QStringList>
#include <QVariant>
#include <qdebug.h>
#include <map>
#include <utility>
#include <vector>
#include <iostream>

#include "protobuf_helper.h"

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
    V const &back() const { return values_.back(); }
    V const &front() const { return values_.front(); }

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
        endResetModel();
    }

    // 设置表头
    void setHeaders(const QStringList &headers)
    {
        headers_ = headers;
    }

    int rowCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return index_map_.size();
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return headers_.size();
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

        if (index.row() >= index_map_.size())
        {
            return QVariant();
        }
        return GetValueByHeaderName(*index_map_.getByIndex(index.row()), headers_[index.column()]);
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
        auto index2 = index_map_.getIndexByKey(t0 * 1e3);
        return this->index(index2, 0);
    }

private:
    IndexedMap<size_t, spMessage> index_map_;
    QStringList headers_; // 存储表头
};

class MainProxyModel : public QSortFilterProxyModel
{
public:
    MainProxyModel(QObject *parent = nullptr) : QSortFilterProxyModel(parent) {}

    void setInterval(int interval)
    {
        if (interval != interval_)
        {
            interval_ = interval;
            invalidateFilter(); // 重新应用过滤器
        }
    }
    int getInterval() const { return interval_; }


protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override
    {
        return sourceRow % interval_ == 0;
    }

private:
    int interval_ = 100;
};

class SubProxyModel : public QSortFilterProxyModel
{
public:
    SubProxyModel(QObject *parent = nullptr) : QSortFilterProxyModel(parent) {}

    void setMiddleRow(int row)
    {
        if (row != middle_row_)
        {
            middle_row_ = row;
            invalidateFilter(); // 重新应用过滤器
        }
    }
    int getStartRow() const { return middle_row_; }

    void setRange(int range)
    {
        if (range != range_)
        {
            range_ = range;
            invalidateFilter(); // 重新应用过滤器
        }
    }
    int getRange() const { return range_; }

protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override
    {
        // 示例过滤逻辑: 只显示与选中行前后10行的数据
        int lowerBound = std::max(0, middle_row_ - range_);
        int upperBound = middle_row_ + range_;
        return sourceRow >= lowerBound && sourceRow <= upperBound;
    }

private:
    int middle_row_ = 0;
    int range_ = 100;
};

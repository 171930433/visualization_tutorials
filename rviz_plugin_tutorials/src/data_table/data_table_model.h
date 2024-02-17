#pragma once

#include "protobuf_helper.h"
#include <QAbstractTableModel>
#include <QSortFilterProxyModel>
#include <QStringList>
#include <QVariant>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index_container.hpp>
#include <iostream>
#include <map>
#include <qdebug.h>
#include <utility>
#include <vector>

using namespace boost::multi_index;

inline size_t ExactT0_ms(sp_cPbMsg msg) { return GetHeaderT0(*msg, "header") * 1e3; }

namespace inner {
struct t0 {};
} // namespace inner

using DataContainer = boost::multi_index_container<
    sp_cPbMsg,
    indexed_by<random_access<>, ordered_unique<tag<inner::t0>, global_fun<sp_cPbMsg, size_t, ExactT0_ms>>>>;

class MyTableModel : public QAbstractTableModel {
  Q_OBJECT
public:
  MyTableModel(QObject *parent = nullptr) : QAbstractTableModel(parent) {}

  void Reset() {
    beginResetModel();
    headers_.clear();
    datas_.clear();
    endResetModel();
  }

  // 设置数据
  void setData(const std::map<size_t, sp_cPbMsg> &newData) {
    beginResetModel();
    for (auto const &elem : newData) {
      datas_.push_back(elem.second);
    }
    endResetModel();
  }

  void AddData(const std::map<size_t, sp_cPbMsg> &newData) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount() + newData.size() - 1);
    for (auto const &elem : newData) {
      datas_.push_back(elem.second);
    }
    endInsertRows();
    qDebug() << "add data called " << datas_.size();
  }

  // 设置表头
  void setHeaders(const QStringList &headers) {
    beginInsertColumns(QModelIndex(), columnCount(), columnCount() + headers.size() - 1);
    headers_ = headers;
    endInsertColumns();
  }

  int rowCount(const QModelIndex &parent = QModelIndex()) const override {
    return datas_.size();
  }

  int columnCount(const QModelIndex &parent = QModelIndex()) const override { return headers_.size(); }

  QVariant data(const QModelIndex &index, int role) const override {
    if (datas_.empty()) { return QVariant(); }
    if (!index.isValid() || role != Qt::DisplayRole) { return QVariant(); }

    if (index.row() >= datas_.size()) { return QVariant(); }

    return GetValueByHeaderName(*datas_[index.row()], headers_[index.column()]);
  }
  QVariant LastData(int col) const {
    if (datas_.empty()) { return QVariant(); }
    return GetValueByHeaderName(*datas_.back(), headers_[col]);
  }

  QVariant headerData(int section, Qt::Orientation orientation, int role) const override {
    if (headers_.empty()) { return QVariant(); }
    if (role != Qt::DisplayRole || orientation != Qt::Horizontal) { return QVariant(); }

    if (section >= headers_.size()) { return QVariant(); }

    return headers_[section];
  }

  // std::vector<spMessage> const &AllDatas() const { return datas_; }
  QModelIndex getIndexByt0(double const t0) const {
    auto it2 = datas_.project<0>(datas_.get<inner::t0>().lower_bound(t0 * 1e3));
    
    return this->index(std::distance(datas_.begin(), it2), 0);
  }

  QStringList const &headers() const { return headers_; }

private:
  QStringList headers_; // 存储表头
  DataContainer datas_;
};

class MainProxyModel : public QSortFilterProxyModel {
public:
  MainProxyModel(QObject *parent = nullptr) : QSortFilterProxyModel(parent) {}

  void setInterval(int interval) {
    if (interval != interval_) {
      interval_ = interval;
      invalidateFilter(); // 重新应用过滤器
    }
  }
  int getInterval() const { return interval_; }

protected:
  bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override {
    return sourceRow % interval_ == 0;
  }

private:
  int interval_ = 100;
};

class SubProxyModel : public QSortFilterProxyModel {
public:
  SubProxyModel(QObject *parent = nullptr) : QSortFilterProxyModel(parent) {}

  void setMiddleRow(int row) {
    if (row != middle_row_) {
      middle_row_ = row;
      invalidateFilter(); // 重新应用过滤器
    }
  }
  int getStartRow() const { return middle_row_; }

  void setRange(int range) {
    if (range != range_) {
      range_ = range;
      invalidateFilter(); // 重新应用过滤器
    }
  }
  int getRange() const { return range_; }

  void setColumnIndex(int const &col) {
    if (col != column_index_) { column_index_ = col; }
  }

protected:
  bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override {
    // 示例过滤逻辑: 只显示与选中行前后10行的数据
    int lowerBound = std::max(0, middle_row_ - range_);
    int upperBound = middle_row_ + range_;
    bool b1 = (sourceRow >= lowerBound && sourceRow <= upperBound);
    if (!b1) { return b1; }
    // 开始查询条件
    bool b2 = false;
    int start_col = (column_index_ == -1 ? 0 : column_index_);
    int end_col = (column_index_ == -1 ? sourceModel()->columnCount() : start_col + 1);
    for (int i = start_col; i < end_col; ++i) {
      QModelIndex const index = sourceModel()->index(sourceRow, i, sourceParent);
      QVariant const &value = sourceModel()->data(index);
      QString const &str =
          (value.type() == QVariant::Double ? QString::number(value.toDouble(), 'f', 6) : value.toString());
      b2 |= str.contains(filterRegExp());
      if (b2) {
        // qDebug() << QString("row=%1,col=%2, content=%4, contains %3").arg(sourceRow).arg(i).arg(filterRegExp().pattern()).arg(str);
        break;
      }
    }

    return b2;
  }

private:
  int middle_row_ = 0;
  int range_ = 100;
  int column_index_ = -1; // -1 表示所有, >=0 表示指定列
};

#ifndef MYTABLEMODEL_H
#define MYTABLEMODEL_H

#include <QAbstractTableModel>
#include <boost/hana.hpp>
#include <vector>
#include <QStringList>
#include <QVariant>

namespace hana = boost::hana;

// 转换函数
template <typename T>
inline QVariant convertToVariant(const T &value)
{
    return QVariant::fromValue(value);
}

// 特化对于 std::string 的转换
template <>
inline QVariant convertToVariant<std::string>(const std::string &value)
{
    return QVariant(QString::fromStdString(value));
}

class MyTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    MyTableModel(QObject *parent = nullptr, bool isSubTable = false)
        : QAbstractTableModel(parent), m_isSubTable(isSubTable) {}

    // 设置数据
    template <typename T>
    void setData(const std::vector<T> &newData)
    {
        beginResetModel();
        m_data.clear();
        m_data.reserve(newData.size());

        for (const auto &item : newData)
        {
            QVector<QVariant> rowData;
            hana::for_each(hana::members(item), [&](const auto &field)
                           { using FieldType = typename std::remove_reference<decltype(field)>::type; 
                                rowData.push_back(convertToVariant(field)); });

            m_data.push_back(rowData);
        }

        endResetModel();
    }

    // 设置表头
    void setHeaders(const QStringList &headers)
    {
        m_headers = headers;
    }

    // 设置主表显示间隔
    void setDisplayInterval(int interval)
    {
        if (!m_isSubTable)
        {
        }
        m_interval = interval;
    }

    // 设置子表显示范围
    void setSubTableRange(int range)
    {
        if (m_isSubTable)
        {
        }
        dynamic_range = m_range = range;
        qDebug() <<"--------- m_range = " << m_range;
    }

    int rowCount(const QModelIndex &parent = QModelIndex()) const override
    {
        if (m_isSubTable)
        {
            return dynamic_range;
            // return std::min(dynamic_range, m_data.size() - 1);
        }
        else
        {
            // 主表的行数根据间隔计算
            return m_interval > 0 ? (m_data.size() + m_interval - 1) / m_interval : 0;
        }
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return m_headers.size();
    }

    void UpdateStart(int start)
    {
        int s0 = start * m_interval - m_range / 2;
        start_ = (s0 < 0 ? 0 : s0);
        int end = start * m_interval + m_range / 2;
        end = (end > m_data.size() - 1 ? m_data.size() - 1 : end);
        dynamic_range = end - start_;
        qDebug() << QString("start = %1 range = %2 raw = %3  m_interval = %4 dy_range= %5 end = %6").arg(start_).arg(m_range).arg(start).arg(m_interval).arg(dynamic_range).arg(end);
    }

    QVariant data(const QModelIndex &index, int role) const override
    {
        if (!index.isValid() || role != Qt::DisplayRole)
        {
            return QVariant();
        }

        int rowIndex = index.row();
        if (m_isSubTable)
        {
            // 子表数据处理
            rowIndex += start_;
            if (rowIndex >= m_data.size())
            {
                return QVariant();
            }
        }
        else
        {
            // 主表数据处理
            rowIndex *= m_interval;
            if (rowIndex >= m_data.size())
            {
                return QVariant();
            }
        }

        return m_data[rowIndex][index.column()];
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role) const override
    {
        if (role != Qt::DisplayRole || orientation != Qt::Horizontal)
        {
            return QVariant();
        }

        if (section >= m_headers.size())
        {
            return QVariant();
        }

        return m_headers[section];
    }

private:
    QVector<QVector<QVariant>> m_data; // 存储数据的二维数组
    QStringList m_headers;             // 存储表头
    bool m_isSubTable;                 // 标识是否为子表
    int m_interval = 1;                // 主表显示间隔
    int m_range = 10;                  // 子表显示范围
    int dynamic_range = 10;                  // 子表显示范围
    int start_ = 0;                    // 主表的开始行数
};

#endif // MYTABLEMODEL_H

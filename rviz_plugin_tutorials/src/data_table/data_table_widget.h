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
#include <eigen3/Eigen/Dense>

// 示例结构体

enum class Color
{
    Red,
    Green
};

struct MyStruct
{
    int a;
    double b;
    std::string c;
    Eigen::Vector3d acc_ = Eigen::Vector3d::Identity();
    Color color_ = Color::Red;
};
Q_DECLARE_METATYPE(Eigen::Vector3d);
Q_DECLARE_METATYPE(Color);
BOOST_HANA_ADAPT_STRUCT(MyStruct, a, b, c, acc_, color_);

// 转换函数
template <typename T, typename std::enable_if<!std::is_enum<T>::value, T>::type * = nullptr>
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

// 特化对于 Eigen::Vector3d 的转换
template <>
inline QVariant convertToVariant<Eigen::Vector3d>(const Eigen::Vector3d &value)
{
    return QVariant(QString("%1 %2 %3").arg(value.x()).arg(value.y()).arg(value.z()));
}

// 通用的枚举到 QVariant 的转换函数
template <typename T, typename = std::enable_if_t<std::is_enum<T>::value>>
QVariant convertToEnumVariant(const T &value)
{
    return QVariant(static_cast<int>(value));
}

// 特化 convertToVariant 函数，用于所有枚举类型
template <typename T>
inline typename std::enable_if_t<std::is_enum<T>::value, QVariant>
convertToVariant(const T &value)
{
    return convertToEnumVariant(value);
}

class DataTableDisplay;

class DataTableWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DataTableWidget(QWidget *parent = nullptr);

    template <typename T>
    void setData(const std::vector<T> &newData)
    {

        QStringList headers;

        // 使用 Boost.Hana 获取字段名
        boost::hana::for_each(boost::hana::keys(T{}), [&](auto key)
                              { headers << QString::fromStdString(boost::hana::to<char const *>(key)); });

        qDebug() << " headers = " << headers;
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

public slots:
    void setMainInterval(int interval);
    void setSubRange(int range);
    void OnMainSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);
private:
    void Scrol2SubMiddle();
private:
    QTableView *mainTableView_;
    QTableView *subTableView_;
    MyTableModel *mainModel_;
    MyTableModel *subModel_;
    QVector<QVector<QVariant>> view_data_; // 存储数据的二维数组,持有显示数据的资源

    friend DataTableDisplay;
};

#endif // DATATABLEWIDGET_H

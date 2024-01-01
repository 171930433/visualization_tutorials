#pragma once
#include <map>
#include "plot/qcustomplot.h"
#include "time_sync.h"
// template<typename EnumT>
// inline QStringList EnumToStringList(EnumT const val)
// {
//     QStringList list;
//     const QMetaObject &mo = MyClass::staticMetaObject;
//     int enumIndex = mo.indexOfEnumerator("MyEnum");
//     QMetaEnum metaEnum = mo.enumerator(enumIndex);

//     for (int i = 0; i < metaEnum.keyCount(); ++i)
//     {
//         list << QString(metaEnum.key(i));
//     }

//     return list;
// }
class PlotBase : public QCustomPlot, public ITimeSync
{
  Q_OBJECT
public:
  enum Type
  {
    None = 0,
    Matrix = 1,
    Trajectory = 2
  };
  Q_ENUM(Type);

protected:
  PlotBase(QWidget *parent = nullptr);

public:
  virtual void SyncDataAndView() = 0;

protected:
  Type type_ = Type::None;
  QCPRange time_range_;     // 当前所有数据的范围
};
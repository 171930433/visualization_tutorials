#pragma once

#include <QHBoxLayout>
#include <QTableView>
#include <QWidget>
#include <eigen3/Eigen/Dense>
#include <qdebug.h>
#include <string>
#include <vector>

#include "data_table/data_table_model.h"
#include "time_sync.h"

class DataTableDisplay;
class DisplaySyncBase;
class MatrixDisplay;
class FilterWidget;
class DataTableWidget : public QWidget, public ITimeSync {
  Q_OBJECT

public:
  explicit DataTableWidget(QWidget *parent = nullptr);

  void CreateMatrixPlot(QString const &name, QStringList const &field_names);
  void CreateRowVectorPlot(QString const &name, QStringList const &field_names);
  void CreateVectorPlot(QString const &name, QStringList const &field_names);

  void setChannelName(QString const &name) { channel_name_ = name; };
  QString getChannelName() { return channel_name_; };
  void setDataTypeName(std::string const &type_name);
  void UpdateData(const std::map<size_t, sp_cPbMsg> &newData);
  double getLastDataTime() const;

public slots:
  void setMainInterval(int interval);
  void setSubRange(int range);
  void OnMainSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);
  void OnSubSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);

private slots:
  void textFilterChanged();

protected:
  void FocusPoint(double const t0) override;
  void FouseRange(QCPRange const &time_range) override;

private:
  MatrixDisplay *CreateMatrixDisplay();
  void Scrol2SubMiddle();
  void showHeaderMenu(const QPoint &pos);

private:
  QString channel_name_;
  QTableView *mainTableView_;
  QTableView *subTableView_;
  //
  FilterWidget *filterWidget_;
  QLabel *filterPatternLabel_;
  QComboBox *column_;
  QStringListModel *column_model_;
  //
  MyTableModel *model_;
  MainProxyModel *main_proxy_;
  SubProxyModel *sub_proxy_;
};

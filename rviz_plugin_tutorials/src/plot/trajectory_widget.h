#pragma once
#include "plot/plot_base.h"
#include "protobuf_helper.h"
#include <ros/time.h>
#include <deque>
class QWidget;

class QCP_LIB_DECL QCPMapAxisTickerFixed : public QCPAxisTickerFixed
{
  Q_GADGET
public:
  QCPMapAxisTickerFixed(QCPAxis *x_axis, QCPAxis *y_axis) : QCPAxisTickerFixed()
  {
    x_axis_ = x_axis;
    y_axis_ = y_axis;
    //    this->setTickLength(0, 0);
  }

  double Step() const { return tick_step_; }
  double pixel_per_meter() const { return pixel_per_meter_; }

protected:
  // reimplemented virtual methods: range in meter
  virtual double getTickStep(const QCPRange &range) Q_DECL_OVERRIDE
  {
    y_axis_->setScaleRatio(x_axis_);

    double re = 0;

    if (x_axis_->range().size() >= y_axis_->range().size())
    {
      re = CalcStep(y_axis_->range().size(), y_axis_->axisRect()->height());
    }
    else
    {
      re = CalcStep(x_axis_->range().size(), x_axis_->axisRect()->width());
    }

    // qDebug() << "synced_ = " << synced_ << "getTickStep range = " << range << " step = " << re;
    tick_step_ = re;
    return re;
  }

  virtual int getSubTickCount(double tickStep) Q_DECL_OVERRIDE { return 0; }

  double CalcStep(double const rangle_meter, int const range_pixel)
  {
    double step = 10;
    pixel_per_meter_ = rangle_meter / range_pixel;
    double t[] = {1.0, 2.0, 5.0, 10.0}, tick = 30.0 * pixel_per_meter_;
    double order = pow(10.0, floor(log10(tick)));
    for (int i = 0; i < 4; i++)
    {
      if (tick <= t[i] * order)
      {
        step = t[i] * order;
        break;
      }
    }
    return step;
  }

private:
  QCPAxis *x_axis_;
  QCPAxis *y_axis_;
  double tick_step_ = 0;
  double pixel_per_meter_ = 0;
};

class TrajectoryWidget : public PlotBase
{
  Q_OBJECT

public:
  TrajectoryWidget(QWidget *parent = nullptr);

public:
  void setFocusWhenSelect(bool const flag) { focus_when_select_ = flag; }
  QCPCurve *addTrajectory(QString const &name, QCPScatterStyle const &ss, QPen const &lp);
  void UpdateTrajectory(QCPCurve *curve, std::map<size_t, sp_cPbMsg> const &new_data);


protected:

protected:
  void setupTrajectoryDemo();

private:
  QSharedPointer<QCPMapAxisTickerFixed> map_ticker_;
  QString StepString(double const step); // 分辨率文字
  QCPItemText *step_text_ = nullptr;

private:
  // std::map<std::string, QCPCurve *> all_curve_;
  double current_t0_s_ = 0;
  bool focus_when_select_ = true;
  int plot_type_ = 0;
  QCPAxisRect *new_rect_;
private slots:
  void mouseWheel(QWheelEvent *);

public slots:
  void ChangeScatterShape(QCPScatterStyle::ScatterShape const type);
  void ChangeLineStyle(QCPGraph::LineStyle const type);
};
#include "plot/trajectory_display.h"
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

#include "plot/trajectory_widget.h"
#include "protobuf_helper.h"

TrajectoryDisplay::TrajectoryDisplay() {
  InitPersons();

  view_ = new TrajectoryWidget();
  view_->setDisplaySync(this);

  focus_when_select_ = new rviz::BoolProperty(
      "foucs when select", true, "focus the selected points", this, SLOT(UpdateFocusWhenSelect()));
  counts_prop_ =
      new rviz::IntProperty("trajectory counts", 0, "the number of trajectory counts", this, SLOT(UpdateGraphCount()));
  counts_prop_->setMin(1); // 会触发UpdateGraphCount
  counts_prop_->setMax(10);

  connect(&dataTimer_, SIGNAL(timeout()), this, SLOT(SyncInfo()));
}

TrajectoryDisplay::~TrajectoryDisplay() {
  if (initialized()) { delete view_; }
}

// Overrides from Display
void TrajectoryDisplay::onInitialize() { setAssociatedWidget(view_); }

void TrajectoryDisplay::update(float dt, float ros_dt) {}

void TrajectoryDisplay::UpdateFocusWhenSelect() { view_->setFocusWhenSelect(focus_when_select_->getBool()); }

void TrajectoryDisplay::UpdateGraphCount() {
  int const new_count = counts_prop_->getInt();
  int const old_count = graphs_.size();

  auto when_deleted = [this](rviz::SubCurveProperty *graph) {
    this->takeChild(graph);
    delete graph;
  };

  graphs_.resize(new_count);
  for (int i = 0; i < new_count; ++i) {
    if (graphs_[i]) { continue; }
    QString header = (i == 0 ? QString("main_trj") : QString("trj-%1").arg(i));
    auto *curve = new rviz::SubCurveProperty(header, "", "", this);
    connect(curve, &rviz::Property::changed, this, &TrajectoryDisplay::UpdateTopic);
    graphs_[i] = std::shared_ptr<rviz::SubCurveProperty>(curve, when_deleted);
  }
  // qDebug() <<" UpdateGraphCount called";
}

void TrajectoryDisplay::UpdateTopic() {
  dataTimer_.stop();

  auto *curve_prop = dynamic_cast<rviz::SubCurveProperty *>(sender());

  QString name = curve_prop->getString();
  // 删除当前轨迹
  if (name.isEmpty()) {
    curve_prop->graph().reset();
  } else {
    curve_prop->graph() = view_->addTrajectory(name, curve_prop->getScatterStyle(), curve_prop->getLinePen());
  }

  dataTimer_.start(500); // Interval 0 means to refresh as fast as possible
  view_->replot();
}

void TrajectoryDisplay::SyncInfo() {
  // 去buffer里面查询通道更新

  for (auto curve_prop : graphs_) {
    auto curve = curve_prop->graph();
    if (!curve) { continue; }
    // 当前时间
    double t0 = (curve->interface1D()->dataCount() == 0
                     ? 0
                     : curve->interface1D()->dataSortKey(curve->interface1D()->dataCount() - 1));
    auto msgs = g_cacher_->GetProtoWithChannleName(curve_prop->getStdString(), t0);
    if (!msgs.empty()) {
      qDebug() << QString("new msgs form t0= %1").arg(t0, 0, 'f', 3);
      view_->UpdateTrajectory(curve.get(), msgs);
      view_->replot();
    }
  }

  // 去buffer里面查询数据更新
}

void TrajectoryDisplay::load(const rviz::Config &config) {
  int graph_counts = 0;
  if (config.mapGetInt("trajectory counts", &graph_counts)) { counts_prop_->setInt(graph_counts); }

  rviz::Display::load(config);
  // qDebug() << "end graph size = " << graphs_.size();
}
void TrajectoryDisplay::save(rviz::Config config) const { rviz::Display::save(config); }

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrajectoryDisplay, rviz::Display)
#include "time_sync.h"
#include "display_sync_base.h"

void ITimeSync::onFocusPoint(double const t0, bool update_view, bool emit_signal)
{
  // qDebug() << getDisplaySync()->getName() << " ITimeSync::onFocusPoint " << QString("%1").arg(t0, 0, 'f', 3);
  getDisplaySync()->onFocusPoint(t0, update_view, emit_signal);
}
void ITimeSync::onFouseRange(QCPRange const &time_range, bool update_view, bool emit_signal)
{
  getDisplaySync()->onFouseRange(time_range, update_view, emit_signal);
}

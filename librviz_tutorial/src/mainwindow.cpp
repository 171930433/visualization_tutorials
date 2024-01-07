#include "mainwindow.h"
#include "myviz.h"
#include <QLabel>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // 创建 Dock Manager
    dockManager = new ads::CDockManager(this);

    // 创建一些停靠小部件
    auto *dockWidget1 = new ads::CDockWidget("DockWidget 1");
    dockWidget1->setWidget(new QLabel("Content of DockWidget 1"));
    dockManager->addDockWidget(ads::LeftDockWidgetArea, dockWidget1);

    auto *dockWidget2 = new ads::CDockWidget("DockWidget 2");
    dockWidget2->setWidget(new QLabel("Content of DockWidget 2"));
    dockManager->addDockWidget(ads::RightDockWidgetArea, dockWidget2);

    auto *dockWidget3 = new ads::CDockWidget("myviz");
    dockWidget3->setWidget(new MyViz());
    dockManager->addDockWidget(ads::BottomDockWidgetArea, dockWidget3);
}

MainWindow::~MainWindow()
{
    delete dockManager;
}

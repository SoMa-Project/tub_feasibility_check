#include <Inventor/Qt/SoQt.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  SoQt::init(this);
  SoDB::init();
  ui->setupUi(this);
}

MainWindow::~MainWindow()
{
  delete ui;
}

Viewer *MainWindow::ifcoSceneViewer()
{
  return ui->ifco_scene_viewer;
}

Viewer *MainWindow::tabletopSceneViewer()
{
  return ui->tabletop_scene_viewer;
}

void MainWindow::selectViewer(MainWindow::ViewerType type)
{
  switch (type)
  {
  case ViewerType::IfcoScene:
    ui->stackedWidget->setCurrentWidget(ui->ifco_scene_viewer);
    break;
  case ViewerType::TabletopScene:
    ui->stackedWidget->setCurrentWidget(ui->tabletop_scene_viewer);
    break;
  }
}

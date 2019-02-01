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

Viewer *MainWindow::viewer()
{
  return ui->viewer;
}

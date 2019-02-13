#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Viewer.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  enum class ViewerType
  {
    IfcoScene,
    TabletopScene
  };

  explicit MainWindow(QWidget* parent = 0);
  ~MainWindow();

  Viewer* ifcoSceneViewer();
  Viewer* tabletopSceneViewer();

public slots:
  void selectViewer(MainWindow::ViewerType type);

private:
  Ui::MainWindow* ui;
};

#endif  // MAINWINDOW_H

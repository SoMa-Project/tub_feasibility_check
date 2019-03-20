#include "service_worker.h"
#include "mainwindow.h"
#include <ros/package.h>

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "check_kinematics_server", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    std::string ifco_scene_graph_file;
    std::string tabletop_scene_graph_file;
    std::string kinematics_file;
    double delta;
    bool mdl;

    n.param("/feasibility_check/ifco_scene_graph_file", ifco_scene_graph_file, std::string());
    n.param("/feasibility_check/tabletop_scene_graph_file", tabletop_scene_graph_file, std::string());
    n.param("/feasibility_check/kinematics_file", kinematics_file, std::string());
    n.param("/feasibility_check/mdl", mdl, false);
    n.param("/feasibility_check/delta", delta, 0.017);

    auto ifco_scene = std::unique_ptr<IfcoScene>(new IfcoScene(ifco_scene_graph_file, kinematics_file, mdl));
    auto tabletop_scene =
        std::unique_ptr<TabletopScene>(new TabletopScene(tabletop_scene_graph_file, kinematics_file, mdl));

    QApplication application(argc, argv);
    MainWindow main_window(new MainWindow);

    bool hide_window;
    n.param("/feasibility_check/hide_window", hide_window, false);
    if (!hide_window)
    {
      ifco_scene->connectToViewer(main_window.ifcoSceneViewer());
      tabletop_scene->connectToViewer(main_window.tabletopSceneViewer());
      main_window.show();
    }
    else
      main_window.hide();

    ServiceWorker service_worker(std::move(ifco_scene), std::move(tabletop_scene), delta);
    QThread worker_thread;
    service_worker.moveToThread(&worker_thread);
    QObject::connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
    QObject::connect(&application, SIGNAL(lastWindowClosed()), &worker_thread, SLOT(quit()));
    qRegisterMetaType<MainWindow::ViewerType>("MainWindow::ViewerType");
    QObject::connect(&service_worker, SIGNAL(selectViewer(MainWindow::ViewerType)), &main_window,
                     SLOT(selectViewer(MainWindow::ViewerType)));

    ros::ServiceServer checkKinematicsIfcoService =
        n.advertiseService("check_kinematics", &ServiceWorker::checkKinematicsIfcoQuery, &service_worker);

    ros::ServiceServer checkKinematicsTabletopService =
        n.advertiseService("check_kinematics_tabletop", &ServiceWorker::checkKinematicsTabletopQuery, &service_worker);

    ros::ServiceServer checkSurfaceGraspService =
        n.advertiseService("check_surface_grasp", &ServiceWorker::checkSurfaceGraspQuery, &service_worker);

    ros::ServiceServer checkWallGraspService =
        n.advertiseService("check_wall_grasp", &ServiceWorker::checkWallGraspQuery, &service_worker);

    ros::ServiceServer visualizeTrajectoryService =
        n.advertiseService("visualize_trajectory", &ServiceWorker::visualizeTrajectoryQuery, &service_worker);

    worker_thread.start();
    service_worker.start(20);

    auto result = application.exec();
    worker_thread.wait();
    return result;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }
}

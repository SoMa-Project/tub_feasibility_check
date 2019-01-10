#include "service_worker.h"
#include "MainWindow.h"
#include <ros/package.h>

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "check_kinematics_server");
    ros::NodeHandle n;

    std::string scene_graph_file;
    std::string kinematics_file;
    // TODO fix to run in QT and access ros pkg path  - if solved add to readme steps to fix it
    std::string default_root_dir = ros::package::getPath("tub_feasibility_check");
    n.param("/feasibility_check/scene_graph_file", scene_graph_file,  default_root_dir + "/model/rlsg/wam-rbohand-ifco.convex.xml");
    n.param("/feasibility_check/kinematics_file", kinematics_file,  default_root_dir + "/model/rlkin/barrett-wam-ocado2.xml");

    auto ifco_scene = IfcoScene::load(scene_graph_file, kinematics_file);

    QApplication application(argc, argv);
    std::unique_ptr<MainWindow> main_window(new MainWindow);

    bool hide_window;
    n.param("/feasibility_check/hide_window", hide_window, false);
    if (!hide_window)
    {
      ifco_scene->connectToViewer(main_window->viewer);
      main_window->show();
    }
    else
      main_window->hide();

    ServiceWorker service_worker(std::move(ifco_scene));
    QThread worker_thread;
    service_worker.moveToThread(&worker_thread);
    QObject::connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
    QObject::connect(&application, SIGNAL(lastWindowClosed()), &worker_thread, SLOT(quit()));

    ros::ServiceServer checkKinematicsService = n.advertiseService("check_kinematics",
                                                    &ServiceWorker::checkKinematicsQuery, &service_worker);
    ros::ServiceServer cerrtExampleService = n.advertiseService("cerrt_example",
        &ServiceWorker::cerrtExampleQuery, &service_worker);

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

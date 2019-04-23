#include "service_worker.h"
#include "mainwindow.h"
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <tub_feasibility_check/KinematicsCheckConfig.h>

std::unique_ptr<ServiceWorker> service_worker;
std::string kinematics_file_fingertips;
std::string kinematics_file_palm;

// TODO: this is only a temporary fix for the disney use case until hand_transform is handled meaningfully
/**
 * allows us to swap the kinematics during runtime
 */
void dynamic_reconfigure_callback(tub_feasibility_check::KinematicsCheckConfig &config, uint32_t level) {

    std::string kinematics_file;
    switch(config.ee_tf) {
        case tub_feasibility_check::KinematicsCheck_Fingertips :
            kinematics_file = kinematics_file_fingertips;
            break;
        case tub_feasibility_check::KinematicsCheck_Palm :
            kinematics_file = kinematics_file_palm;
            break;
    }

    service_worker->reloadKinematics(kinematics_file);
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "check_kinematics_server");
    ros::NodeHandle n;

    std::string ifco_scene_graph_file;
    std::string tabletop_scene_graph_file;

    // TODO fix to run in QT and access ros pkg path  - if solved add to readme steps to fix it
    std::string default_root_dir = ros::package::getPath("tub_feasibility_check");
    n.param("/feasibility_check/ifco_scene_graph_file", ifco_scene_graph_file, default_root_dir + "/model/rlsg/"
                                                                                                  "wam-rbohand2-ifco."
                                                                                                  "convex.xml");
    n.param("/feasibility_check/tabletop_scene_graph_file", tabletop_scene_graph_file,
            default_root_dir + "/model/rlsg/"
                               "wam-rbohand2-with-camera-tabletop.convex.xml");
    n.param("/feasibility_check/kinematics_file_fingertips", kinematics_file_fingertips, default_root_dir + "/model/rlkin/"
                                                                                                          "barrett-wam-rbohand2.xml");

    n.param("/feasibility_check/kinematics_file_palm", kinematics_file_palm, default_root_dir + "/model/rlkin/"
                                                                                                          "barrett-wam-rbohand2.xml");

    std::vector<double> table_dimensions;
    n.param("/feasibility_check/table_dimensions", table_dimensions, { 0.8, 0.6, 0.03 });

    if (table_dimensions.size() != 3)
      throw std::runtime_error("Table dimensions must be 3 numbers");

    if (table_dimensions[0] < 0 || table_dimensions[1] < 0 || table_dimensions[2] < 0)
      throw std::runtime_error("All table dimensions must be positive");

    auto ifco_scene = std::unique_ptr<IfcoScene>(new IfcoScene(ifco_scene_graph_file, kinematics_file_fingertips));
    auto tabletop_scene = std::unique_ptr<TabletopScene>(new TabletopScene(
        tabletop_scene_graph_file, kinematics_file_fingertips, { table_dimensions[0], table_dimensions[1], table_dimensions[2] }));

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

    service_worker.reset(new ServiceWorker(std::move(ifco_scene), std::move(tabletop_scene)));

    QThread worker_thread;
    service_worker->moveToThread(&worker_thread);
    QObject::connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
    QObject::connect(&application, SIGNAL(lastWindowClosed()), &worker_thread, SLOT(quit()));
    qRegisterMetaType<MainWindow::ViewerType>("MainWindow::ViewerType");
    QObject::connect(service_worker.get(), SIGNAL(selectViewer(MainWindow::ViewerType)), &main_window,
                     SLOT(selectViewer(MainWindow::ViewerType)));

    ros::ServiceServer checkKinematicsIfcoService =
        n.advertiseService("check_kinematics", &ServiceWorker::checkKinematicsIfcoQuery, service_worker.get());

    ros::ServiceServer checkKinematicsTabletopService =
        n.advertiseService("check_kinematics_tabletop", &ServiceWorker::checkKinematicsTabletopQuery, service_worker.get());

    ros::ServiceServer visualizeTrajectoryService =
        n.advertiseService("visualize_trajectory", &ServiceWorker::visualizeTrajectoryQuery, service_worker.get());

    worker_thread.start();
    service_worker->start(20);

    // set callback for dynamic reconfigure
    dynamic_reconfigure::Server<tub_feasibility_check::KinematicsCheckConfig> server;
    dynamic_reconfigure::Server<tub_feasibility_check::KinematicsCheckConfig>::CallbackType f;

    f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
    server.setCallback(f);

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

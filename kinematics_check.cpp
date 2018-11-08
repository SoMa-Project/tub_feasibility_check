#include "service_thread.h"

MainWindow* MainWindow::singleton = NULL;

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "check_kinematics_server");
    ros::NodeHandle n;

    std::string scene_graph_file;
    std::string kinematics_file;
    std::string default_root_dir = "/home/ilia/cmp_2/contact-motion-planning";
    n.param("scene_graph_file", scene_graph_file, default_root_dir + "/soma/rlsg/wam-rbohand-ifco.convex.xml");
    n.param("kinematics_file", kinematics_file, default_root_dir + "/soma/rlkin/barrett-wam-ocado2.xml");

    auto ifco_scene = IfcoScene::load(scene_graph_file, kinematics_file);

    QApplication application(argc, argv);
    bool hide_window;
    n.param("hide_window", hide_window, false);
    if (!hide_window)
    {
      ifco_scene->connectToViewer(MainWindow::instance()->viewer);
      MainWindow::instance()->show();
    }
    else
      MainWindow::instance()->hide();

    ServiceThread thread(std::move(ifco_scene), 20);
    ros::ServiceServer service = n.advertiseService("check_kinematics", &ServiceThread::query, &thread);

    thread.start();
    return application.exec();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }
}

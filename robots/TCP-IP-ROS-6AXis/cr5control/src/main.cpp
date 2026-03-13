/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <QApplication>
#include "main_menu.h"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );

    QApplication app(argc, argv);
    MainMenu main_menu;
    main_menu.show();

    return app.exec();
}

/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include "main_menu.h"
#include "ui_main_menu.h"
#include <rviz/display.h>
#include <rviz/visualization_manager.h>

MainMenu::MainMenu(QWidget* parent) : QWidget(parent), ui(new Ui::MainMenu), render_panel_(nullptr)
{
    ui->setupUi(this);

    render_panel_ = new rviz::RenderPanel();
    QGridLayout* layout = new QGridLayout(ui->widget);
    layout->addWidget(render_panel_);
    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(), manager_);
    manager_->initialize();
    manager_->startUpdate();

    grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);

    manager_->createDisplay("rviz/RobotModel", "Robot", true);
    manager_->setFixedFrame("elfin_base");
    // Configure the GridDisplay the way we like it.
    grid_->subProp("Line Style")->setValue("Billboards");
    grid_->subProp("Color")->setValue(QColor(Qt::yellow));
}

MainMenu::~MainMenu()
{
    delete ui;
}

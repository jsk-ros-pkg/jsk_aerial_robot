/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <QWidget>
#include <rviz/panel.h>
#include <rviz/robot/robot.h>
#include <rviz/render_panel.h>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainMenu;
}
QT_END_NAMESPACE

class MainMenu : public QWidget
{
    Q_OBJECT

public:
    explicit MainMenu(QWidget* parent = nullptr);
    ~MainMenu() override;

private:
    Ui::MainMenu* ui;
    rviz::Display* grid_;
    rviz::RenderPanel* render_panel_;
    rviz::VisualizationManager* manager_;
};

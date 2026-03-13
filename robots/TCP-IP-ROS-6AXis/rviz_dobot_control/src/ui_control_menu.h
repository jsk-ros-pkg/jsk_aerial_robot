/********************************************************************************
** Form generated from reading UI file 'control_menu.ui'
**
** Created by: Qt User Interface Compiler version 5.12.11
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROL_MENU_H
#define UI_CONTROL_MENU_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ControlMenu
{
public:
    QGridLayout *gridLayout;
    QPushButton *enable_robot_btn;
    QPushButton *disable_robot_btn;
    QLabel *label;
    QLineEdit *is_robot_connected;
    QLabel *label_2;
    QLineEdit *is_robot_enable;
    QLabel *label_3;
    QLineEdit *enable_robot_topic;
    QLabel *label_4;
    QLineEdit *disable_robot_topic;
    QLabel *label_5;
    QLineEdit *robot_status_topic;

    void setupUi(QWidget *ControlMenu)
    {
        if (ControlMenu->objectName().isEmpty())
            ControlMenu->setObjectName(QString::fromUtf8("ControlMenu"));
        ControlMenu->resize(234, 210);
        gridLayout = new QGridLayout(ControlMenu);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        enable_robot_btn = new QPushButton(ControlMenu);
        enable_robot_btn->setObjectName(QString::fromUtf8("enable_robot_btn"));

        gridLayout->addWidget(enable_robot_btn, 0, 0, 1, 1);

        disable_robot_btn = new QPushButton(ControlMenu);
        disable_robot_btn->setObjectName(QString::fromUtf8("disable_robot_btn"));

        gridLayout->addWidget(disable_robot_btn, 0, 1, 1, 1);

        label = new QLabel(ControlMenu);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        is_robot_connected = new QLineEdit(ControlMenu);
        is_robot_connected->setObjectName(QString::fromUtf8("is_robot_connected"));
        is_robot_connected->setReadOnly(true);

        gridLayout->addWidget(is_robot_connected, 1, 1, 1, 1);

        label_2 = new QLabel(ControlMenu);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        is_robot_enable = new QLineEdit(ControlMenu);
        is_robot_enable->setObjectName(QString::fromUtf8("is_robot_enable"));
        is_robot_enable->setReadOnly(true);

        gridLayout->addWidget(is_robot_enable, 2, 1, 1, 1);

        label_3 = new QLabel(ControlMenu);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 3, 0, 1, 1);

        enable_robot_topic = new QLineEdit(ControlMenu);
        enable_robot_topic->setObjectName(QString::fromUtf8("enable_robot_topic"));

        gridLayout->addWidget(enable_robot_topic, 3, 1, 1, 1);

        label_4 = new QLabel(ControlMenu);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 4, 0, 1, 1);

        disable_robot_topic = new QLineEdit(ControlMenu);
        disable_robot_topic->setObjectName(QString::fromUtf8("disable_robot_topic"));

        gridLayout->addWidget(disable_robot_topic, 4, 1, 1, 1);

        label_5 = new QLabel(ControlMenu);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 5, 0, 1, 1);

        robot_status_topic = new QLineEdit(ControlMenu);
        robot_status_topic->setObjectName(QString::fromUtf8("robot_status_topic"));

        gridLayout->addWidget(robot_status_topic, 5, 1, 1, 1);


        retranslateUi(ControlMenu);

        QMetaObject::connectSlotsByName(ControlMenu);
    } // setupUi

    void retranslateUi(QWidget *ControlMenu)
    {
        ControlMenu->setWindowTitle(QApplication::translate("ControlMenu", "Form", nullptr));
        enable_robot_btn->setText(QApplication::translate("ControlMenu", "EnableRobot", nullptr));
        disable_robot_btn->setText(QApplication::translate("ControlMenu", "DisableRobot", nullptr));
        label->setText(QApplication::translate("ControlMenu", "ConnectSate", nullptr));
        label_2->setText(QApplication::translate("ControlMenu", "IsRobotEnable", nullptr));
        label_3->setText(QApplication::translate("ControlMenu", "EnableRobotTopic", nullptr));
        label_4->setText(QApplication::translate("ControlMenu", "DisableRobotTopic", nullptr));
        label_5->setText(QApplication::translate("ControlMenu", "RobotStatusTopic", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ControlMenu: public Ui_ControlMenu {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROL_MENU_H

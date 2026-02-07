/**
 * @file mission_command_panel.h
 *
 * @brief Simple UI panel plugin for RViz for sending commands to UGVs.
 *
 * @date 11/21/2024
*
 * @author Evan Vandermate (evanderm@mtu.edu)
 *         Keweenaw Research Center (KRC)
 */
#ifndef MISSION_CMD_PANEL_H
#define MISSION_CMD_PANEL_H

#ifndef Q_MOC_RUN
#include <stdio.h>
#include <string>
#include <limits>

#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QDoubleValidator>
#include <QIntValidator>
#include <QRegExp>
#include <QRegExpValidator>

#include <ros/ros.h>
#include "avt_341_msgs/Communication.h"
#include "std_msgs/String.h"

#include <rviz/panel.h>
#endif

class QLineEdit;

namespace avt_341 {
namespace ui {

class MissionCommandPanel: public rviz::Panel
{

Q_OBJECT
public:
    MissionCommandPanel( QWidget* parent = 0 );

protected Q_SLOTS:
    void sendCommand();
    void resetPerception();

protected:
    // QT Entries
    QLineEdit* sender_name_entry_;
    QLineEdit* msg_id_entry_;
    QComboBox* msg_type_combo_;
    QLineEdit* formation_entry_;
    QLineEdit* receiver_entry_;
    QLineEdit* leader_name_entry_;
    QLineEdit* follower1_entry_;
    QLineEdit* follower2_entry_;
    QLineEdit* follower3_entry_;
    QLineEdit* objective_entry_;
    QLineEdit* desired_speed_entry_;
    QComboBox* priority_type_combo_;
    QComboBox* termination_method_combo_;
    QLineEdit* x_scale_entry_;
    QLineEdit* y_scale_entry_;
    QLineEdit* x_offset_entry_;
    QLineEdit* y_offset_entry_;
    QLineEdit* distance_entry_;
    QLineEdit* target_msg_entry_;

    // QT Buttons
    QPushButton* send_msg_button_;
    QPushButton* reset_perception_button_;

    // The ROS node handle.
    ros::NodeHandle nh_;

    // ROS Publishers
    ros::Publisher command_pub_;
    ros::Publisher reset_pub_;

};
} // end namespace ui
} // end namespace avt_341

#endif // MISSION_CMD_PANEL_Hs
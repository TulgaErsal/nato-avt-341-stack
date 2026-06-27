#include <avt_341_rviz_plugins/mission_command_panel.h>

namespace avt_341::rviz_plugins
{

    MissionCommandPanel::MissionCommandPanel( QWidget* parent )
        : rviz_common::Panel( parent )
    {
        // Create entries
        sender_name_entry_ = new QLineEdit("MRZR");
        msg_id_entry_ = new QLineEdit("0");
        msg_type_combo_ = new QComboBox();
        formation_entry_ = new QLineEdit();
        receiver_entry_ = new QLineEdit("MRZR");
        leader_name_entry_ = new QLineEdit("MRZR");
        follower1_entry_ = new QLineEdit();
        follower2_entry_ = new QLineEdit();
        follower3_entry_ = new QLineEdit();
        objective_entry_ = new QLineEdit();
        desired_speed_entry_ = new QLineEdit("5.0");
        priority_type_combo_ = new QComboBox();
        termination_method_combo_ = new QComboBox();
        x_scale_entry_ = new QLineEdit("1.0");
        y_scale_entry_ = new QLineEdit("1.0");
        x_offset_entry_ = new QLineEdit("0.0");
        y_offset_entry_ = new QLineEdit("0.0");
        distance_entry_ = new QLineEdit("0.0");
        target_msg_entry_ = new QLineEdit("0");

        // Setup entries
        msg_id_entry_->setValidator(new QIntValidator(0, 99, this));
        msg_type_combo_->addItems({ "FORM", "ACK", "ARRIVE", "TASK_COMPLETE", "MOVETO", "SHUTDOWN", "SET_SPEED", "CANCEL", "CANCEL_ALL", "OVERWATCH" });
        desired_speed_entry_->setValidator(new QDoubleValidator(0, 100, 2, this));
        priority_type_combo_->addItems({ "QUEUE", "PREEMPT", "CANCEL_ALL" });
        termination_method_combo_->addItems({ "LEADER_ARRIVED", "ALL_ARRIVED" });
        x_scale_entry_->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 4, this));
        y_scale_entry_->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 4, this));
        x_offset_entry_->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 4, this));
        y_offset_entry_->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 4, this));
        distance_entry_->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 4, this));
        target_msg_entry_->setValidator(new QIntValidator(0, 99, this));

        // Create button
        send_msg_button_ = new QPushButton("Send Message");
        reset_perception_button_ = new QPushButton("Reset Perception");

        // Layout widgets
        QGridLayout* layout = new QGridLayout;
        layout->addWidget( new QLabel( "Sender Name:" ),        0,  0,  Qt::AlignRight);
        layout->addWidget( sender_name_entry_,                  0,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Message ID:" ),         1,  0,  Qt::AlignRight);
        layout->addWidget( msg_id_entry_,                       1,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Message Type:" ),       2,  0,  Qt::AlignRight);
        layout->addWidget( msg_type_combo_,                     2,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Formation:" ),          3,  0,  Qt::AlignRight);
        layout->addWidget( formation_entry_,                    3,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Receiver Name:" ),      4,  0,  Qt::AlignRight);
        layout->addWidget( receiver_entry_,                     4,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Leader Name:" ),        5,  0,  Qt::AlignRight);
        layout->addWidget( leader_name_entry_,                  5,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Follower #1 Name:" ),   6,  0,  Qt::AlignRight);
        layout->addWidget( follower1_entry_,                    6,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Follower #2 Name:" ),   7,  0,  Qt::AlignRight);
        layout->addWidget( follower2_entry_,                    7,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Follower #3 Name:" ),   8,  0,  Qt::AlignRight);
        layout->addWidget( follower3_entry_,                    8,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Objective:" ),          9,  0,  Qt::AlignRight);
        layout->addWidget( objective_entry_,                    9,  1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Desired Speed:" ),      10, 0,  Qt::AlignRight);
        layout->addWidget( desired_speed_entry_,                10, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Priority Type:" ),      11, 0,  Qt::AlignRight);
        layout->addWidget( priority_type_combo_,                11, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Termination Method:" ), 12, 0,  Qt::AlignRight);
        layout->addWidget( termination_method_combo_,           12, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "X Scale:" ),            13, 0,  Qt::AlignRight);
        layout->addWidget( x_scale_entry_,                      13, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Y Scale:" ),            14, 0,  Qt::AlignRight);
        layout->addWidget( y_scale_entry_,                      14, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "X Offset:" ),           15, 0,  Qt::AlignRight);
        layout->addWidget( x_offset_entry_,                     15, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Y Offset:" ),           16, 0,  Qt::AlignRight);
        layout->addWidget( y_offset_entry_,                     16, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Distance:" ),           17, 0,  Qt::AlignRight);
        layout->addWidget( distance_entry_,                     17, 1,  Qt::AlignLeft);
        layout->addWidget( new QLabel( "Target Message ID:" ),  18, 0,  Qt::AlignRight);
        layout->addWidget( target_msg_entry_,                   18, 1,  Qt::AlignLeft);
        layout->addWidget( send_msg_button_,                    19, 0,  1,  2,  Qt::AlignHCenter);
        layout->addWidget( reset_perception_button_,            20, 0,  1,  2,  Qt::AlignHCenter);
        setLayout( layout );

        // Create signal/slot connections.
        connect( send_msg_button_, SIGNAL( clicked() ), this, SLOT( sendCommand() ));
        connect( reset_perception_button_, SIGNAL( clicked() ), this, SLOT( resetPerception() ));
    }

    void MissionCommandPanel::onInitialize()
    {
        // Initialize ROS node and publishers
        node_ = std::make_shared<rclcpp::Node>("mission_command_panel_node");
        command_pub_ = node_->create_publisher<avt_341_msgs::msg::Communication>("avt_341/comm_messages", 10);
        reset_pub_ = node_->create_publisher<std_msgs::msg::String>("avt_341/reset", 10);
    }

    void MissionCommandPanel::sendCommand()
    {
        avt_341_msgs::msg::Communication command_msg;
        command_msg.sender_name = sender_name_entry_->text().toUpper().toStdString();
        command_msg.msg_id = msg_id_entry_->text().toInt();
        command_msg.type = msg_type_combo_->currentText().toStdString();
        command_msg.formation = formation_entry_->text().toStdString();
        command_msg.receiver_name = receiver_entry_->text().toUpper().toStdString();
        command_msg.leader_name = leader_name_entry_->text().toUpper().toStdString();
        command_msg.follower1_name = follower1_entry_->text().toUpper().toStdString();
        command_msg.follower2_name = follower2_entry_->text().toUpper().toStdString();
        command_msg.follower3_name = follower3_entry_->text().toUpper().toStdString();
        command_msg.objective_name = objective_entry_->text().toStdString();
        command_msg.desired_speed = desired_speed_entry_->text().toDouble();
        command_msg.priority_type = priority_type_combo_->currentText().toStdString();
        command_msg.termination_method = termination_method_combo_->currentText().toStdString();
        command_msg.x_scale = x_scale_entry_->text().toDouble();
        command_msg.y_scale = y_scale_entry_->text().toDouble();
        command_msg.x_offset = x_offset_entry_->text().toDouble();
        command_msg.y_offset = y_offset_entry_->text().toDouble();
        command_msg.distance = distance_entry_->text().toDouble();
        command_msg.target_msg_id = target_msg_entry_->text().toInt();
        command_pub_->publish(command_msg);
    }

    void MissionCommandPanel::resetPerception()
    {
        std_msgs::msg::String reset_msg;
        reset_msg.data = "perception";
        reset_pub_->publish(reset_msg);
    }

}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(avt_341::rviz_plugins::MissionCommandPanel, rviz_common::Panel )
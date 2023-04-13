#!/usr/bin/env roseus

(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::roseus-add-msgs "std_msgs")


(defclass aerial-robot-interface
  :super robot-interface
  :slots (land-topic-name
          halt-topic-name
          takeoff-topic-name
          force-landing-topic-name
          ctrl-mode-topic-name
  ))


(defmethod aerial-robot-interface
  (:init (&key (namespace) (robot) &rest args)
    (prog1
      (send-super* :init
                   :robot robot 
                   :joint-states-topic (format nil "/~A/joint_states" namespace)
                   args)
      (setq land-topic-name (format nil "/~A/teleop_command/land" namespace))
      (setq halt-topic-name (format nil "/~A/teleop_command/halt" namespace))
      (setq start-topic-name (format nil "/~A/teleop_command/start" namespace))
      (setq takeoff-topic-name (format nil "/~A/teleop_command/takeoff" namespace))
      (setq force-landing-topic-name (format nil "/~A/teleop_command/force_landing" namespace))
      (setq ctrl-mode-topic-name (format nil "/~A/teleop_command/ctrl_mode" namespace))
      (ros::advertise land-topic-name std_msgs::Empty)
      (ros::advertise halt-topic-name std_msgs::Empty)
      (ros::advertise start-topic-name std_msgs::Empty)
      (ros::advertise takeoff-topic-name std_msgs::Empty)
      (ros::advertise force-landing-topic-name std_msgs::Empty)
      (ros::advertise ctrl-mode-topic-name std_msgs::Int8)))
  (:land () (ros::publish land-topic-name (instance std_msgs::Empty :init)))
  (:halt () (ros::publish halt-topic-name (instance std_msgs::Empty :init)))
  (:start () (ros::publish start-topic-name (instance std_msgs::Empty :init)))
  (:takeoff () (ros::publish takeoff-topic-name (instance std_msgs::Empty :init)))
  (:force-landing () (ros::publish force-landing-topic-name (instance std_msgs::Empty :init)))
  (:ctrl-mode (data)
    (ros::publish ctrl-mode-topic-name (instance std_msgs::Int8 :init :data data)))
  )


(provide :aerial-robot-interface "package://aerial_robot_base/euslisp/aerial-robot-interface.l")

#!/usr/bin/env roseus

(require :aerial-robot-interface "package://aerial_robot_base/euslisp/aerial-robot-interface.l")
(require :dragon-utils "package://dragon/euslisp/quad/dragon-utils.l")


(defclass dragon-interface
  :super aerial-robot-interface
  :slots ()
  )


(defmethod dragon-interface
  (:init (&rest args)
    (prog1 (send-super* :init :robot dragon-robot :namespace "dragon" args)
      (send self :add-controller :larm-controller)))
  (:default-controller ()
    (append
      (send self :larm-controller)))
  (:larm-controller ()
    (list
      (list
        (cons :controller-action "/dragon/follow_joint_trajectory")
        (cons :controller-state "/dragon/follow_joint_trajectory/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names
              (list "joint1_pitch" "joint1_yaw"
                    "joint2_pitch" "joint2_yaw"
                    "joint3_pitch" "joint3_yaw"))))))


(defun dragon-init ()
  (setq *dragon* (instance dragon-robot :init))
  (setq *robot* *dragon*)
  (setq *ri* (instance dragon-interface :init))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (objects (list *robot*)))


(provide :dragon-interface "package://dragon/euslisp/quad/dragon-interface.l)

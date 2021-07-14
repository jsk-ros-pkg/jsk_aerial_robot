#!/usr/bin/env roseus

(require :aerial-robot-interface "package://aerial_robot_base/euslisp/aerial-robot-interface.l")
(require :hydrus-utils "package://hydrus/euslisp/quad/default_mode_201907/hydrus-utils.l")


(defclass hydrus-interface
  :super aerial-robot-interface
  :slots ()
  )


(defmethod hydrus-interface
  (:init (&rest args)
    (prog1 (send-super* :init :robot hydrus-robot :namespace "hydrus" args)
      (send self :add-controller :larm-controller)))
  (:default-controller ()
    (append
      (send self :larm-controller)))
  (:larm-controller ()
    (list
      (list
        (cons :controller-action "/hydrus/follow_joint_trajectory")
        (cons :controller-state "/hydrus/follow_joint_trajectory/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names (list "joint1" "joint2" "joint3"))))))


(defun hydrus-init ()
  (setq *hydrus* (instance hydrus-robot :init))
  (setq *robot* *hydrus*)
  (setq *ri* (instance hydrus-interface :init))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (objects (list *robot*)))


(provide :hydrus-interface "package://hydrus/euslisp/quad/default_mode_201907/hydrus-interface.l)

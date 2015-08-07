#!/usr/bin/env roseus

;;;

(load "package://aerial_robot_model/robots/test/test.l")
(load "package://pr2eus/robot-interface.l")

;;; robot simulator to robot interface
(defun robot2ri ()
(send *ri* :angle-vector (send *test* :angle-vector)))

;;; robot interface to robot simulator
(defun ri2robot ()
(send *test* :angle-vector (send *ri* :state :potentio-vector)))


;;create robot-simulator instace
(setq *test* (instance test-robot :init))

;; create robot-interface instance
(setq *ri* (instance robot-interface :init :robot test-robot))


(if (not (boundp '*irtviewer*)) (make-irtviewer))
(objects (list *test*))
(send *irtviewer* :draw-objects)


;(send *test* :torso :angle-vector #f(20.0))



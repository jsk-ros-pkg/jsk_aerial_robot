(require :dragon "package://dragon/robots/quad/dragon.l")


(unless (assoc :init-orig (send dragon-robot :methods))
  (rplaca (assoc :init (send dragon-robot :methods)) :init-orig))


(defmethod dragon-robot
  (:init
   (&rest args)
   ;; red
   (dolist (b (list
                :_make_instance_link1_geom0
                :_make_instance_link2_geom0
                :_make_instance_link3_geom0
                :_make_instance_link4_geom0
                :_make_instance_inter_joint1_geom0
                :_make_instance_inter_joint2_geom0
                :_make_instance_inter_joint3_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(1.0 0.0 0.0 0.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 0.0))
                         (assoc b (send  (class self) :methods))))))
   ;; black
   (dolist (b (list
                :_make_instance_gimbal1_pitch_module_geom0
                :_make_instance_gimbal2_pitch_module_geom0
                :_make_instance_gimbal3_pitch_module_geom0
                :_make_instance_gimbal4_pitch_module_geom0
                :_make_instance_gimbal1_roll_module_geom0
                :_make_instance_gimbal2_roll_module_geom0
                :_make_instance_gimbal3_roll_module_geom0
                :_make_instance_gimbal4_roll_module_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.3 0.3 0.3 0.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 0.0))
                         (assoc b (send  (class self) :methods))))))
   (send* self :init-orig args))


(provide :dragon-utils "package://dragon/euslisp/quad/dragon-utils.l")

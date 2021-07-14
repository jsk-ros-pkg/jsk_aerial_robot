(require :hydrus "package://hydrus/robots/quad/default_mode_201907/hydrus.l")


(unless (assoc :init-orig (send hydrus-robot :methods))
  (rplaca (assoc :init (send hydrus-robot :methods)) :init-orig))


(defmethod hydrus-robot
  (:init
   (&rest args)
   ;; red
   (dolist (b (list
                :_make_instance_link1_geom0
                :_make_instance_link2_geom0
                :_make_instance_link3_geom0
                :_make_instance_link4_geom0
                :_make_instance_bat1_geom0
                :_make_instance_bat2_geom0
                ))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(1.0 0.0 0.0 0.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 0.0))
                         (assoc b (send  (class self) :methods))))))
   ;; black
   (dolist (b (list
                :_make_instance_leg1_geom0
                :_make_instance_leg2_geom0
                :_make_instance_leg3_geom0
                :_make_instance_leg4_geom0
                :_make_instance_leg5_geom0
                :_make_instance_thrust1_geom0
                :_make_instance_thrust2_geom0
                :_make_instance_thrust3_geom0
                :_make_instance_thrust4_geom0
                :_make_instance_fc_geom0
                :_make_instance_gps_geom0
                :_make_instance_leddarone_geom0
                ))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.3 0.3 0.3 0.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 0.0))
                         (assoc b (send  (class self) :methods))))))
   (prog1 (send* self :init-orig args)
     ;; transparent
     (send-all (send (send self :thrust1_lk) :bodies) :set-color nil 0.0)
     (send-all (send (send self :thrust2_lk) :bodies) :set-color nil 0.0)
     (send-all (send (send self :thrust3_lk) :bodies) :set-color nil 0.0)
     (send-all (send (send self :thrust4_lk) :bodies) :set-color nil 0.0))))


(provide :hydrus-utils "package://hydrus/euslisp/quad/default_mode_201907/hydrus-utils.l")

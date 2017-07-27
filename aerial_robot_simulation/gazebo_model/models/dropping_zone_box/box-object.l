;; box.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defclass box-object
    :super cascaded-link)

(defmethod box-object
    (:init
     (&rest args &key (name "dropping_zone_box") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
      (let (c bc
              blink0
              box inner)
        (setq box (make-cube 100000 50000 80000 :pos #f(0 0 40000)))
        (setq inner (make-cube 95000 45000 77500 :pos #f(0 0 42500)))
        (setq box (body- box inner))

        (send-super* :init :name name args)
        (setq bc (list box))
        (dolist (b (cdr bc)) (send (car bc) :assoc b))
        (send (elt bc 0) :set-color :gray)
        (setq blink0 (instance bodyset-link :init (make-cascoords) :bodies bc :name :box-bodyset :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
        ;; definition of assoc
        (send self :assoc blink0)
        (setq links (list blink0))
        (setq joint-list (list))
        (send self :init-ending)
        (send self :move-to (make-coords :pos pos :rot rot))
        (send-all links :worldcoords)
        self))

  (:handle (&rest args) (forward-message-to-all (list ) args))
  (:attention (&rest args) (forward-message-to-all (list ) args))
  (:button (&rest args) (forward-message-to-all (list ) args))
)

(defun box (&rest args) (instance* box-object :init args))


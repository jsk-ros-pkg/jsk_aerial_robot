;; box-to-collada.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(load "irteus/irtcollada.l")

(load "box-object.l")
(collada::eus2collada (box) ".")
(exit)

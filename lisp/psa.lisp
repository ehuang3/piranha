(in-package :piranha)

(defvar *s-e*)
(defvar *s-obj*)

(defparameter *psa-s-pt-hinge-0* (aa::tf-qv2duqu (aa::tf-qmul (aa::tf-yangle2quat (/ pi 2))
                                                              (aa::tf-xangle2quat (/ pi -2)))
                                                 (aa::vec 2.77d-2 0 0)))

;; q-obj * q-pt = q-e
;; q-obj = q-e * conj(q-pt)
(defun psa-s-e->s-obj (s-e s-pt)
  (aa::tf-duqu-mulc s-e s-pt))



(defun psa-grasp (s-obj s-pt r-open r-close x-appr &key (t0 10d0) (t1 15d0))
  (assert (< t0 t1))
  (let* ((s-grasp (aa::tf-duqu-mul s-obj s-pt))
         (s-appr (aa::tf-duqu-mul s-grasp
                                  (aa::tf-qv2duqu aa::+tf-quat-ident+ (aa::vec (- x-appr) 0 0)))))
    (pir-go (list (make-trajx-point :pose s-appr
                                    :time t0)
                  (make-trajx-point :pose s-grasp
                                    :time t1)))
    (sleep 1)
    (pir-pinch .15 r-open)
    (sleep (1- t1))
    (pir-pinch .15 r-close)
    (sleep 1)
    (pir-go (list (make-trajx-point :pose s-appr
                                    :time (- t1 t0))))))

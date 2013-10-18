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



;(psa-grasp *s-obj*  *psa-s-pt-hinge-0* .02 .007 6e-2 :t0 7d0 :t1 10d0)

(psa-grasp *s-obj*  *psa-s-pt-hinge-0* .02 .007 6e-2 :t0 7d0 :t1 10d0)


(psa-grasp (aa::tf-duqu-mul *s-obj*
                            (aa::tf-zxyz2duqu (* -.5 pi) 0d0 0d0 0d0))
           *psa-s-pt-hinge-0* .02 .007 6e-2 :t0 7d0 :t1 10d0)

(defun splend-demo (&optional (s-0 (pir-state-s-f-l (get-state))))
  (let* ((s-1 (aa::tf-duqu-mul s-0 (aa::tf-xxyz2duqu (* .99 pi) 0d0 0d0 0d0)))
         (s-2 (aa::tf-duqu-mul s-1 (aa::tf-yxyz2duqu (* .1d0 pi) 0d0 0d0 0d0)))
         (s-3 (aa::tf-duqu-mul s-0 (aa::tf-yxyz2duqu (* .1d0 pi) 0d0 0d0 0d0)))
         (s-4 s-0)
         (t-1 8d0)
         (t-2 (+ t-1 3d0))
         (t-3 (+ t-2 12d0))
         (t-4 (+ t-3 3d0)))
    (pir-go (list (make-trajx-point :pose s-1 :time t-1)
                  (make-trajx-point :pose s-2 :time t-2)
                  (make-trajx-point :pose s-3 :time t-3)
                  (make-trajx-point :pose s-4 :time t-4)))))

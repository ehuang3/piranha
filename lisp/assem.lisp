(in-package :piranha)

(defparameter +r-screw-l0+ (g* +r-right+ (x-angle (degrees 120))))
(defparameter +w-screw+ 0.0150)
(defparameter +w-driver+ 0.0105)
(defparameter +w-nut+ 0.022)
(defparameter +w-box+ 0.023)

;(defparameter +rad/m+ (/ (* 2 pi 2.5d0) 1d-2))
(defparameter +rad/m+ (/ (* 2 pi 2.5d0) 2.0d-2))
;(defparameter +rad/m+ (/ (* 2 pi 2.5d0) 1.0d-2))

(defparameter +dtheta+ (* .75 pi))
(defparameter +dx+ (/ +dtheta+ +rad/m+))

(defparameter +e-screw-left+
  (quaternion-translation-2 +r-screw-l0+ (aa::vec3 .403 .065 0.001)))

(defparameter +e-screw-approach-left+
  (g* +e-screw-left+
      (quaternion-translation-2 (x-angle 0)
                                (aa::vec3 -10e-2 0 0))))


(defparameter +e-screw-rel+
  (quaternion-translation-2 (quaternion (x-angle +dtheta+))
                            (aa::vec3 +dx+ 0 0)))

(defparameter +e-unscrew-rel+
  (quaternion-translation-2 (quaternion (x-angle (- +dtheta+)))
                            (aa::vec3 (- +dx+) 0 0)))

;; (pir-go-1 :left (quaternion-translation-2 (g* +r-right+ (x-angle (degrees 120))) (aa::vec3 .403 .065 .003)) :time 4d0)

(defun screw-pitch (&key (time 10d0))
  (let* ((q0 +e-screw-left+)
         (e-rel +e-screw-rel+)
         (q1 (g* q0 e-rel))
         (q2 (g* q1 e-rel)))
  (pir-go :left (list (trajx-point q1 time)
                     (trajx-point q2  time)))))


(defun unscrew-pitch (side &key (time 10d0))
  (let* ((q0 (aa::g-chain +e-screw-left+ +e-screw-rel+ +e-screw-rel+))
         (q1 (g* q0 +e-unscrew-rel+))
         (q2 (g* q1 +e-unscrew-rel+)))
  (pir-go side (list (trajx-point q1 time)
                     (trajx-point q2  time)))))
  ;(pir-screw side (/ theta +rad/m+) theta :time time))


(defparameter +e-start-l+ (quaternion-translation-2 +r-up-in+
                                                    (aa::vec3 .4 .5 .3)))
(defparameter +q-start-l+
  (aa::vec -1.0010510457738677d0 -0.4106410664092259d0 1.0136872295583066d0
           -1.6339772457170914d0 1.2310156280166404d0 1.5006515441572446d0
           -1.8219317528643606d0))

(defparameter +q-screw-l+
  (aa::vec 0.9278519369452255d0 -0.654987161688432d0 -1.2634438455186952d0
           -1.7105622932946025d0 0.6518106624498023d0 1.2738809144456213d0
           -2.645866786145844d0))

(defun screw-approach ()
  (pir-go :left (list (trajx-point +e-screw-approach-left+ 12d0)
                      (trajx-point +e-screw-left+ 8d0))))

(defun screw-demo ()
  (let* ((q0 +e-screw-left+)
         (e-rel +e-screw-rel+)
         (q1 (g* q0 e-rel))
         (q2 (g* q1 e-rel)))
    (let* ((points (list (trajx-point +e-screw-approach-left+ 12d0)
                         (trajx-point q0  12d0)
                         (trajx-point q1  10d0)
                         (trajx-point q2  10d0)))
           (time (loop for p in points
                      summing (trajx-point-time p))))
      (pir-go :left points)
      (sleep (1+ time))
      (pir-pinch :left .14 .03)
      (sleep 1)
      (pir-go-1 :left (g* q2 (quaternion-translation-2 (x-angle 0) (aa::vec3 -10d-2 0 0)))
                :time 5d0)
      )))

(defun screwdriv (&key (time 10d0))
  (let* ((q0 +e-screw-approach-left+)
         (e-rel +e-screw-rel+)
         (q1 (g* q0 e-rel))
         (q2 (g* q1 e-rel)))
  (pir-go :left (list (trajx-point q1 time)
                     (trajx-point q2  time)))))

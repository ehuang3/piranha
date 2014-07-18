(in-package :piranha)

;; release camera
(pir-pinch :left .15 .015)

;; left init
(pir-go-1  :left *x-0*)

;; initial pick
(progn
  ;; pick cam
  (pir-go :left (list (trajx-point (g* *x* (aa::build-tf :x -.05))  8d0)
                      (trajx-point *x* 5d0)))
  (sleep 14)
  (pir-pinch :left .15 .0025 )
  (sleep 1)
  ;; position cam
  (pir-go :left (list (trajx-point (quaternion-translation-2 (aa::rotation *x*)
                                                             (aa::g+ (aa::translation *x*) (vec3 0 0 .1)))
                                   5d0)
                      (trajx-point *x-1* 7d0))))


;; second pose
(pir-go-1 :left *x-2* :time 5d0)

;; servo
(progn
  (setq *c-e-o* (sns:msg-aref (sns:msg-get *chan-marker* 'sns::msg-wt-tf) 41))
  (pir-servo-cam
   *c-e-o*
   (quaternion-translation-2 (g* +r-down+ (x-angle pi/2)) (vec3 0 0 .010))))


;; back-off
(pir-go-1  :right (quaternion-translation-2 (g* +r-down+ (x-angle pi/2)) (vec3 .55 -.3 -.3)))

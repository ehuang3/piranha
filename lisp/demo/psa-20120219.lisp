(in-package :pir)

(defvar *chan-obj*)
(defvar *cam-e-obj*)

(defparameter *body-e-cam*
  (aa::make-tf-tag
   :tf
   (quaternion-translation
    ;; static calibration
    (aa:vec -0.728869 0.682359 -0.018334 0.052918 0.443670 0.100229 0.635026))
   :parent 'body
   :child 'camera))


   ;; (aa:vec 0.742149	0.665537	-0.055754	0.056274
   ;;         0.418694	0.118289	0.632887)))

(defvar *cam-e-obj*)


;(g* *body-e-cam* *cam-e-obj*)

(defun ana-pose (vx vy vz qw qx qy qz)
  (aa::make-tf-tag :tf
                   (quaternion-translation-2 (aa::quaternion-4 qx qy qz qw)
                                             (vec3 vx vy vz))
                   :parent 'object
                   :child 'sdh))

(defun ana-sdh (axial r0 r1 l0 l1 t0 t1)
  (vec axial l0 l1 t0 t1 r0 r1))

(defstruct simple-grasp
  pose
  q0
  q1)

(defun ana-grasp (pose q0 q1)
  (make-simple-grasp :pose (apply #'ana-pose (map 'list #'identity pose))
                     :q0 (apply #'ana-sdh (map 'list #'identity q0))
                     :q1 (apply #'ana-sdh (map 'list #'identity q1))))


(defparameter *grasp-hinge*
  (ana-grasp
   '( -0.0478482 0.0153472 -0.143841 0.666231 -0.469807 -0.450957 -0.363394)
   '(0.597707 -0.539671 0.842593 -0.539671 0.842593 -0.539671 0.842593)
   '(0.597707 -0.438577 0.943687 -0.453733 0.928531 -0.522796 0.859468)))

(defparameter *grasp-hinge-2*
  (ana-grasp
   '(-0.0911991 0.0571066 -0.109 0.82394 0.350305 -0.445356 -0.00826581)
   '(0.814484 -0.473742 0.796809 -0.473742 0.796809 -0.473742 0.796809)
   '(0.814484 -0.359367 0.911184 -0.443117 0.827434 -0.386555 0.883996)))

(defparameter *grasp-clutch-1*
  (ana-grasp
   '(0.0669456 0.109431 0.0557195 0.0262162 -0.667703 0.728865 -0.149135)
   '(0.719672 -0.586089 0.890811 -0.586089 0.890811 -0.586089 0.890811)
   '(0.719672 -0.507964 0.968936 -0.438276 1.03862 -0.450776 1.02612)))

(defun pir-go-cam (side x)
  (pir-go-1 side (quaternion-translation-2 +r-down+ (aa:transform *body-e-cam* x))))


(defun go-obj-test ()
  (let ((xyz (aa::quaternion-translation-translation (aa::tf-tag-tf (g* *body-e-cam* *cam-e-obj*)))))
    (pir-go-1 :right (quaternion-translation-2 +r-down+
                                               (aa::g+ xyz (vec3 0 0 .05))))))

(defun sdh-trajx-1 (b-e-sdh &key (time 10d0))
  (let ((b-e-w (g* b-e-sdh (aa::inverse +wrist-e-sdh+))))
    (pir-go-1 :right b-e-w :point :wrist :time time)))

(defun sdh-trajx (points)
  (let ((points (loop for p in points
                   collect
                     (trajx-point (g* (trajx-point-pose p) (aa::inverse +wrist-e-sdh+))
                                  (trajx-point-time p)))))
    (pir-go :right points :point :wrist)))

(defun grasp-target (grasp &key (cam-e-obj *cam-e-obj*))
  (aa::g-chain  *body-e-cam* cam-e-obj (simple-grasp-pose grasp)))

(defun grasp-it (grasp &key (cam-e-obj *cam-e-obj*))
  (pir-sdh-set :right (simple-grasp-q0 grasp))
  (sleep 1)
  (let* ((sdh-pose (aa::tf-tag-tf (aa::g-chain  *body-e-cam* cam-e-obj (simple-grasp-pose grasp))))
         (sdh-approach (g* sdh-pose (aa::build-tf :x -.12))))
    (sdh-trajx (list (trajx-point sdh-approach 10d0)
                     (trajx-point sdh-pose 5d0))))
  (sleep 15)
  (pir-sdh-set :right (simple-grasp-q1 grasp)))


(defun cam-pose-kludge (cam-pose)
  (g* (aa::make-tf-tag :tf (aa::build-tf :z .00) :parent 'camera :child 'camera) cam-pose))

(defun obj-pose ()
  (let ((msg (sns:msg-get *chan-obj* 'sns:msg-tf :last t :wait t)))
    (setq *cam-e-obj*
          (aa::make-tf-tag :tf (sns:msg-aref msg 0)
                           :child 'object
                           :parent 'camera))))

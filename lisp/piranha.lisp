;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2013, Georgia Tech Research Corporation
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;; Georgia Tech Humanoid Robotics Lab
;;;; Under Direction of Prof. Mike Stilman
;;;;
;;;;
;;;; This file is provided under the following "BSD-style" License:
;;;;
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above copyright
;;;;     notice, this list of conditions and the following disclaimer.
;;;;
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
;;;;
;;;;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
;;;;   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
;;;;   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;;;;   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;;;;   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;;;   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
;;;;   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;;;;   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;;;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;;;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;;;   POSSIBILITY OF SUCH DAMAGE.

;; Author: Neil T. Dantam

(in-package :piranha)



(defvar *ctrl-channel* nil)
(defvar *state-channel* nil)
(defvar *config-channel* nil)
(defvar *complete-channel* nil)

(defvar *message-seq-no* 0)
(defvar *message-salt*)

(defvar *last-traj*)

(defvar *state*)
(defvar *statex*)

(defconstant pi/2 (* .5 pi))
(defconstant -pi/2 (* -.5 pi))

(defun pi* (x) (* x pi))

(defun degrees (x) (/ (* x pi) 180d0))

(defparameter +r-down+ (quaternion (y-angle pi/2)))
(defparameter +r-up-in+ (quaternion (y-angle -pi/2)))
(defparameter +r-left+ (g* (quaternion (z-angle pi/2))
                           (x-angle pi/2)))
(defparameter +r-right+ (g* (quaternion (z-angle -pi/2))
                            (x-angle -pi/2)))

(defun pir-start ()
  (assert (null *ctrl-channel*))
  (setq *ctrl-channel* (ach::open-channel "pir-ctrl"))
  (setq *config-channel* (ach::open-channel "pir-config"))
  (setq *state-channel* (ach::open-channel "pir-state"))
  (setq *complete-channel* (ach::open-channel "pir-complete")))

(defun pir-stop ()
  (ach::close-channel *ctrl-channel*)
  (setq *ctrl-channel* nil ))


(cffi:defcstruct pir-message
  (type :char :count 64)
  (salt :uint64)
  (seq-no :uint64)
  (n :uint64))

(cffi:defcstruct pir-message-complete
  (salt :uint64)
  (seq-no :uint64))


(cffi:defcstruct pir-cstate
  (q :double :count 29)
  (dq :double :count 29)

  (f-l :double :count 6)
  (f-r :double :count 6)

  (s-l :double :count 8)
  (s-r :double :count 8)

  (j-l :double :count #.(* 7 6))
  (j-r :double :count #.(* 7 6))

  (s-eer-l :double :count 8)
  (s-eer-r :double :count 8)

  )


(defstruct pir-state
  q         ; joint angles
  dq        ; joint velocities
  q-l       ; left joints
  q-r       ; right joints
  q-sdh-l   ; left sdh joints
  q-sdh-r   ; left sdh joints

  ;; s-l       ; left pose quaternion
  ;; s-r       ; right pose quaternion
  ;; s-eer-l   ; left end-effector relative
  ;; s-eer-r   ; right end-effector relative
  ;; s-f-l     ; left finger
  ;; s-f-r     ; right finger
  ;; r-l       ; left rotion quaternion
  ;; r-r       ; right rotion quaternion
  ;; x-l       ; left translation vector
  ;; x-r       ; right translation vector

  (e-l (quaternion-translation nil) :type aa::quaternion-translation)
  (e-r (quaternion-translation nil) :type aa::quaternion-translation)
  (e-eer-l (quaternion-translation nil) :type aa::quaternion-translation)
  (e-eer-r (quaternion-translation nil) :type aa::quaternion-translation)
  (e-f-l (quaternion-translation nil) :type aa::quaternion-translation)
  (e-f-r (quaternion-translation nil) :type aa::quaternion-translation)

  f-l
  f-r
  )


(defun read-doubles (pointer size &optional (offset 0))
  (let ((x (make-array size :element-type 'double-float)))
    (dotimes (i size)
      (setf (aref x i)
            (mem-aref pointer :double (+ i offset))))
    x))

(defun get-config ()
  (let ((config (make-array (* 2 2 (+ 7 7)) :element-type 'double-float)))
    (cffi:with-pointer-to-vector-data (ptr config)
      (let ((size (* (length config) 8)))
        (ach::get-pointer *config-channel* ptr size :wait t :last t)))
    config))

(defun get-state ()
  (with-foreign-object (state '(:struct pir-cstate))
    (ach:get-pointer *state-channel* state (foreign-type-size '(:struct pir-cstate))
                     :wait t :last t)
    (labels ((extract (x n)
               (read-doubles (foreign-slot-pointer state '(:struct pir-cstate) x) n))
             (extract-qutr (x)
               (quaternion-translation (dual-quaternion (extract x 8)))))
      (let ((e-l (extract-qutr 's-l))
            (e-r (extract-qutr 's-r))
            (e-eer-l (extract-qutr 's-eer-l))
            (e-eer-r (extract-qutr 's-eer-r))
            (q (extract 'q 29)))
        (setq *state*
              (make-pir-state
                   :q q
                   :dq (extract 'dq 29)
                   :q-l (amino::vec-copy q :start 1 :end 8)
                   :q-r (amino::vec-copy q :start 8 :end 15)
                   :q-sdh-l (amino::vec-copy q :start 15 :end 22)
                   :q-sdh-r (amino::vec-copy q :start 22 :end 28)
                   :f-l (extract 'f-l 6)
                   :f-r (extract 'f-r 6)

                   :e-l e-l
                   :e-r e-r
                   :e-eer-l e-eer-l
                   :e-eer-r e-eer-r
                   :e-f-l (g* e-l e-eer-l)
                   :e-f-r (g* e-r e-eer-r)

                   ;; :s-l s-l
                   ;; :s-r s-r
                   ;; :s-eer-l s-eer-l
                   ;; :s-eer-r s-eer-r
                   ;; :s-f-l (aa::tf-duqu-mul s-l s-eer-l)
                   ;; :s-f-r (aa::tf-duqu-mul s-r s-eer-r)

                   ;; :r-l r-l
                   ;; :r-r r-r
                   ;; :x-l x-l
                   ;; :x-r x-r

                   ))))))

(defun print-state (&optional (state (get-state)))
  (labels ((print-e (name e)
             (let ((list (matrix->list e)))
               (format t "~&~A-orientation: ~{~,3F~^ ~}~&" name (car list))
               (format t "~&~A-translation: ~{~,3F~^ ~}~&" name (cadr list)))))
    (print-e "left" (pir-state-e-l state))
    (print-e "right" (pir-state-e-r state))
    (print-e "left-finger" (pir-state-e-f-l state))
    (print-e "right-finger" (pir-state-e-f-r state)))
  )


(defun pir-set-mode (msg mode)
  (assert (< (length mode) 63))
  (labels ((setit (i val)
             (setf (mem-aref (foreign-slot-pointer msg '(:struct pir-message) 'type)
                             :char i)
                   val)))

    (loop for i below (length mode)
       do (setit i (char-code (aref mode i))))
    (setit (length mode) 0)))

(defun pir-message (mode &optional data)
  (with-foreign-pointer (msg (+ (foreign-type-size '(:struct pir-message))
                                (* 8 (length data)))
                             msg-size)
    (pir-set-mode msg mode)
    (setf (foreign-slot-value msg '(:struct pir-message) 'n)
          (length data))
    (setf (foreign-slot-value msg '(:struct pir-message) 'seq-no)
          (incf *message-seq-no*))
    (setf (foreign-slot-value msg '(:struct pir-message) 'salt)
          (setq *message-salt* (random (expt 2 64))))
    (dotimes (i (length data))
      (let ((pointer (inc-pointer (foreign-slot-pointer msg '(:struct pir-message) 'n)
                                  (* 8 (1+ i))))
            (x (elt data i)))
        (etypecase x
          (fixnum
           (setf (mem-aref pointer :int64) x))
          (double-float
           (setf (mem-aref pointer :double) x)))))
    (ach::put-pointer *ctrl-channel* msg msg-size)))

(defun pir-wait (&key (seq-no *message-seq-no*) (salt *message-salt*))
  (labels ((get-msg ()
             (with-foreign-object (msg '(:struct pir-message-complete))
               (ach::get-pointer *complete-channel* msg (foreign-type-size '(:struct pir-message-complete))
                                 :wait t)
               (values (foreign-slot-value msg '(:struct pir-message-complete) 'salt)
                       (foreign-slot-value msg '(:struct pir-message-complete) 'seq-no))))
           (done-p ()
             (multiple-value-bind (msg-salt msg-seq-no) (get-msg)
               (if (and (= salt msg-salt)
                        (= seq-no msg-seq-no))
                   t
                   (progn
                     (unless (= msg-seq-no (1- seq-no))
                       (format t "~&Unexpected values -- salt: ~D, seq-no ~D~&"
                               msg-salt msg-seq-no))
                     nil)))))
    ;; TODO: should also do a sanity check of state message to ensure
    ;; velocity is sufficiently small
    (format t "Waiting for salt: ~D, seq-no: ~D~&"
            salt seq-no)
    (loop until (done-p))))

(defstruct trajx-point
  pose
  time)

(defun trajx-point (pose time)
  (make-trajx-point :pose (dual-quaternion pose)
                    :time time))

(defun trajx-point-data (points)
  (let* ((n (length points))
         (data (amino::make-vec (* 9 n))))
    (dotimes (i n)
      (let ((point (elt points i))
            (offset (* i (/ (length data) n))))
        (setf (aref data offset)
              (trajx-point-time point))
        (replace data (amino::dual-quaternion-data (trajx-point-pose point))
                 :start1 (+ 1 offset) :end1 (+ 9 offset))))
    data))

(defun pir-go (side points &key
               (point :finger)
               (state (get-state)))
  (let ((thing (ecase point
                 (:finger "trajx")
                 (:wrist "trajx-w"))))
    (setq *last-traj* (append (list (trajx-point (pir-state-e-l state)
                                                 0d0))
                              points))
    (pir-message (side-case side thing) (trajx-point-data points))))

(defun pir-go-1 (side s &key (point :finger) (time 10d0))
  (pir-go side (list (trajx-point s time))
          :point point))


(defun pir-go-rel (side &key
                   (x 0d0)
                   (y 0d0)
                   (z 0d0)
                   (xyz (amino::vec3 x y z))
                   (time 5d0)
                   (r amino::+tf-quat-ident+))
  (let ((state (get-state)))
    (let* ((e-rel (quaternion-translation-2 r xyz))
           (e-0 (ecase side
                  (:left (pir-state-e-f-l state))
                  (:right (pir-state-e-f-r state)))))
      (pir-go side (list (trajx-point (g* e-0 e-rel)
                                      time))
              :state state))))

(defun pir-screw (side x theta &key (time 5d0))
  (pir-go-rel side
              :x x
              :r (amino::tf-xangle2quat theta)
              :time time))

(defun pir-rotate (side r &key (time 10d0))
  (let ((state (get-state)))
    (pir-go side (list (make-trajx-point :pose (amino:dual-quaternion-2 r
                                                                  (aa::quaternion-translation-translation
                                                                   (ecase side
                                                                     (:left (pir-state-e-f-l state))
                                                                     (:right (pir-state-e-f-r state)))))
                                         :time time))
            :state state)))

(defun pir-zrotate (side theta &key (time 10d0))
  (pir-rotate side
              (aa::g* +r-down+ (x-angle theta))
              :time time))

(defstruct trajq-point
  q
  time)

(defun trajq-point-data (points)
  (let* ((k (length (trajq-point-q (car points))))
         (p (+ 1 k))
         (n (length points))
         (data (amino::make-vec (* p n))))
    (dotimes (i n)
      (let ((point (elt points i))
            (offset (* i (/ (length data) n))))
        (assert (= k (length (trajq-point-q point))))
        (setf (aref data offset)
              (trajq-point-time point))
        (replace data (amino::matrix-data (trajq-point-q point))
                 :start1 (+ 1 offset))))
    data))



(defun side-case (side base)
  (concatenate 'string base "-"
               (ecase side
                 (:lr "lr")
                 (:left "left")
                 (:right "right"))))

(defun pir-trajq (side points)
  (pir-message (side-case side "trajq")
               (trajq-point-data points)))


(defun pir-set-side (side q &key (time 10d0))
  (check-type q (simple-array double-float (7)))
  (pir-trajq-side side (list (make-trajq-point :q q :time time))))

(defun pir-set (side q &key (time 10d0))
  ;(check-type q (simple-array double-float (7)))
  (pir-trajq side (list (make-trajq-point :q q :time time))))


(defun pir-set-axis (side i  q &key (time 10d0) (state (get-state)))
  (let* ((q0 (ecase side (:left
                          (pir-state-q-l state))
                    (:right
                     (pir-state-q-r state))))
         (q1 (aa::vec-copy q0)))
    (setf (aa::vecref q1 i) q)
    (pir-set side q1 :time time)))

(defun pir-torso (q &key (time 10d0))
  (pir-message "trajq-torso" (aa::vec time q)))

(defun pir-sdh-set (side q)
  (check-type q (simple-array double-float (7)))
  (pir-message (side-case side "sdh-set")
               q))

(defun pir-sdh-set-pi (q0 q1 q2 q3 q4 q5 q6)
  (let ((v (make-array 7 :element-type 'double-float)))
    (setf (aref v 0) (* pi (coerce q0 'double-float))
          (aref v 1) (* pi (coerce q1 'double-float))
          (aref v 2) (* pi (coerce q2 'double-float))
          (aref v 3) (* pi (coerce q3 'double-float))
          (aref v 4) (* pi (coerce q4 'double-float))
          (aref v 5) (* pi (coerce q5 'double-float))
          (aref v 6) (* pi (coerce q6 'double-float)))
  (pir-message "sdh-set" v)))

(defparameter *q-zero*
  (aa::vec 0 0 0 0 0 0 0))

;; Right Arm


(defparameter *q-store-l* (aa::vec (* 0.5 pi) (* -0.5 pi) 0 0 0 0 0))
(defparameter *q-up-l* (aa::vec (* 0.5 pi) (* -0.2 pi) 0 (* -0.2 pi) 0 (* .55 pi) 0))
(defparameter *q-over-l* (aa::vec (* 0.1 pi) (* -0.2 pi) 0 (* -0.2 pi) 0 (* .55 pi) 0))
(defparameter *q-go-l* (aa::vec 0.6127501937901693d0 -1.063760725798024d0 -0.7395658172400772d0
                                -1.2926781104896001d0 -0.940331041096985d0 1.3763666481227284d0
                                0.5270894341022875d0))
(defparameter *q-go-r* (aa::vec -0.5366189318181765d0 0.9482722891935591d0 0.5890311692555663d0
                                 1.286272752134781d0 1.0863976461963905d0 -1.4665827171583152d0
                                 -1.585422185926609d0))

(defun pir-mirror-l->r (q)
  (aa::g* -1d0 q))

(defun pir-mirror-cat (q)
  (aa::veccat q (aa::g* -1d0 q)))

(defun pir-trajq-side-point (side point)
  (ecase side
    (:left point)
    (:right (make-trajq-point :q (pir-mirror-l->r (trajq-point-q point))
                              :time (trajq-point-time point)))
    (:lr (make-trajq-point :q (pir-mirror-cat (trajq-point-q point))
                              :time (trajq-point-time point)))))


(defun pir-trajq-side (side traj)
  (pir-trajq side (loop for p in traj
                     collect (pir-trajq-side-point side p))))


(defun pir-trajq-zero->store (side)
  (pir-trajq-side side (list (make-trajq-point :q (aa::vec (* 0.5 pi) 0 0 0 0 0 0) :time 5d0)
                             (make-trajq-point :q (aa::vec (* 0.5 pi) (* -.5 pi) 0 0 0 0 0) :time 15d0))))


(defun pir-trajq-zero->table (side)
  (pir-trajq-side side (list (make-trajq-point :q (aa::vec (* 0.5 pi) 0 0 0 0 0 0) :time 5d0)
                             (make-trajq-point :q *q-up-l* :time 10d0)
                             (make-trajq-point :q *q-over-l* :time 10d0)
                             (make-trajq-point :q *q-go-l* :time 5d0))))

(defun pir-trajq-store->zero (side)
  (pir-trajq-side side(list (make-trajq-point :q (aa::vec (* 0.5 pi) 0 0 0 0 0 0) :time 10d0)
                            (make-trajq-point :q *q-zero* :time 5d0))))

(defun pir-trajq-table->store (side)
  (pir-trajq-side side (list (make-trajq-point :q *q-go-l* :time 8d0)
                             (make-trajq-point :q *q-over-l* :time 8d0)
                             (make-trajq-point :q *q-up-l* :time 7d0)
                             (make-trajq-point :q *q-store-l* :time 5d0))))

(defun pir-trajq-store->table (side)
  (pir-trajq-side side (list (make-trajq-point :q *q-up-l* :time 8d0)
                             (make-trajq-point :q *q-over-l* :time 10d0)
                             (make-trajq-point :q *q-go-l* :time 5d0))))

(defun pir-table ()
  (pir-trajq :lr (list (make-trajq-point :q (pir-mirror-cat *q-up-l*)
                                         :time 8d0)
                       (make-trajq-point :q (pir-mirror-cat *q-over-l*)
                                         :time 10d0)
                       (make-trajq-point :q (aa::veccat *q-go-l* *q-go-r*)
                                         :time 5d0))))


(defun pir-store ()
  ;; TODO: check if near go position already
  (pir-trajq :lr (list (make-trajq-point :q (aa::veccat *q-go-l* *q-go-r*)
                                         :time 8d0)
                       (make-trajq-point :q (pir-mirror-cat *q-over-l*)
                                         :time 8d0)
                       (make-trajq-point :q (pir-mirror-cat *q-up-l*)
                                         :time 7d0)
                       (make-trajq-point :q (pir-mirror-cat *q-store-l*)
                                         :time 5d0))))


(defun pir-pinch (side y r)
  (pir-message (ecase side
                 (:left "pinch-left")
                 (:right "pinch-right"))
               (aa::vec r y)))

(defun pir-zero (side &optional (time 5d0))
  (pir-set side *q-zero* :time time))

;; (defun pir-pose ()
;;   (let ((s (pir-state-s-f-l (get-state))))
;;     (values s
;;             (aa::tf-duqu-trans s))))


(defun xyz2duqu (x y z)
  (aa::tf-xxyz2duqu 0d0 x y z))

(defun pose-hover (s z)
  (aa::tf-duqu-mul s
                   (aa::tf-qv2duqu (amino::tf-yangle2quat (/ pi 2)) (aa::vec 0d0 0d0 z))))
(defun go-grasp (side s z0 z1 &key (t0 5d0) (t1 10d0))
  (pir-go side (list (make-trajx-point :pose (pose-hover s z0)
                                       :time t0)
                     (make-trajx-point :pose (pose-hover s z1)
                                       :time t1))))

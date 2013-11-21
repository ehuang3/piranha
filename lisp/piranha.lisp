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
(defvar *complete-channel* nil)

(defvar *message-seq-no* 0)
(defvar *message-salt*)

(defvar *last-traj*)

(defvar *state*)

(defconstant pi/2 (* .5 pi))
(defconstant -pi/2 (* -.5 pi))

(defun pir-start ()
  (assert (null *ctrl-channel*))
  (setq *ctrl-channel* (ach::open-channel "pir-ctrl"))
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
  s-l       ; left pose quaternion
  s-r       ; right pose quaternion
  s-eer-l   ; left end-effector relative
  s-eer-r   ; right end-effector relative
  s-f-l     ; left finger
  s-f-r     ; right finger
  r-l       ; left rotion quaternion
  r-r       ; right rotion quaternion
  x-l       ; left translation vector
  x-r       ; right translation vector
  f-l
  f-r
  )


(defun read-doubles (pointer size)
  (let ((x (make-array size :element-type 'double-float)))
    (dotimes (i size)
      (setf (aref x i)
            (mem-aref pointer :double i)))
    x))

(defun get-state ()
  (with-foreign-object (state '(:struct pir-cstate))
    (ach:get-pointer *state-channel* state (foreign-type-size '(:struct pir-cstate))
                     :wait t :last t)
    (labels ((extract (x n)
               (read-doubles (foreign-slot-pointer state '(:struct pir-cstate) x) n)))
      (let ((s-l (extract 's-l 8))
            (s-r (extract 's-r 8))
            (s-eer-l (extract 's-eer-l 8))
            (s-eer-r (extract 's-eer-r 8))
            (q (extract 'q 29)))
        (multiple-value-bind (r-l x-l) (amino::tf-duqu2qv s-l)
          (multiple-value-bind (r-r x-r) (amino::tf-duqu2qv s-r)
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
                   :s-l s-l
                   :s-r s-r
                   :s-eer-l s-eer-l
                   :s-eer-r s-eer-r
                   :s-f-l (aa::tf-duqu-mul s-l s-eer-l)
                   :s-f-r (aa::tf-duqu-mul s-r s-eer-r)
                   :r-l r-l
                   :r-r r-r
                   :x-l x-l
                   :x-r x-r))))))))

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

(defun trajx-point-data (points)
  (let* ((n (length points))
         (data (amino::make-vec (* 9 n))))
    (dotimes (i n)
      (let ((point (elt points i))
            (offset (* i (/ (length data) n))))
        (setf (aref data offset)
              (trajx-point-time point))
        (replace data (amino::matrix-data (trajx-point-pose point))
                 :start1 (+ 1 offset) :end1 (+ 9 offset))))
    data))

(defun pir-go (points &key
               (state (get-state)))
  (setq *last-traj* (append (list (make-trajx-point :pose (pir-state-s-l state)
                                                    :time 0d0))
                            points))
  (pir-message "trajx" (trajx-point-data points)))

(defun pir-go-1 (s &optional (time 10d0))
  (let ((state (get-state)))
    (pir-go (list (make-trajx-point :pose s
                                    :time time))
            :state state)))


(defun pir-go-rel (&key
                   (x 0d0)
                   (y 0d0)
                   (z 0d0)
                   (xyz (amino::col-vector x y z))
                   (time 5d0)
                   (r amino::+tf-quat-ident+))
  (let ((state (get-state)))
    (let* ((S-rel (amino::tf-qv2duqu r xyz))
           (S-1 (amino::tf-duqu-mul (pir-state-s-f-l state) S-rel)))
      (pir-go (list (make-trajx-point :pose S-1
                                      :time time))
              :state state))))

(defun pir-screw (x theta &key (time 5d0))
  (pir-go-rel :x x
              :r (amino::tf-xangle2quat theta)
              :time time))

(defun pir-rotate (r &key (time 10d0))
  (let ((state (get-state)))
    (pir-go (list (make-trajx-point :pose (amino::tf-qv2duqu r
                                                             (aa::tf-duqu-trans (pir-state-s-f-l state)))
                                    :time time))
            :state state)))

(defun pir-zrotate (theta &key (time 10d0))
  (pir-rotate (aa::tf-qmul (aa::tf-yangle2quat (/ pi 2)) (aa::tf-xangle2quat theta)) :time time))

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


(defun pir-set (side q &key (time 10d0))
  (check-type q (simple-array double-float (7)))
  (pir-trajq side (list (make-trajq-point :q q :time time))))


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
(defparameter *q-go-l* (aa::vec 0.7729539658307287d0 -1.0176316736678137d0 -0.9906488834319814d0
                                -1.4043966359097573d0 -0.8265181205744347d0 1.306605837920515d0
                                -0.07307693578100258d0))

(defun pir-trajq-side-point (side point)
  (ecase side
    (:left point)
    (:right (make-trajq-point :q (aa::g* -1d0 (trajq-point-q point))
                              :time (trajq-point-time point)))
    (:lr (make-trajq-point :q (aa::veccat (trajq-point-q point)
                                          (aa::g* -1d0 (trajq-point-q point)))
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


(defun pir-pinch (side y r)
  (pir-message (ecase side
                 (:left "pinch-left")
                 (:right "pinch-right"))
               (aa::vec r y)))

(defun pir-zero (side &optional (time 5d0))
  (pir-set side *q-zero* :time time))

(defun pir-pose ()
  (let ((s (pir-state-s-f-l (get-state))))
    (values s
            (aa::tf-duqu-trans s))))


(defun xyz2duqu (x y z)
  (aa::tf-xxyz2duqu 0d0 x y z))

(defun pose-hover (s z)
  (aa::tf-duqu-mul s
                   (aa::tf-qv2duqu (amino::tf-yangle2quat (/ pi 2)) (aa::vec 0d0 0d0 z))))
(defun go-grasp (s z0 z1 &key (t0 5d0) (t1 10d0))
  (pir-go (list (make-trajx-point :pose (pose-hover s z0)
                                  :time t0)
                (make-trajx-point :pose (pose-hover s z1)
                                  :time t1))))

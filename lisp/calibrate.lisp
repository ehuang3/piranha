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

(defvar *tf-camera*)

(defvar *chan-marker*)




(defparameter +cal-right-q+
  (aa::vec 0 0 0 0 0 0 0))

(defparameter +cal-right-x-min+ .45)
(defparameter +cal-right-x-max+ .5001)

(defparameter +cal-right-y-min+ -.0)
(defparameter +cal-right-y-max+ .4001)

(defparameter +cal-right-z-min+ -.02)
(defparameter +cal-right-z-max+ .0801)

(defparameter +cal-right-rot0+
  (g* +r-left+ (x-angle (degrees 90))))

(defparameter +cal-right-rot+
  (list
   +cal-right-rot0+
   (aa:g* +cal-right-rot0+ (y-angle (degrees -10)))
   (aa:g* +cal-right-rot0+ (y-angle (degrees 20)))
   (aa:g* +cal-right-rot0+ (x-angle (degrees 20)))
   (aa:g* +cal-right-rot0+ (x-angle (degrees -20)))
   (aa:g* +cal-right-rot0+ (z-angle (degrees 20)))
   (aa:g* +cal-right-rot0+ (z-angle (degrees -10)))))



(defun cal-go-sleep (e time side)
  (format t "~&Going to ~A in ~As..." e time)
  (pir-go-1 side e :time time)
  (sleep (1+ time))
  (format t "~&done"))

(defun pir-cal-1 (tf pid &key
                  (side :right)
                  (time 5d0))
  (cal-go-sleep tf time side)
  (sb-posix:kill pid sb-posix:sigusr1)
  (sleep 4)
  )


(defun cal-loop-dim (i min max dx function &optional (state (get-state)))
  (let ((x (aa::vec3-ref (aa::quaternion-translation-translation (pir-state-e-f-r state))
                         i)))
    (if (< (abs (- x min))
           (abs (- x max)))
        (loop for x = min then (+ x dx)
           while (<= x max)
           do (funcall function x))
        (loop for x = max then (- x dx)
           while (>= x min)
           do (funcall function x)))))

(defun pir-cal (pid &key
                (dx 5e-2)
                (dy 10e-2)
                (dz 10e-2)
                (side :right))

  (dolist (r +cal-right-rot+)
    ;; go to initial pose
    (cal-loop-dim
     0 +cal-right-x-min+ +cal-right-x-max+ dx
     (lambda (x)
       (cal-loop-dim
        1 +cal-right-y-min+ +cal-right-y-max+ dy
        (lambda (y)
          (cal-loop-dim
           2 +cal-right-z-min+ +cal-right-z-max+ dz
           (lambda (z)
             (pir-cal-1 (quaternion-translation-2 r
                                                  (aa::vec3 x y z))
                        pid :side side)))))))))


    ;; (cal-go-sleep (aa:quaternion-translation-2 r (aa::vec3 +cal-right-x-min+
    ;;                                                        +cal-right-y-min+
    ;;                                                        +cal-right-z-min+))
    ;;               20d0 side)
    ;; ;; loop through translations
    ;; (loop for x = +cal-right-x-min+ then (+ x dx)
    ;;    while (< x +cal-right-x-max+)
    ;;    do (loop for y = +cal-right-y-min+ then (+ y dx)
    ;;          while (< y +cal-right-y-max+)
    ;;          do (loop for z = +cal-right-z-min+ then (+ z dx)
    ;;                while (< z +cal-right-z-max+)
    ;;                do (pir-cal-1 (quaternion-translation-2 r
    ;;                                                        (aa::vec3 x y z))
    ;;                              pid :side side))))))
;; (defun pir-cal (pid &key
;;                 (dx 5e-2)
;;                 (side :right))

;;   (dolist (r +cal-right-rot+)
;;     ;; go to initial pose
;;     (cal-go-sleep (aa:quaternion-translation-2 r (aa::vec3 +cal-right-x-min+
;;                                                            +cal-right-y-min+
;;                                                            +cal-right-z-min+))
;;                   20d0 side)
;;     ;; loop through translations
;;     (loop for x = +cal-right-x-min+ then (+ x dx)
;;        while (< x +cal-right-x-max+)
;;        do (loop for y = +cal-right-y-min+ then (+ y dx)
;;              while (< y +cal-right-y-max+)
;;              do (loop for z = +cal-right-z-min+ then (+ z dx)
;;                    while (< z +cal-right-z-max+)
;;                    do (pir-cal-1 (quaternion-translation-2 r
;;                                                            (aa::vec3 x y z))
;;                                  pid :side side))))))

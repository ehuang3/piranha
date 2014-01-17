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


;;; (frame :name NAME
;;;        :type (or fixed revolute)
;;;        :axis (x y z)
;;;        :configuration NAME
;;;        :translation (x y z))

(defconstant pi/2 (* .5 pi))
(defconstant -pi/2 (* -.5 pi))

;; Try to load Quicklisp or ASDF
(unless (find-package :quicklisp)
  (let ((ql (find-if #'probe-file
                     (map 'list (lambda (setup) (merge-pathnames setup (user-homedir-pathname)))
                          '("quicklisp/setup.lisp" ".quicklisp/setup.lisp" "Quicklisp/setup.lisp")))))
    (cond
      (ql (load ql))
      ((not (find-package :asdf))
       (require :asdf)))))

;; Guess where some ASDF files lives
(loop for pathname in (list "./src/"
                            (merge-pathnames ".asdf/systems/"
                                             (user-homedir-pathname))
                            (merge-pathnames ".sbcl/systems/"
                                             (user-homedir-pathname)))

   do (when (probe-file pathname)
        (pushnew pathname asdf:*central-registry* :test #'equal)))

;; Load Motion-Grammar-Kit
(progn
  (if (find-package :quicklisp)
      (funcall (intern "QUICKLOAD" :ql) :reflex)
      (require :reflex)))


;;; Now, define the LWA4 frames
(defparameter *lwa4-frames*
  `((:frame :name "SHOULDER0"
            :configuration "Q_SHOULDER0"
            :type :revolute
            :axis (-1 0 0)
            :translation (0 0 0))
    (:frame :name "SHOULDER1"
            :parent "SHOULDER0"
            :configuration "Q_SHOULDER1"
            :type :revolute
            :axis (0 -1 0)
            :translation (0 0 0))
    (:frame :name "SHOULDER2"
            :parent "SHOULDER1"
            :configuration "Q_SHOULDER2"
            :type :revolute
            :axis (-1 0 0)
            :translation (0 0 0))
    (:frame :name "ELBOW"
            :parent "SHOULDER2"
            :configuration "Q_ELBOW"
            :type :revolute
            :axis (0 -1 0)
            :translation (LWA4_L_1 0 0))
    (:frame :name "WRIST0"
            :parent "ELBOW"
            :type :revolute
            :configuration "Q_WRIST0"
            :axis (-1 0 0)
            :translation (LWA4_L_2 0 0))
    (:frame :name "WRIST1"
            :parent "WRIST0"
            :configuration "Q_WRIST1"
            :type :revolute
            :axis (0 1 0)
            :translation (0 0 0))
    (:frame :name "WRIST2"
            :parent "WRIST1"
            :configuration "Q_WRIST2"
            :type :revolute
            :axis (1 0 0)
            :translation (0 0 0))))


(defun sdh-finger (parent prefix z)
  (reflex:prefix-frames
   parent prefix
   `((:frame :name "0"
             :configuration "Q_0"
             :type :revolute
             :axis (0 1 0)
             :translation (0 0 ,z))
     (:frame :name "1"
             :configuration "Q_1"
             :parent "0"
             :type :revolute
             :axis (0 1 0)
             :translation ("SDH_L1" 0 0))
     ;; finger tip
     ,(reflex:make-fixed-frame "2" "1" (aa:x-angle 0) :x "SDH_L2")
     ;; Knuckles
     ,(reflex:make-fixed-frame "K0M" parent (aa:x-angle -pi/2)  :y "-SDH_L0M" :z z)
     ,(reflex:make-fixed-frame "K0P" parent (aa:x-angle pi/2)  :y "SDH_L0P" :z z)
     ,(reflex:make-fixed-frame "K1M" "1" (aa:x-angle -pi/2)  :x "SDH_L1" :y "-SDH_L0M")
     ,(reflex:make-fixed-frame "K1P" "1" (aa:x-angle pi/2)  :x "SDH_L1" :y "SDH_L0P")
     )))

(defun sdh (parent prefix)
  (reflex:prefix-frames
   parent prefix
   `(,(reflex:make-fixed-frame "BASE" nil (aa:x-angle (* -60 (/ pi 180))) :x "LWA4_L_e + LWA4_FT_L")
      ,(reflex:make-fixed-frame "CENTER" "BASE" (aa:x-angle 0) :x "SDH_LB")
      ,(reflex:make-fixed-frame "FINGERTIP" "BASE" (aa:x-angle 0))
      (:frame :name "L_AXIAL"
              :parent "CENTER"
              :type :revolute
              :configuration "Q_AXIAL"
              :offset "-M_PI"
              :axis (-1 0 0)
              :translation (0 "-SDH_B/2" "-SDH_FC"))
      (:frame :name "R_AXIAL"
              :parent "CENTER"
              :type :revolute
              :configuration "Q_AXIAL"
              :offset "M_PI"
              :axis (1 0 0)
              :translation (0 "SDH_B/2" "-SDH_FC"))
      ,@(sdh-finger "L_AXIAL" "L_" 0)
      ,@(sdh-finger "CENTER" "T_" "SDH_TC")
      ,@(sdh-finger "R_AXIAL" "R_" 0))))

(defparameter *pir-arm*
  `( ,@*lwa4-frames*
    ,(reflex:make-fixed-frame "FT" "WRIST2"
                              (aa:g* (aa:quaternion (aa::col-matrix '(0 1 0) '(0 0 1) '(1 0 0)))
                                     (aa:quaternion (aa:z-angle (* .25 pi)))))
    ,(reflex:make-fixed-frame "HAND_BASE" "WRIST2"
                              (amino:x-angle (* -60 (/ pi 180)))
                              :X "LWA4_L_e + LWA4_FT_L")))

(defun pir-frames ()
  (append
   `((:frame :name "TORSO"
             :type :fixed
             :quaternion (0 0 0 1)
             :translation (0 0 0))
     ,(reflex:make-fixed-frame "LEFT_BASE" "TORSO"
                               (aa:quaternion (aa::col-matrix '(0 1 0)
                                                              '(0 0 -1)
                                                              '(-1 0 0)))
                               :y "LWA4_L_0 + PIR_L_SHOULDER_WIDTH/2 - LWA4_L_P")
     ,(reflex:make-fixed-frame "RIGHT_BASE" "TORSO"
                               (aa:quaternion (aa::col-matrix '(0 -1 0)
                                                              '(0 0 -1)
                                                              '(1 0 0)))
                               :y "-(LWA4_L_0 + PIR_L_SHOULDER_WIDTH/2 - LWA4_L_P)"))
   (reflex:prefix-frames "LEFT_BASE" "LEFT_" *pir-arm*)
   (reflex:prefix-frames "RIGHT_BASE" "RIGHT_" *pir-arm*)
   (sdh "LEFT_WRIST2" "LEFT_SDH_")
   (sdh "RIGHT_WRIST2" "RIGHT_SDH_")
   ))


(let ((frames (reflex:prefix-frames nil "PIR_TF_" (pir-frames))))
  (reflex:write-frame-files "pir-frame.h" "pir-frame.c" frames
                            :frame-max "PIR_TF_FRAME_MAX"
                            :configuration-max "PIR_TF_CONFIG_MAX"
                            :headers '("pir-param.h")
                            :relative-function "pir_tf_rel"
                            :absolute-function "pir_tf_abs"
                            :normalize t
                            :parents-array "pir_tf_parents"
                            :names-array "pir_tf_names"
                            :dot-file "pir-frame.dot"
                            :headers '("pir-frame.h")))

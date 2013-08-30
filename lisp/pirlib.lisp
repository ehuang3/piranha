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


(define-foreign-library libreflex
  (:unix (:or #.(concatenate 'string (namestring (user-homedir-pathname)) "/lib/libreflex.so")
              #.(concatenate 'string (namestring (user-homedir-pathname)) "/local/lib/libreflex.so")
              "libreflex.so"))
  (t (:default "libreflex")))

(define-foreign-library libpiranha
  (:unix (:or #.(concatenate 'string (namestring (user-homedir-pathname)) "/lib/libpiranha.so")
              #.(concatenate 'string (namestring (user-homedir-pathname)) "/local/lib/libpiranha.so")
              "libpiranha.so"))
  (t (:default "libpiranha")))

(use-foreign-library libreflex)

(use-foreign-library libpiranha)

(define-foreign-type lwa4-config-t ()
  ()
  (:simple-parser lwa4-config-t)
  (:actual-type :pointer))

(defmethod cffi:expand-to-foreign-dyn (value var body (type lwa4-config-t))
  (amino::expand-vector value var body 7))


(define-foreign-type lwa4-jacobian-t ()
  ()
  (:simple-parser lwa4-jacobian-t)
  (:actual-type :pointer))

(defmethod cffi:expand-to-foreign-dyn (value var body (type lwa4-jacobian-t))
  (amino::expand-vector value var body (* 6 7)))

;; FK

(cffi:defcfun lwa4-kin-duqu :void
  (q lwa4-config-t)
  (S-0 amino::dual-quaternion-t)
  (S-ee amino::dual-quaternion-t)
  (S amino::dual-quaternion-t)
  (J :pointer))

(defun pir-fk (q &key
               (S-0 amino::+tf-duqu-ident+)
               (S-ee amino::+tf-duqu-ident+)
               (S (amino::make-vec 8)))
  (lwa4-kin-duqu q S-0 S-ee S (cffi:null-pointer))
  S)

;; IK


(cffi:defcfun pir-kin-solve :int
  (q0 lwa4-config-t)
  (S0 amino::dual-quaternion-t)
  (q1 lwa4-config-t))


(defun pir-ik (q0 S &key
               (q1 (amino::make-vec 7)))
  (pir-kin-solve q0 S q1)
  q1)

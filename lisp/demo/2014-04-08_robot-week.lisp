(in-package :pir)

(defparameter +q-r-wave-0+
  (vec 0.6775193623316788d0 1.3165716679494026d0 -1.3637130110457694d0
       1.2390615958683344d0 2.771129066561477d0 1.0862754731487507d0
       -2.205991454765713d0))



(defun wave ()
  (let* ((state (get-state))
         (e-0 (pir-state-e-r state))
         (e-r (quaternion-translation-2 (quaternion (y-angle (degrees 30)))
                                        (vec3 0 0 0)))
         (e-1 (g* e-0 e-r))
         (e-2 (g* e-0 (inverse e-r))))
    (pir-go :right (list (trajx-point e-1 2.1d0)
                         (trajx-point e-2 2.5d0)
                         (trajx-point e-0 2.1d0))
            :point :wrist)))

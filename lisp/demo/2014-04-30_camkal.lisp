(in-package :piranha)

(defvar *chan-marker*)
(defvar *chan-reg*)
(defvar *demo-target*)

(defun demo-start ()
  (setq *chan-marker* (ach::open-channel "markers")))
  ;(setq *chan-reg* (ach::open-channel "pir-reg")))

(defun get-target ()
  (setq *demo-target* (sns::msg-aref (sns::get-msg *chan-marker* 'sns::msg-wt-tf)
                                     36)))

(defun go-target ()
    (pir-servo-cam
     *demo-target*
     (quaternion-translation-2 (g* +r-down+ (x-angle (* .75 pi))) (vec3 0 0 .050))))


  ;; (let ((cam-e-obj (aa::make-tf-tag
  ;;                   :tf (sns::msg-aref (sns::get-msg *chan-marker* 'sns::msg-wt-tf)
  ;;                                      36)
  ;;                   :parent 'cam
  ;;                   :child 'obj))
  ;;       (body-e-cam (aa::make-tf-tag
  ;;                    :tf (sns::msg-aref (sns::get-msg *chan-reg* 'sns::msg-tf)
  ;;                                       1)
  ;;                    :parent 'body
  ;;                    :child 'cam)))
  ;;   (setq *demo-target*
  ;;         (aa:g* body-e-cam cam-e-obj))))

(load "./wheeled_robot.l")
(setq *robot* (wheeled_robot))

(defun pose1 ()
  (setq *box* (make-cube 100 100 100))
  (objects (list *robot* *box*))
  (send *robot* :move-to (make-coords :pos #f(0 50 300)) :world)
  (send *robot* :angle-vector #f(0.0 0.0 70.0 -120.0 30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  )

(defun pose2 ()
  (objects (list *robot*))
  (send *robot* :move-to (make-coords :pos #f(0 50 300)) :world)
  (setq av-list  #f(0.0 0.0 70.0 -120.0 30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)))

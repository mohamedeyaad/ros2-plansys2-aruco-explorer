(define (domain arucobot)
  (:requirements :strips :typing :durative-actions)

  (:types
    robot
    waypoint
  )

  (:predicates
    (at ?r - robot ?w - waypoint)
    (connected ?w1 ?w2 - waypoint)
    (surveyed ?w - waypoint)       ; We checked this WP
    (picture_taken ?w - waypoint)  ; We finished the task here
  )

  (:durative-action move
    :parameters (?r - robot ?from ?to - waypoint)
    :duration ( = ?duration 10)
    :condition (and
      (at start (at ?r ?from))
      (at start (connected ?from ?to))
    )
    :effect (and
      (at start (not (at ?r ?from)))
      (at end (at ?r ?to))
    )
  )

  ;; PHASE 1 ACTION: Just look and report ID
  (:durative-action survey
    :parameters (?r - robot ?w - waypoint)
    :duration ( = ?duration 5)
    :condition (and
      (at start (at ?r ?w))
    )
    :effect (and
      (at end (surveyed ?w))
    )
  )

  ;; PHASE 2 ACTION: Center and Take Picture
  (:durative-action take_picture
    :parameters (?r - robot ?w - waypoint)
    :duration ( = ?duration 10)
    :condition (and
      (at start (at ?r ?w))
    )
    :effect (and
      (at end (picture_taken ?w))
    )
  )
)
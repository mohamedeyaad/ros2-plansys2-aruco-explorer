(define (problem mission_setup)
    (:domain arucobot_domain) 

    ;; 1. Define Objects (No more "set instance..." in Python!)
    (:objects
        r - robot
        wp_start wp1 wp2 wp3 wp4 - waypoint
    )

    ;; 2. Define Initial State (No more "set predicate..." in Python!)
    (:init
        (at r wp_start)
        
        ;; The Static Map
        (connected wp_start wp1) (connected wp1 wp_start)
        (connected wp1 wp2)      (connected wp2 wp1)
        (connected wp2 wp3)      (connected wp3 wp2)
        (connected wp3 wp4)      (connected wp4 wp3)
        (connected wp4 wp1)      (connected wp1 wp4)
        (connected wp1 wp3)      (connected wp3 wp1)
        (connected wp2 wp4)      (connected wp4 wp2)
    )

    ;; 3. Dummy Goal (Required by syntax, but we won't use it)
    (:goal (and (at r wp_start))) 
)
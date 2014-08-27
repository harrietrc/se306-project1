
define assistant position 
(
  color "gray50"
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])
  size [2.5 2.5 5]

	localization_origin [0 0 0 0]
	velocity_bounds [-10 10 -10 10 -10 10 -180 180]

  # body bottom
  block 
  (
    points 4
    point[0] [ 0.1 0.25 ] 
    point[1] [ 0.9 0.25 ] 
    point[2] [ 0.9 0.75 ] 
    point[3] [ 0.1 0.75 ] 
    z [ 0.2 0.6 ]
	 	color "dark grey"
  )

  # body top
  block 
  (
    points 4
    point[0] [ 0.1 0.25 ] 
    point[1] [ 0.9 0.25 ] 
    point[2] [ 0.9 0.75 ] 
    point[3] [ 0.1 0.75 ] 
    z [ 0.6 0.7 ]
		color "thistle"
  )

   #right wheel 
  block 
  (
    points 4
    point[0] [ 1 0 ] 
    point[1] [ 1 0.25 ] 
    point[2] [ 0.0 0.25 ] 
    point[3] [ 0 0.0 ] 
    z [ 0.0 0.3 ]
		color "dark grey"
  )

   #right wheel 
  block 
  (
   points 4
   point[0] [ 1 0.75 ] 
   point[1] [ 1 1 ] 
   point[2] [ 0 1 ] 
   point[3] [ 0 0.75 ] 
   z [ 0.0 0.3 ]
  )

  # face front
  block
  (
	 points 4
	 point[0] [0.8 0.20]
	 point[1] [0.8 0.80]
	 point[2] [0.6 0.80]
	 point[3] [0.6 0.20]
	 z [0.7 0.95]
  )

  # right eye rear
  block
  (
	 points 4
	 point[0] [0.6 0.34]
	 point[1] [0.6 0.49]
	 point[2] [0.25 0.49]
	 point[3] [0.25 0.34]
	 z [0.82 0.93]
  )

  # neck
  block (
   points 4
	 point[3] [0.3 0.45]
	 point[2] [0.3 0.55]
	 point[1] [0.4 0.55]
	 point[0] [0.4 0.45]
	 z [0.7 0.9]
  )   

  # right eye rear
  block
  (
	 points 4
	 point[0] [0.6 0.51]
	 point[1] [0.6 0.66]
	 point[2] [0.25 0.66]
	 point[3] [0.25 0.51]
	 z [0.82 0.93]
  )

  # right arm
  block
  (
	 points 4
	 point[0] [1.1 0.18]
	 point[1] [1.1 0.25]
	 point[2] [0.35 0.25]
	 point[3] [0.35 0.18]
	 z [0.55 0.65]
  )

  # left arm
  block
  (
	 points 4
	 point[0] [0.35 0.82]
	 point[1] [0.35 0.75]
	 point[2] [1.1 0.75]
	 point[3] [1.1 0.82]
	 z [0.55 0.65]
  )

  # left hand right finger
  block
  (
	 points 4
	 point[0] [1.1 0.78]
	 point[1] [1.1 0.70]
	 point[2] [1.2 0.70]
	 point[3] [1.2 0.78]
	 z [0.5 0.65]
  )

  # left hand left finger
  block
  (
	 points 4
	 point[0] [1.1 0.87]
	 point[1] [1.1 0.79]
	 point[2] [1.2 0.79]
	 point[3] [1.2 0.87]
	 z [0.5 0.65]
  )

  # right hand right finger
  block
  (
	 points 4
	 point[0] [1.1 0.21]
	 point[1] [1.1 0.13]
	 point[2] [1.2 0.13]
	 point[3] [1.2 0.21]
	 z [0.5 0.65]
  )

  # right hand left finger
  block
  (
	 points 4
	 point[0] [1.1 0.30]
	 point[1] [1.1 0.22]
	 point[2] [1.2 0.22]
	 point[3] [1.2 0.30]
	 z [0.5 0.65]
  )

)

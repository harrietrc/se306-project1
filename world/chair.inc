define chair position
(
	# top
	block 
  (
    points 4
    point[0] [ 0 0 ] 
    point[1] [ 0.5 0 ] 
    point[2] [ 0.5 5 ] 
    point[3] [ 0 5 ] 
    z [ 0 1 ]

	)

	block
	(
    points 4
    point[0] [ 2 0 ] 
    point[1] [ 2.5 0 ] 
    point[2] [ 2.5 0.5 ] 
    point[3] [ 2 0.5 ] 
    z [ 0 0.3 ]

	)

	block
	(
    points 4
    point[0] [ 2 4.5 ] 
    point[1] [ 2.5 4.5 ] 
    point[2] [ 2.5 5 ] 
    point[3] [ 2 5 ] 
    z [ 0 0.3 ]

	)
	
	block
	(
    points 4
    point[0] [ 0 0 ] 
    point[1] [ 0 5 ] 
    point[2] [ 2.5 5 ] 
    point[3] [ 2.5 0 ] 
    z [ 0.3 0.4]

	)

	#fit shape to this size in meters
	size [4 4 5.5]
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])
)

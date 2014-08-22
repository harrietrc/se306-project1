define table position
(
	# top
	block
	(
		points 8
		point[0] [0 0]
		point[1] [0 4.8]  
		point[2] [-0.2 5.0]
		point[3] [-2.8 5.0]
		point[4] [-3.0 4.8]
		point[5] [-3.0 0]
    point[6] [-2.8 -0.2]
    point[7] [-0.2 -0.2]
		z [0.8 1]

	)
	# leg 1
  block 
  (
    points 4
    point[0] [ -0.4 0.4 ] 
    point[1] [ -0.8 0.4 ] 
    point[2] [ -0.8 0.8 ] 
    point[3] [ -0.4 0.8 ] 
    z [ 0 0.8 ]

  ) 

	# leg 2
  block 
  (
    points 4
    point[0] [ -0.4 4.6 ] 
    point[1] [ -0.8 4.6 ] 
    point[2] [ -0.8 4.2 ] 
    point[3] [ -0.4 4.2 ] 
    z [ 0 0.8 ]

  )  

	# leg 3
  block 
  (
    points 4
    point[0] [ -2.6 4.6 ] 
    point[1] [ -2.2 4.6 ] 
    point[2] [ -2.2 4.2 ] 
    point[3] [ -2.6 4.2 ] 
    z [ 0 0.8 ]

  )  

	# leg 4
  block 
  (
    points 4
    point[0] [ -2.2 0.4 ] 
    point[1] [ -2.6 0.4 ] 
    point[2] [ -2.6 0.8 ] 
    point[3] [ -2.2 0.8 ] 
    z [ 0 0.8 ]

  )  

	
	#fit shape to this size in meters
	size [6.5 10 4]
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])
)

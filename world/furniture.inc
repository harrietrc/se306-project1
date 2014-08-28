define sofa model
(
	# left arm rest
	block
	(
		points 4
		point[0] [1.7 0]
		point[1] [-4 0]  
		point[2] [-4 -1.5]
		point[3] [1.7 -1.5]
		z [0 0.7]
	)

	# back support
	block
	(
		points 4
		point[0] [1.7 0]
		point[1] [1.7 9]  
		point[2] [0 9]
		point[3] [0 0]
		z [0 1]
	)

	# right arm rest
	block
	(
		points 4
		point[0] [1.7 10.5]
		point[1] [-4 10.5]
		point[2] [-4 9]
		point[3] [1.7 9]
						
		z [0 0.7]
	)

	# cushion 1
	block
	(
		points 5
		point[0] [0 0]
		point[1] [-4.3 0] 
		point[2] [-4.5 0.2]
		point[3] [-4.5 3.0]
		point[4] [0 3.0]			
		z [0 0.4]
	)
	
	# cushion 2
	block
	(
		points 4
		point[0] [0 3.0]
		point[1] [-4.5 3.0]
		point[2] [-4.5 6.0]
		point[3] [0 6.0]			
		z [0 0.4]
	)

	# cushion3
	block
	(
		points 5
		point[0] [0 6.0]
		point[1] [-4.5 6.0] 
		point[2] [-4.5 8.7]
		point[3] [-4.3 9.0]
		point[4] [0 9.0]			
		z [0 0.4]
	)

	#fit shape to this size in meters
	size [6 15 7]
)

######## BED #########

define bed model
(
	# duvet
	block
	(
		points 6
		point[0] [0 0]
		point[1] [0 -4.9]  
		point[2] [-0.1 -5.0]
		point[3] [-3.9 -5.0]
		point[4] [-4.0 -4.9]
		point[5] [-4.0 0]
		z [0 0.7]
		color "steel blue"
	)

	# matrice
	block
	(
		points 6
		point[0] [-4.0 0]
		point[1] [0 0]  
		point[2] [0 1.9]
		point[3] [-0.1 2.0]
		point[4] [-3.9 2]
		point[5] [-4.0 1.9]
		z [0 0.7]
		color "white"
	)

	# pillow left
	block
	(
		points 8
		point[0] [-0.3 0]
		point[1] [-0.1 0.2]  
		point[2] [-0.1 1.6]
		point[3] [-0.3 1.8]
		point[4] [-1.8 1.8]
		point[5] [-2.0 1.6]
		point[6] [-2.0 0.2]
		point[7] [-1.8 0]
		z [0.7 0.9]
		color "snow"
	)
	
	# pillow right
	block
	(
		points 8
		point[0] [-2.2 0]
		point[1] [-3.7 0]  
		point[2] [-3.9 0.2]
		point[3] [-3.9 1.6]
		point[4] [-3.7 1.8]
		point[5] [-2.2 1.8]
		point[6] [-2.0 1.6]
		point[7] [-2.0 0.2]
		z [0.7 0.9]
	)
	
	#fit shape to this size in meters
	size [12 15 7]

)

######### CHAIR ##########

define chair model
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

)

######### COUNTER ##########

define counter model
(
	# stove
	block
	(
		points 4
		point[0] [0 0]#
		point[1] [0 16]  
		point[2] [-8 16]
		point[3] [-8 0]
		z [0 1]

	)
	# counter bottom
	block
	(
		points 4
		point[0] [0 0]#
		point[1] [-17 0]  
		point[2] [-17 -8]
		point[3] [0 -8]
		z [0 1]

	)
	# counter top
	block
	(
		points 4
		point[0] [0 16]#
		point[1] [0 24]  
		point[2] [-24 24]
		point[3] [-24 16]
		z [0 1]

	)

	#fit shape to this size in meters
	size [24 24 5.5]
)

######### TABLE ##########

define table model
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
	size [5 8 4]
)

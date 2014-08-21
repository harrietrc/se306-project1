define sofa position
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
		point[1] [-4.3 0]  #
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
		point[1] [-4.5 3.0]  #
		point[2] [-4.5 6.0]
		point[3] [0 6.0]			
		z [0 0.4]
	)

	# cushion3
	block
	(
		points 5
		point[0] [0 6.0]
		point[1] [-4.5 6.0]  #
		point[2] [-4.5 8.7]
		point[3] [-4.3 9.0]
		point[4] [0 9.0]			
		z [0 0.4]
	)

	#fit shape to this size in meters
	size [7 17 7]
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])
)

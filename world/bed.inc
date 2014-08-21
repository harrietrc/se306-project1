define bed position
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
		color "light yellow"
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
		z [0.7 1]
		color "light green"
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
		z [0.7 1]
	)
	
	#fit shape to this size in meters
	size [15 18 7]
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])
)

define door position
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
		z [0 1]

	)

	#fit shape to this size in meters
	size [5 12 10]
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])

)

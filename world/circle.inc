define circle position
(
	# stove
	block
	(
		points 14
		point[0] [-0.8 0]#
		point[1] [-1 2]
		point[2] [-1.5 2.5]  
		point[3] [-2 3]
		point[4] [-3 3]
		point[5] [-3.5 2.5]
		point[6] [-4 2]
		point[7] [-4.2 0]
		point[8] [-4 -2]
		point[9] [-3.5 -2.5]
		point[10] [-3 -3]
		point[11] [-2 -3]
		point[12] [-1.5 -2.5]
		point[13] [-1 -2]
		z [0 1]

	)

	#block
	#(
	#	points 4
	#	point[0] [-2.4 0.2]#
	#	point[1] [-2.8 0.2]
	#	point[2] [-2.8 -0.2]  
	#	point[3] [-2.4 -0.2]
	#	z [0 0.8]
#
	#)

	#fit shape to this size in meters
	size [14 8 1]
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])
)

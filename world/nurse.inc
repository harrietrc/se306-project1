define nurse position
(

	block
	(
		points 4
		point[0] [0 -0.25]
		point[1] [1 -0.25]  
		point[2] [1 2.25]
		point[3] [0 2.25]

		z [0.8 1]
	)
	#body
	block
	(
		points 4
		point[0] [0 -1]
		point[1] [0.8 -1]  
		point[2] [0.8 3]
		point[3] [0 3]

		z [0.4 0.75]
	)
	#arms
	block
	(
		points 4
		point[0] [0 3.3]
		point[1] [0.8 3.3]  
		point[2] [0.8 4.7]
		point[3] [0 4.7]

		z [0.5 0.75]
	)
	#arms
	block
	(
		points 4
		point[0] [0 -0.7]
		point[1] [0.8 -0.7]  
		point[2] [0.8 -2.1]
		point[3] [0 -2.1]

		z [0.5 0.75]
	)
	
	#legs
	block
	(

		points 4
		point[0] [0 1.8]
		point[1] [0.8 1.8]  
		point[2] [0.8 3]
		point[3] [0 3]

		z [0 0.4]
	)

	#legs
	block
	(

		points 4
		point[0] [0 0.2]
		point[1] [0.8 0.2]  
		point[2] [0.8 -1]
		point[3] [0 -1]

		z [0 0.4]
	)

	#CGImage(pose [ 0.200 0.000 1.000 90.000 ] name "r7" color "pink")

	size [1.5 2.5 7.5]
	drive "diff"
	mylaser(pose [ 0.050 0.000 0.000 0.000 ])
)

#define CGImage model
#(
	#bitmap "heart.png"
	#size [2.5 2.5 0.5]

#)

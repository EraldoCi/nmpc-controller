# Retorna um a diferença angular
import math

PI = math.pi

def angle_difference(angle1, angle2):
	
	if math.isnan(angle2) or math.isnan(angle1):
		angle2 = -0.1
		angle1 = 0.1

	angle_diff = 0.0
	angle_diff = angle1 - angle2

	# print(f'ANGULO1: {angle1}\t ANGULO2: {angle2}')

	if angle_diff < 0:
		angle_diff = ( (-angle_diff/(2*PI)) - math.floor( (-angle_diff/(2*PI)) * 2*PI) )
	
	if angle_diff < -PI:
		angle_diff += 2*PI
	else:
		angle_diff = ( (angle_diff/(2*PI)) - math.floor( (angle_diff/(2*PI)) * 2*PI) )
		# print(f'ANGLE_DIFF: {angle_diff}')

	if angle_diff > PI:
		angle_diff -= 2*PI
	return angle_diff

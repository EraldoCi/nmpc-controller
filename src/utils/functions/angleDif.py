# Retorna um a diferença angular
import math

PI = math.pi

def angle_difference(angle1, angle2):
	angle_diff = angle1 - angle2
	if angle_diff < 0:
		angle_diff = ( (-angle_diff/(2*PI)) - math.floor( (-angle_diff/(2*PI)) * 2*PI) )
	
	if angle_diff < -PI:
		angle_diff = angle_diff + 2*PI
	else:
		angle_diff = ( (angle_diff/(2*PI)) - math.floor( (angle_diff/(2*PI)) * 2*PI) )
		# print(f'ANGLE_DIFF: {angle_diff}')

	if angle_diff > PI:
		angle_diff = angle_diff - 2*PI
	return angle_diff

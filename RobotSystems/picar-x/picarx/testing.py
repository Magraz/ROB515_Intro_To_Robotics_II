import math
l = 0.1
w = 0.12
dir_current_angle = math.radians(-2)
inner_angle = math.atan((2*l*math.sin(dir_current_angle))/(2*l*math.cos(dir_current_angle) - w*math.sin(dir_current_angle)))
outer_angle = math.atan((2*l*math.sin(dir_current_angle))/(2*l*math.cos(dir_current_angle) + w*math.sin(dir_current_angle)))

if (inner_angle == 0) or (outer_angle == 0):
    scaling = 1
else:
    abs_outer_angle = abs(outer_angle)
    abs_inner_angle = abs(inner_angle)

    if abs(outer_angle) > abs(inner_angle):
        scaling = abs_inner_angle/abs_outer_angle
    else:
        scaling = abs_outer_angle/abs_inner_angle

print(f"Scaling {scaling}")
"""Draft Code for lifting in bay area and navigating back"""


# 1. Side TOF sensor detects a block and junction is in loading area sets 
# Orange 1, Orange 2, Purple 1, Purple 2. 
# 2. Raises the ForkLift to the corresponding height
# 3. Turn at the junction to the right direction.
# 4. Go forwards for a finite time, then stop. 
# 5. Read colour using the colour sensor. Record Colour.
# Insert the fork lift by moving forwards a finite amount of time (measure speed) 
# 6. Reverse out away from the bay
# 7. Do a 180 degree pure rotation
# 8. Reverse the trajectory from baseline to the colour, and add 
# the route from baseline to required colour.
# 9. Go to corresponding unloading area
# UNLOADING MECHANISM 
# 10. Lower forklift
# 11. Reverse out of the forklift 
# 12. 180 degree turn, and leave the unloading area

# takes in start junction, 
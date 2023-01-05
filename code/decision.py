import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # Find all the rock angles within 10 degrees of the left of the rover   
    Rover.left_samp_angles = np.where(Rover.rock_angles * 180/np.pi > -10)[0]
    
    # If the rover has collected all 5 samples, start going back to the starting position 
    if Rover.samples_collected == 5:
        print("Going Back to Start Point")
        # Calculate the distance from the rover's current position to the starting position
        dist_start = np.sqrt((Rover.pos[0] - Rover.start_pos[0])**2 + (Rover.pos[1] - Rover.start_pos[1])**2)
        # Make sure we are heading in right general direction
        # TODO
        # If we are in 10 meters steer to starting point
   
         # If we are within 10 meters of the starting point, stop the rover
        if dist_start < 10.0 :
            print("10m away from start")
            Rover.mode = 'stop'
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            return Rover
    
    # If the rover is currently picking up a rock, increment the number of samples collected
    if Rover.picking_up == 1:
        print("Right Now in state of picking up rock")
        # Check if the rover has finished picking up the rock
        if Rover.picking_up == False:
            # Update the number of samples collected
            Rover.picking_up = True
            Rover.samples_collected += 1
        return Rover
    else:
         # Set the picking_up flag to False
         Rover.picking_up = False
    
    # If the rover is within 3 meters of a rock sample, stop the rover
    if Rover.near_sample == 1:
        print("Rover is currently stopping at sample")
        Rover.mode = 'stop'
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        # If the rover has stopped moving, set the flag to send a pickup command
        if Rover.vel == 0: 
            Rover.send_pickup = True
        return Rover  
        
    # If the rover is stuck, try to get unstuck    
    if Rover.mode == 'stuck':
        if Rover.stuck_mode == 'forward':
            print("Stuck in the position to move forward")
            # Try moving forward to get unstuck
            Rover.throttle = 1
            Rover.stuck_counter = Rover.stuck_counter + 1
            print(Rover.stuck_counter)
            # If trying to move forward for more than 30 iterations hasn't worked, try changing the rover's angle
            if Rover.stuck_counter > 30:
                Rover.stuck_mode = 'yaw'
                Rover.stuck_counter = 0
        elif Rover.stuck_mode == 'yaw':
            print("Stuck in the position to change angle")
            # Try changing the rover's angle to get unstuck
            Rover.throttle = 0
            Rover.steer = -15
            Rover.stuck_counter += 1
            # If trying to change the rover's angle for more than 20 iterations hasn't worked, try moving forward again
            if Rover.stuck_counter > 20:
                Rover.stuck_mode = 'forward'
                Rover.stuck_counter = 0
        
        # If the rover is able to move again, switch back to the 'forward' mode
        if Rover.vel > 0.6:
            Rover.mode = 'forward'

        return Rover
        
    # If the rover is in the 'forward' mode, check if it is stuck    
    if Rover.mode == 'forward':
    	# If the rover's velocity is less than 0.5, increment the stuck counter
        if Rover.vel < 0.5:
            Rover.stuck_counter += 1
    else:
    	# If the rover is not in the 'forward' mode, reset the stuck counter
        Rover.stuck_counter = 0
    
     # If the stuck counter is greater than 75, switch the rover to the 'stuck' mode    
    if Rover.stuck_counter > 75:
        Rover.mode = 'stuck'
        Rover.stuck_mode = 'forward'
        Rover.stuck_counter = 0
        
    # If we are driving in a circle try to brake out by turning other way
    if Rover.mode == 'looping':
        print("Stopping the looping condition of rover")
        Rover.throttle = 0
        Rover.steer = -15
        Rover.brake = 0
        Rover.left_count += 1
        if Rover.left_count > 50: 
            Rover.mode = 'forward'
            Rover.left_count = 0
        return Rover

    if Rover.steer > 5:
        Rover.left_count += 1
    else:
        Rover.left_count = 0

    if Rover.left_count > 250:
        Rover.mode = 'looping'
        Rover.left_count = 0
        return Rover
        
    # Get the left half of nav angles to wall crawl
    Rover.nav_angles = np.sort(Rover.nav_angles)[-int(len(Rover.nav_angles)/2):]

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
        
            # Check for samples to collect
            if len(Rover.left_samp_angles) > 1:
                print("Approaching sample")

                if Rover.vel < 0.75: #when finding sample approach slower
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                    
                Rover.brake = 0

                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)    
        
            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
    		
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                 # Check for samples to collect
                if len(Rover.left_samp_angles) > 1:
                    print("Approaching sample")
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)            
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0 
           
                # Now we're stopped and we have vision data to see if there's a path forward
                elif len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
 
    
    return Rover


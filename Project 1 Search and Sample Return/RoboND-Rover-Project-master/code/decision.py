import numpy as np
import random


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # Depending on available clear space, set a max velocity
    if len(Rover.nav_angles) > 3500 and Rover.get_rock == False:
        Rover.max_vel = 3
    elif len(Rover.nav_angles) > 2500 and Rover.get_rock == False:
        Rover.max_vel = 2.5
    elif len(Rover.nav_angles) > 1500 and Rover.get_rock == False:
        Rover.max_vel = 2
    else:
        Rover.max_vel = 1
        
    narrow_nav = (Rover.nav_angles[:] < 5) & (Rover.nav_angles[:] > -5)
    num_narrow_nav = len(narrow_nav)
    if num_narrow_nav < 300 and Rover.get_rock != True: # Previously 100
        Rover.nav_angles = Rover.nav_angles - 10*(np.pi/180)
    
    Rover.yaw_average = np.insert(Rover.yaw_average[0:-1], 0, Rover.yaw)
    Rover.yaw_var = np.var(Rover.yaw_average)
    
    # Determine if the rover is stuck.        
    if (Rover.total_time - Rover.last_time) > 5 and Rover.mode == 'forward': # Check every x seconds
        # Check if the machine position has changed by more than mag(x,y) over the given number of seconds
        if np.linalg.norm(np.array(Rover.last_pos) - np.array(Rover.pos)) <= 0.075:
            Rover.mode = 'stuck'
        if np.linalg.norm(np.array(Rover.last_pos) - np.array(Rover.pos)) <= 0.35 and Rover.yaw_var <= 0.03:
            Rover.mode = 'stuck'
        # Update counters
        Rover.last_time = Rover.total_time
        Rover.last_pos = Rover.pos
    
    if Rover.near_sample == True and np.min(Rover.nav_dists) > 50 and Rover.get_rock == True:
        Rover.max_vel = 0.5
    elif Rover.near_sample == True and np.min(Rover.nav_dists) < 50:
        Rover.send_pickup = True
        Rover.get_rock == False
        
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        
        
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            Rover.brake = 0
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -30, 30)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward and Rover.get_rock == False:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
            else:
                # Just make sure we keep moving. Likely we saw a rock and don't have many visible pixels.
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                else: # Else coast
                    Rover.throttle = 0
                    Rover.brake = 0
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -30, 30)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
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
                    #Rover.steer = np.int(Rover.steer - 0.4*(Rover.steer-(np.max(-15,np.min(Rover.nav_angles*180/np.pi)))))
                    Rover.mode = 'forward'
        # If we're stuck randomly spin
        elif Rover.mode == 'stuck':
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif Rover.vel <= 0.2:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                if Rover.rand_dir == 1:
                    Rover.steer = -15 # Could be more clever here about which way to turn
                else:
                    Rover.steer = 15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if Rover.count >= 150:
                    Rover.throttle = Rover.throttle_set
                    Rover.mode = 'forward'
                    Rover.rand_dir = random.choice([1, 2]) # Pick a new random direction
                    Rover.count = 0 # Reset the stuck counter
                    Rover.yaw_average = np.array([0, 1, 2, 3, 4])
                    Rover.last_pos = [0, 0]
            Rover.count = Rover.count + 1
        #else:
        #    Rover.mode = 'forward'
            
            
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    elif Rover.near_sample:
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = Rover.brake_set
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.get_rock = False
    
    return Rover
    
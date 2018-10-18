import numpy as np
import cv2
from itertools import repeat

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_thresh_above(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_thresh_below(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_thresh_middle(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = ((img[:,:,0] > rgb_thresh[0]-40) & (img[:,:,0] < rgb_thresh[0]+40)) \
                & ((img[:,:,1] > rgb_thresh[1]-40) & (img[:,:,1] < rgb_thresh[1]+40)) \
                & ((img[:,:,2] > rgb_thresh[2]-40) & (img[:,:,2] < rgb_thresh[2]+40))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_find_rock(img, rgb_thresh=(110, 110, 50)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    # Return the results
    return warped, mask

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # Navigable terrain
    threshed = color_thresh_above(warped, rgb_thresh=(150, 150, 150))
    # Non navigable terrain
    obs_map = np.absolute(np.float32(threshed) - 1) * mask
    # Location of rocks
    thresh_rock = color_find_rock(warped, rgb_thresh=(110, 110, 50))
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obs_map*255
    Rover.vision_image[:,:,1] = thresh_rock*255
    Rover.vision_image[:,:,2] = threshed*255
    
    # 5) Convert map image pixel values to rover-centric coords
    
    xpix, ypix = rover_coords(threshed)
    obsxpix, obsypix = rover_coords(obs_map)
    rocksxpix, rocksypix = rover_coords(thresh_rock)
    
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2* dst_size
    
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    rocks_x_world, rocks_y_world = pix_to_world(rocksxpix, rocksypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # Count up pixels of navigable vs obstacle pixels. 
    # Navigable terrain - Blue
    # If more than three degrees of role or pitch, don't update the counts
    #if Rover.pitch < np.radians(3) and Rover.roll < np.radians(3):
    if (Rover.pitch < 3 or Rover.pitch > 357) and (Rover.roll < 3 or Rover.roll > 357):
        Rover.worldmapcnt[y_world, x_world, 2] = Rover.worldmapcnt[y_world, x_world, 2]+2 #blue channel, 
        # Obstacles - Red
        Rover.worldmapcnt[obs_y_world, obs_x_world, 0] = Rover.worldmapcnt[obs_y_world, obs_x_world, 0]+1
        # Rocks - Green
        Rover.worldmapcnt[rocks_y_world, rocks_x_world, 1] = Rover.worldmapcnt[rocks_y_world, rocks_x_world, 0]+1
    
     # Where blue is greater than red, navigable
    nav_pix = Rover.worldmapcnt[:,:,2] > Rover.worldmapcnt[:,:,0]+1
    # Where obstacle pic count is greater than navigable pic count
    obs_pix = Rover.worldmapcnt[:,:,0] > Rover.worldmapcnt[:,:,2]+1 
    # Where rock pixel count is greater than 5, consider it a rock
    rocks_pix = Rover.worldmapcnt[:,:,1] > 5
    
    # When previous condition true, set red to zero. 
    Rover.worldmap[nav_pix, 0] = 0
    Rover.worldmap[nav_pix, 2] = 255
    
    Rover.worldmap[obs_pix, 0] = 255
    Rover.worldmap[obs_pix, 2] = 0
    # Turn rocks white for better visibility
    Rover.worldmap[rocks_pix, :] = 255
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    # Using rover central pixel coords, find their radial distance and angular position
    dist, angles = to_polar_coords(xpix, ypix)
    
    # If we find a rock, send that info to the robot and use 
    # Send available distances and angles to rover
    # Only go towards the rock if we have at least x pixels
    if sum(thresh_rock).any() > 3 and 1==1: # Force this not to run for now
        dist_rock, angles_rock = to_polar_coords(rocksxpix, rocksypix)
        Rover.nav_dists = dist_rock
        Rover.nav_angles = angles_rock
        Rover.get_rock = True
    else:
        Rover.get_rock = False
        Rover.nav_dists = dist
        Rover.nav_angles = angles
    
    return Rover
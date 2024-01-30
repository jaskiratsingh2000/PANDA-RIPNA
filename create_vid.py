import os
from PIL import Image
import matplotlib.pyplot as plt
import networkx as nx
#import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation, PillowWriter, ArtistAnimation

def create_animation_from_frames(folder_path, output_path, frame_rate):
    # Get all files in the folder and sort them (assuming filenames have a proper order)
    files = sorted([os.path.join(folder_path, f) for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))])
    
    # Read the images
    images = [Image.open(f) for f in files]
    
    # Create a figure and axes to display the animation
    fig, ax = plt.subplots()
    
    # Remove axis
    ax.axis('off')
    
    # Create a list to store the plots
    ims = []
    for im in images:
        # Append the images to the list of plots
        ims.append([plt.imshow(im, animated=True)])
    
    # Create an animation
    ani = ArtistAnimation(fig, ims, interval=1000/frame_rate, blit=True, repeat_delay=1000)
    
    # Save the animation
    ani.save(output_path, writer='imagemagick', fps=frame_rate)

# Example usage:
folder_path = 'plots'
output_path = 'animation.gif'
frame_rate = 15  # frames per second
create_animation_from_frames(folder_path, output_path, frame_rate)

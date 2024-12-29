import os
import pyvista as pv
import argparse

def load_and_plot_all_obj_files(obj_folder:str, tilename:str, resolution:int):
    """
    Renders the 1:10 scale Road Modules .obj files parts contained in a folder to .png image while maintaining original proportions using PyVista.
    Tiles are rendered by plotting all the parts together and colouring them individually.
    The tile image is then created and saved by screenshooting the resulting mesh.
    
    Parameters:
        directory (str): The path to the directory containing .obj files with.
        tilename (str): The path where the final rendered tile will be saved.
        resolution (int): The desired resolution for the rendered tiles. 
    """
    # Configure PyVista global settings
    pv.global_theme.full_screen = True
    pv.set_plot_theme('dark')

    # Initialize the PyVista plotter with off-screen rendering
    plotter = pv.Plotter(off_screen=True, window_size=(resolution, resolution))

    # Retrieve all .obj files in the directory that form the tile
    tile_part = [f for f in os.listdir(obj_folder) if f.lower().endswith('.obj')]

    if not tile_part:
        print(f"No .obj files found in the obj_folder: {obj_folder}")
        return

    # Load each .obj file and plot with either black or white determined by the filename
    for obj_file in tile_part:
        mesh = pv.read(os.path.join(obj_folder, obj_file))
        
        # Determine color based on the filename
        if 'black' in obj_file.lower():
            color = 'black'
        elif 'white' in obj_file.lower():
            color = 'white'
        else:
            raise ValueError(f"No color indicated in the tile_part for {obj_file}. Please ensure the tile part includes 'black' or 'white'.")


        # Add the mesh to the plotter with the specified color
        plotter.add_mesh(mesh, color=color, opacity=1)

    # Set the camera view, zoom, and save the rendered image
    plotter.view_xy()
    plotter.zoom_camera("tight") # Method that fits the tile to the specified resolution
    plotter.show()
    plotter.screenshot(filename=tilename, transparent_background=True) # Transparent background set to True to avoid white noise in the tile image.


def process_folders(obj_directory: str= "obj_folders", resolution: int = 900, output_directory: str = None):
    """
    Given the directory with the Road Tiles (.obj) files and a resolution, renders the 1:10 scale Road Modules .obj folders contained and saves them in output_directory.

    Parameters:
        obj_directory (str): The path to the parent directory containing subfolders with .obj files.
        resolution (int):       Desired resolution for the rendered circuit tiles.
        output_directory (str): The path to the directory where output images will be saved. 
                                If None, the directory is named and created based on the resolution.
    """
    # If no output directory is provided, generate one with a standard name based on the resolution
    if output_directory is None:
        output_directory = "rendered_tiles_" + str(resolution)
    
    # Ensure the output directory exists
    os.makedirs(output_directory, exist_ok=True)

    # Get all subdirectories in the parent directory
    obj_folders = [f for f in os.listdir(obj_directory) if os.path.isdir(os.path.join(obj_directory, f))]

    for folder in obj_folders:
        folder_path = os.path.join(obj_directory, folder)
        output_image_path = os.path.join(output_directory, f"{folder}.png")
        
        # Load and render the .obj files in the folder
        load_and_plot_all_obj_files(folder_path, output_image_path, resolution)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Render 1:10 scale Road Modules .obj files to .png images.")
    parser.add_argument('--obj_directory', type=str, default="obj_folders", help="The path to the parent directory containing subfolders with .obj files. Defaults to 'obj_folders'.")
    parser.add_argument('--resolution', type=int, default=900, help="Desired resolution for the rendered circuit tiles. Defaults to 900.")
    parser.add_argument('--output_directory', type=str, default=None, help="The path to the directory where output images will be saved. If not provided, a directory named based on the resolution will be created.")

    args = parser.parse_args()
    process_folders(args.obj_directory, args.resolution, args.output_directory)


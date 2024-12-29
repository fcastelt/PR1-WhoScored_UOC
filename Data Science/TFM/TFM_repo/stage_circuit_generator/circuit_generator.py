import cv2
import numpy as np
from PIL import Image
import argparse
import os

# Allow for handling of very large images, otherwise it raises an error. 
Image.MAX_IMAGE_PIXELS = None

def rotate_image(image, angle):
    """
    Rotate the image by a specified angle.
    """
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    rot_img = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return rot_img

def concat_vh(array_of_images):
    """
    Concatenates an array of images both vertically and horizontally.
    """
    concatenated_image = cv2.vconcat([cv2.hconcat(list_h) for list_h in array_of_images])
    num_rows = len(array_of_images)
    num_cols = max(len(row) for row in array_of_images) if num_rows > 0 else 0
    return concatenated_image, num_rows, num_cols

def flip_image(image, flip_code):
    """
    Flip an image along the specified axis.

    Parameters:
    image (numpy.ndarray): The input image to flip.
    flip_code (int): The code specifying the flip direction:
                     0 means flipping around the x-axis (vertical flip),
                     1 means flipping around the y-axis (horizontal flip),
                    -1 means flipping around both axes.

    Returns:
    numpy.ndarray: The flipped image.
    """
    # Flip the image based on the specified flip code
    flipped_image = cv2.flip(image, flip_code)
    return flipped_image

def load_images_from_dir(rendered_tiles_dir):
    """
    Loads all required images from the specified directory.
    """
    images = {
        "two_lanes_separated_vertical": cv2.imread(f'{rendered_tiles_dir}/2Lanes_separated.png'),
        "two_lanes_guideline1_vertical": cv2.imread(f'{rendered_tiles_dir}/2Lanes_guideline1.png'),
        "two_lanes_guideline2_vertical": cv2.imread(f'{rendered_tiles_dir}/2Lanes_guideline2.png'),
        "two_lanes_to_1_lane_vertical": cv2.imread(f'{rendered_tiles_dir}/2to1Lane.png'),
        "x_junction": cv2.imread(f'{rendered_tiles_dir}/2Lanes_Xjunction.png'),
        "t_junction_leftwards": cv2.imread(f'{rendered_tiles_dir}/2Lanes_Tjunction.png'),
        "one_lane_to_LtoR_vertical": cv2.imread(f'{rendered_tiles_dir}/1Lane_LtoR.png'),
        "one_lane_to_RToL_vertical": cv2.imread(f'{rendered_tiles_dir}/1Lane_RtoL.png'),
        "one_lane_vertical": cv2.imread(f'{rendered_tiles_dir}/1Lane.png'),
        "one_to_two_lanes_vertical": cv2.imread(f'{rendered_tiles_dir}/1to2Lanes.png'),
        "soft_curve_1_top_right": cv2.imread(f'{rendered_tiles_dir}/2SoftCurve_30_1.png'),
        "soft_curve_2_top_right": cv2.imread(f'{rendered_tiles_dir}/2SoftCurve_30_2.png'),
        "soft_curve_3_top_right": cv2.imread(f'{rendered_tiles_dir}/2SoftCurve_30_3.png'),
        "soft_curve_4_top_right": cv2.imread(f'{rendered_tiles_dir}/2SoftCurve_30_4.png'),
        "curve_90_top_right": cv2.imread(f'{rendered_tiles_dir}/2Lanes90degCurve_guideline.png')
    }
    # Rotate images for horizontal alignment
    images.update({
        "two_lanes_separated_horizontal": rotate_image(images["two_lanes_separated_vertical"], 90),
        "two_lanes_guideline1_horizontal": rotate_image(images["two_lanes_guideline1_vertical"], 90),
        "two_lanes_guideline2_horizontal": rotate_image(images["two_lanes_guideline2_vertical"], 90),
        "two_lanes_to_1_lane_horizontal": rotate_image(images["two_lanes_to_1_lane_vertical"], -90),
        "t_junction_rightwards": rotate_image(images["t_junction_leftwards"], 180),
        "t_junction_upwards": rotate_image(images["t_junction_leftwards"], -90),
        "t_junction_downwards": rotate_image(images["t_junction_leftwards"], 90),
        "one_lane_to_LtoR_horizontal": rotate_image(images["one_lane_to_LtoR_vertical"], 90),
        "one_lane_to_RToL_horizontal": rotate_image(images["one_lane_to_RToL_vertical"], 90),
        "one_lane_horizontal": rotate_image(images["one_lane_vertical"], -90),
        "one_to_two_lanes_horizontal": rotate_image(images["one_to_two_lanes_vertical"], -90),
        "soft_curve_1_top_left": rotate_image(images["soft_curve_1_top_right"], 90),
        "soft_curve_2_top_left": rotate_image(images["soft_curve_2_top_right"], 90),
        "soft_curve_3_top_left": rotate_image(images["soft_curve_3_top_right"], 90),
        "soft_curve_4_top_left": rotate_image(images["soft_curve_4_top_right"], 90),
        "soft_curve_1_bottom_left": rotate_image(images["soft_curve_1_top_right"], 180),
        "soft_curve_2_bottom_left": rotate_image(images["soft_curve_2_top_right"], 180),
        "soft_curve_3_bottom_left": rotate_image(images["soft_curve_3_top_right"], 180),
        "soft_curve_4_bottom_left": rotate_image(images["soft_curve_4_top_right"], 180),
        "soft_curve_1_bottom_right": rotate_image(images["soft_curve_1_top_right"], 270),
        "soft_curve_2_bottom_right": rotate_image(images["soft_curve_2_top_right"], 270),
        "soft_curve_3_bottom_right": rotate_image(images["soft_curve_3_top_right"], 270),
        "soft_curve_4_bottom_right": rotate_image(images["soft_curve_4_top_right"], 270),
        "curve_90_top_left": rotate_image(images["curve_90_top_right"], 90),
        "curve_90_bottom_left": rotate_image(images["curve_90_top_right"], 180),
        "curve_90_bottom_right": rotate_image(images["curve_90_top_right"], 270),
        "one_to_two_lanes_vertical_flipped": flip_image(images["one_to_two_lanes_vertical"], 1),

    })
    # Create a black image to use as padding or placeholder
    height, width = images["two_lanes_separated_vertical"].shape[:2]
    images["black"] = np.zeros((height, width, 3), dtype=np.uint8)
    return images

def load_circuit_from_file(file_path, images_dict):
    """
    Loads circuit configurations from a text file, handling multiple rows for each circuit.
    """
    circuit_rows = []
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            # Parse the tile names for a row and add them to the current circuit
            if line.startswith("row:"):
                tile_names = line.split(":")[1].strip().split(",")
                row_images = [images_dict[tile.strip()] for tile in tile_names]
                circuit_rows.append(row_images)
            elif line.startswith("tiles:"):
                # Parse the tile names and apply repetition logic for horizontal tiling
                tile_names = line.split(":")[1].strip().split(", ")
                current_tiles = [images_dict[tile] for tile in tile_names]
            elif line.startswith("repeat:"):
                # For rows containing repeated tiles
                current_repeat = int(line.split(":")[1].strip())
                repeated_row = current_tiles * current_repeat
                circuit_rows.append(repeated_row)
    return circuit_rows

def main(rendered_tiles_dir, circuit_txt_dir=None, circuit_file=None):
    # Load the images from the directory into a dictionary
    images = load_images_from_dir(rendered_tiles_dir)
    
    # If a specific circuit file is provided, handle it differently
    if circuit_file:
        circuit_txt_files = [os.path.basename(circuit_file)]  # Extract just the filename
        circuit_txt_dir = os.path.dirname(circuit_file)  # Extract the directory
    else:
        # Get all the .txt files in the circuit configuration directory
        circuit_txt_files = [f for f in os.listdir(circuit_txt_dir) if f.endswith(".txt")]
    
    for circuit_txt_file in circuit_txt_files:
        # Generate the full path to the circuit configuration file
        circuit_txt_path = os.path.join(circuit_txt_dir, circuit_txt_file)

        # Load the circuit configuration from the text file
        circuit = load_circuit_from_file(circuit_txt_path, images)

        # Concatenate the circuit tiles into a single image
        circuit_tile, num_rows, num_cols = concat_vh(circuit)

        # Calculate equivalent size in meters
        width_in_meters = str(num_cols * 0.9).replace(".", "_")
        height_in_meters = str(num_rows * 0.9).replace(".", "_")

        # Use the filename (without extension) as the circuit name
        circuit_name = os.path.splitext(circuit_txt_file)[0]

        # Create an output path based on the circuit name and dimensions
        output_path = os.path.join(os.getcwd(), 'circuits_png')
        os.makedirs(output_path, exist_ok=True)  # Ensure the output directory exists

        # Set the output file path
        output_file = os.path.join(output_path, f"{circuit_name}_w{width_in_meters}m_x_h{height_in_meters}m.png")

        # Save the constructed circuit image
        cv2.imwrite(output_file, circuit_tile)
        print(f"Output saved to {output_file}")

        # Optionally, check the saved image's dimensions using PIL
        with Image.open(output_file) as img:
            width, height = img.size
            print(f"The image size is {width}x{height} pixels.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate circuits by concatenating tiles from a text file.")
    parser.add_argument("--rendered_tiles_dir", type=str, default=os.path.join(os.getcwd(), 'rendered_tiles_900'), help="Directory containing the rendered tiles")
    parser.add_argument("--circuit_txt_dir", type=str, default=os.path.join(os.getcwd(), 'circuit_conf_txt'), help="Path to the circuit text file directory")
    parser.add_argument("--circuit_file", type=str, help="If specified, only renders the provided circuit configuration file")
    args = parser.parse_args()

    # Check if the tiles directory exists
    if not os.path.exists(args.rendered_tiles_dir):
        raise FileNotFoundError(f"Directory {args.rendered_tiles_dir} does not exist.")
    
    # If a specific circuit file is provided, ensure it exists
    if args.circuit_file:
        circuit_path = os.path.join(args.circuit_txt_dir, args.circuit_file)
        
        if not os.path.exists(circuit_path):
            raise FileNotFoundError(f"Circuit file {circuit_path} does not exist.")
        
        # Call the main function with the specific circuit file
        main(args.rendered_tiles_dir, circuit_file=circuit_path)
    else:
        # If no specific circuit file is provided, process all files in the circuit directory
        if not os.path.exists(args.circuit_txt_dir):
            raise FileNotFoundError(f"Directory {args.circuit_txt_dir} does not exist.")
        
        # Call the main function for all circuit files
        main(args.rendered_tiles_dir, args.circuit_txt_dir)

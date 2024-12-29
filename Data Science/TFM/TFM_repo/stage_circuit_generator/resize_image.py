from PIL import Image, ImageFilter, ImageEnhance
Image.MAX_IMAGE_PIXELS = None 
def resize_image_by_width(image_path, desired_width):
    # Open the image file
    img = Image.open(image_path)
    
    # Get the original dimensions
    original_width, original_height = img.size
    print(img.size)
    
    # Calculate the new height to maintain the aspect ratio
    aspect_ratio = original_height / original_width
    print(aspect_ratio)
    new_height = int((desired_width * aspect_ratio))
    print(desired_width*aspect_ratio)
    # Resize the image
    resized_img = img.resize((desired_width, new_height), Image.ANTIALIAS)
    
    # Apply smoothing filters to reduce imperfections and smooth curves
    smoothed_img = resized_img.filter(ImageFilter.SMOOTH_MORE)
    
    # Optionally, enhance the contrast to recover white areas
    enhancer = ImageEnhance.Contrast(smoothed_img)
    enhanced_img = enhancer.enhance(2)  # Adjust the factor for more/less contrast

    # Optionally, enhance brightness if needed
    brightness_enhancer = ImageEnhance.Brightness(enhanced_img)
    final_img = brightness_enhancer.enhance(2)  # Adjust the factor to restore brightness

    # Save the final processed image
    output_path = f'circuits_png/circuit_rendered_tiles_900_{desired_width}x{new_height}_resized.png'  # Replace with the desired output path

    final_img.save(output_path)
    print(f"Resized and smoothed image saved to: {output_path}")
# Example usage
input_image_path = 'circuits_png/circuit_rendered_tiles_900_6300x4500.png' # Replace with your image file path
desired_width = 3150  # Replace with your desired width in pixels

resize_image_by_width(input_image_path, desired_width)

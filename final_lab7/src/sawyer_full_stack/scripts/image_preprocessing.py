import cv2
import numpy as np
import xml.etree.ElementTree as ET
import svgpathtools
from scipy.spatial.distance import cdist

def image_to_svg(input_path, output_path, threshold=127, min_contour_area=100):
    """
    Convert an image to SVG by tracing contours
    
    Parameters:
    - input_path: Path to input image
    - output_path: Path to save output SVG
    - threshold: Grayscale threshold for binarization
    - min_contour_area: Minimum contour area to include in SVG
    """
    # Read the image
    img = cv2.imread(input_path)
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Binarize the image
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours
    contours, _ = cv2.findContours(
        binary, 
        cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE
    )
    
    # Prepare SVG
    height, width = img.shape[:2]
    svg_root = ET.Element('svg', {
        'xmlns': 'http://www.w3.org/2000/svg',
        'width': str(width),
        'height': str(height),
        'viewBox': f'0 0 {width} {height}'
    })
    
    # Trace contours
    for contour in contours:
        # Skip small contours
        if cv2.contourArea(contour) < min_contour_area:
            continue
        
        # Convert contour to path string
        path_data = []
        for i, point in enumerate(contour):
            x, y = point[0]
            command = 'M' if i == 0 else 'L'
            path_data.append(f'{command} {x} {y}')
        path_data.append('Z')  # Close the path
        
        # Create path element
        ET.SubElement(svg_root, 'path', {
            'd': ' '.join(path_data),
            'fill': 'black',
            'stroke': 'none'
        })
    
    # Write SVG
    tree = ET.ElementTree(svg_root)
    tree.write(output_path, encoding='unicode', method='xml')
    
    print(f"SVG saved to {output_path}")

def load_svg(file_path):
    paths, _ = svgpathtools.svg2paths(file_path)
    return paths


def simplify_path(path, num_points=500):
    """Approximate SVG path with a high-resolution polyline."""
    points = []
    for i in np.linspace(0, 1, num_points):
        points.append(path.point(i))
    points = np.array([[p.real, p.imag] for p in points])
    return points


def find_outline(paths):
    """Identify the largest path as the main outline."""
    if not paths:
        raise ValueError("No paths found in the SVG file. Please check the file content.")
    
    largest_path = max(paths, key=lambda p: np.linalg.norm(simplify_path(p).ptp(axis=0)))
    outline = simplify_path(largest_path, num_points=1000)
    inner_paths = [p for p in paths if p != largest_path]
    return outline, inner_paths


def connect_all_paths(outline, inner_paths, num_points=500):
    """Connect all paths into a single continuous line with proper ordering and direction."""
    point_clusters = [simplify_path(p, num_points) for p in inner_paths]
    all_points = [outline]

    for cluster in point_clusters:
        # Get the current endpoint of the continuous line
        current_points = np.concatenate(all_points)
        last_point = current_points[-1]

        # Find the closest point in the new cluster to the current endpoint
        dist_to_cluster = cdist([last_point], cluster)
        closest_idx = np.argmin(dist_to_cluster)

        # Check if the closest point is at the start or end of the cluster
        if closest_idx == 0:
            # Cluster starts at the closest point, keep as is
            all_points.append(cluster)
        elif closest_idx == len(cluster) - 1:
            # Cluster ends at the closest point, reverse its order
            all_points.append(cluster[::-1])
        else:
            # Handle clusters that need splitting (rare in practice)
            print("Warning: Path splitting detected, handling edge case.")
            all_points.append(cluster)

    return np.concatenate(all_points)


def save_as_one_continuous_svg(points, output_path="continuous_line.svg"):
    """Save all points as a single continuous path in an SVG."""
    svg_content = '<svg xmlns="http://www.w3.org/2000/svg" version="1.1" fill="none" stroke="black" stroke-width="0.5">'
    svg_content += '<path d="M '
    for x, y in points:
        svg_content += f'{x},{y} '
    svg_content += '"/></svg>'

    with open(output_path, 'w') as file:
        file.write(svg_content)

def process(input_image):
    try:
       
        # Convert image
        output_name = input_image.split(".", 1)
        
        if output_name[1] != 'svg':
            output_svg = f'{output_name[0]}.svg'
                
            image_to_svg(
                input_image, 
                output_svg, 
                threshold=127,  # Adjust for different images
                min_contour_area=50  # Adjust to filter out noise
            )

            input_file = output_svg

        else:
            input_file = input_image
    
        output_file = f'/home/cc/ee106a/fa24/class/ee106a-aaw/ros_workspaces/final_lab7/{output_name[0]}_p.svg'

        # Load SVG paths
        paths = load_svg(input_file)

        # Find the outline and inner paths
        outline, inner_paths = find_outline(paths)

        # Connect all paths into one continuous line
        continuous_points = connect_all_paths(outline, inner_paths)

        # Save as a single continuous SVG path
        save_as_one_continuous_svg(continuous_points, output_file)
        print(f"Continuous line SVG saved to {output_file}")

        return output_file
    
    except Exception as e:
        print(f"Error converting image: {e}")
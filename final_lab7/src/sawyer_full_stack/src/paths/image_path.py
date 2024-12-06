#!/usr/bin/env/python
import svgpathtools
from svgpathtools import svg2paths
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment

def load_svg(file_path):
    paths, _ = svg2paths(file_path)
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
    largest_path = max(paths, key=lambda p: np.linalg.norm(simplify_path(p).ptp(axis=0)))
    outline = simplify_path(largest_path, num_points=1000)
    inner_paths = [p for p in paths if p != largest_path]
    return outline, inner_paths

def connect_paths(outline, inner_paths, num_points=500):
    """Connect the inner paths to the outline and minimize connecting line lengths."""
    point_clusters = [simplify_path(p, num_points) for p in inner_paths]
    ordered_points = [outline]

    # Find optimal connection order for each inner path
    for cluster in point_clusters:
        # Find the closest points between the outline and the cluster
        outline_points = np.array(outline)
        cluster_points = np.array(cluster)

        dist_matrix = cdist(outline_points, cluster_points)
        min_idx = np.unravel_index(np.argmin(dist_matrix), dist_matrix.shape)
        outline_point = outline_points[min_idx[0]]
        cluster_point = cluster_points[min_idx[1]]

        # Add connecting line and the inner cluster
        ordered_points.append([outline_point, cluster_point])
        ordered_points.append(cluster)

    return ordered_points

def plot_one_line_drawing(ordered_points):
    """Visualize the one-line drawing with outline and connecting lines."""
    plt.figure(figsize=(12, 12))

    for segment in ordered_points:
        if len(segment) == 2:
            # This is a connecting line
            x = [segment[0][0], segment[1][0]]
            y = [segment[0][1], segment[1][1]]
            plt.plot(x, y, color='red', linewidth=1.5)
        else:
            x = segment[:, 0]
            y = segment[:, 1]
            plt.plot(x, y, color='black', linewidth=0.8)

    plt.axis('equal')
    plt.axis('off')
    plt.show()

def save_one_line_svg(ordered_points, output_path="outline_with_connections.svg"):
    """Save the one-line drawing with connecting lines as an SVG."""
    svg_content = '<svg xmlns="http://www.w3.org/2000/svg" version="1.1">\n'

    for segment in ordered_points:
        if len(segment) == 2:
            x1, y1 = segment[0]
            x2, y2 = segment[1]
            svg_content += f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="red" stroke-width="1.5"/>\n'
        else:
            svg_content += '<path d="M '
            for x, y in segment:
                svg_content += f'{x},{y} '
            svg_content += '" stroke="black" fill="none" stroke-width="0.5"/>\n'

    svg_content += '</svg>'

    with open(output_path, 'w') as file:
        file.write(svg_content)

def main():
    input_file = "nike_logo.svg"
    output_file = "outlined.svg"

    # Load SVG paths
    paths = load_svg(input_file)

    # Find the outline and inner paths
    outline, inner_paths = find_outline(paths)

    # Generate one-line drawing with minimal connecting lines
    ordered_points = connect_paths(outline, inner_paths)

    # Plot the result
    plot_one_line_drawing(ordered_points)

    # Save as SVG
    save_one_line_svg(ordered_points, output_file)
    print(f"Outline with connections saved to {output_file}")

if __name__ == "__main__":
    main()
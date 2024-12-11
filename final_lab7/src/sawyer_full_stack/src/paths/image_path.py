import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from svgpathtools import svg2paths

class ImagePath(): 
    # Class for generating points, parametric x(t), y(t) of the path, and ability to plot path from an input SVG file
    def __init__(self, file_path):
        """
        Parameters
        file_path : input SVG image path
            the directory of the SVG you want to be drawn
        """
        self.file_path = file_path

    
    def parse_svg_to_waypoints(self, num_waypoints):
        #returns a np.array of waypoints, all you need so far for linear approximation method
        paths, _ = svg2paths(self.file_path)
        all_points = []

        for path in paths:
            for i in np.linspace(0, 1, num_waypoints // len(paths)):
                point = path.point(i)
                all_points.append([point.real, point.imag, 0])  # Z height doesn't matter here bc it eventually gets overidden
        # Rotating SVG image to match base frame
        for w in all_points:
            temp_x = w[0]
            temp_y = w[1]
            w[1] = -1*temp_x
            w[0] = temp_y
        return np.array(all_points)
        
    def load_svg(self, file_path):
        """Load paths from an SVG file."""
        paths, _ = svg2paths(file_path)
        return paths

    def simplify_path(self, path, num_points=500):
        """Approximate SVG path with a high-resolution polyline."""
        points = []
        for i in np.linspace(0, 1, num_points):
            points.append(path.point(i))
        points = np.array([[p.real, p.imag] for p in points])
        return points


    # def parametrize_path(self, points):
    #     """Convert a set of points to parametric equations x(t) and y(t)."""
    #     t = np.linspace(0, 1, len(points))
    #     x = points[:, 0]
    #     y = points[:, 1]

    #     # Fit cubic splines
    #     x_spline = CubicSpline(t, x)
    #     y_spline = CubicSpline(t, y)

    #     return x_spline, y_spline


    # def generate_parametric_function(self, paths):
    #     """Generate parametric functions for SVG paths."""
    #     parametric_functions = []

    #     for path in paths:
    #         points = self.simplify_path(path)
    #         x_spline, y_spline = self.parametrize_path(points)
    #         parametric_functions.append((x_spline, y_spline))

    #     return parametric_functions


    # def plot_parametric_function_with_equations(self, functions, num_points=1000):
    #     """Plot the mathematical functions representation of the SVG with equations."""
    #     plt.figure(figsize=(12, 12))
        
    #     for idx, (x_spline, y_spline) in enumerate(functions):
    #         t = np.linspace(0, 1, num_points)
    #         x = x_spline(t)
    #         y = y_spline(t)
    #         plt.plot(x, y, linewidth=0.8, label=f'Path {idx + 1}')
            
    #         # Generate simplified equations as strings
    #         x_coeffs = x_spline.cboard_origin
            
    #         # Log full equations to the console
    #         print(f"Path {idx + 1} Full Equations:")
    #         print(f"x(t) = {' + '.join([f'{coef:.5f}*t^{len(x_coeffs) - i - 1}' for i, coef in enumerate(x_coeffs.flatten())])}")
    #         print(f"y(t) = {' + '.join([f'{coef:.5f}*t^{len(y_coeffs) - i - 1}' for i, coef in enumerate(y_coeffs.flatten())])}")
    #         print("-" * 60)
            
    #         # Display simplified equation in the plot
    #         plt.text(
    #             x[0], y[0],
    #             f"x(t) ≈ {x_eq}...\n"
    #             f"y(t) ≈ {y_eq}..."board_origin
    #     plt.show()


    def save_parametric_svg(self, functions, output_path="output_parametric.svg", num_points=1000):
        """Save the parametric representation as an SVG."""
        svg_content = '<svg xmlns="http://www.w3.org/2000/svg" version="1.1">\n'
        
        for x_spline, y_spline in functions:
            t = np.linspace(0, 1, num_points)
            x = x_spline(t)
            y = y_spline(t)
            svg_content += '<path d="M '
            svg_content += ' '.join(f'{xi},{yi}' for xi, yi in zip(x, y))
            svg_content += '" stroke="black" fill="none" stroke-width="0.5"/>\n'
        
        svg_content += '</svg>'
        
        with open(output_path, 'w') as file:
            file.write(svg_content)
        
    def get_path(self):
        #retrieve parameterized paths
        paths = self.load_svg(self.file_path)
        parametric_funcs = self.generate_parametric_function(paths)
        return parametric_funcs
    
    def scale_and_center_waypoints(self, waypoints, board_origin, board_height, board_width):
        """
        Scale and center waypoints based on the board dimensions.

        Parameters
        ----------
        waypoints : numpy.ndarray
            Array of 3D waypoints from the SVG file.
        board_origin : numpy.ndarray
            Origin of the board in the robot's base frame.
        board_width : float
            Width of the board in meters.
        board_height : float
            Height of the board in meters.
board_origin
        Returns
        -------
        numpy.ndarray
            Scaled and translated waypoints.
        """
        # Compute SVG bounding box(bottom_left
        min_x, min_y = waypoints[:, 0].min(), waypoints[:, 1].min()
        max_x, max_y = waypoints[:, 0].max(), waypoints[:, 1].max()

        svg_rel_height = max_x - min_x
        svg_rel_width = max_y - min_y

        # Compute scaling factor
        scale_factor = min(board_height / svg_rel_height, board_width / svg_rel_width)
        scale_factor = 0.0001
        breakpoint()
        #TODO: rescale correctly
        # Scale waypoints
        scaled_waypoints = waypoints.copy()
        # Subtract min_x, min_y to offset any potential negative values.
        scaled_waypoints[:, 0] = (waypoints[:, 0] - min_x) * scale_factor
        scaled_waypoints[:, 1] = (waypoints[:, 1] - min_y) * scale_factor
        scaled_waypoints[:, 2] = board_origin[2] #1 cm above surface

        # Center the waypoints on the board
        #note: x is the height of the image, y refers to width of image direction
        shift_h = (board_height - (scaled_waypoints[:, 0].max() - scaled_waypoints[:, 0].min())) / 2
        shift_w = (board_width - (scaled_waypoints[:, 1].max() - scaled_waypoints[:, 1].min())) / 2
        offset_x = board_origin[0] + shift_h
        offset_y = board_origin[1] - shift_w
        scaled_waypoints[:, 0] += offset_x
        scaled_waypoints[:, 1] += offset_y
        breakpoint()
        return scaled_waypoints
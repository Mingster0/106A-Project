import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from svgpathtools import svg2paths

class ImagePath(): 
    # Class for generating points, parametric x(t), y(t) of the path, and ability to plot path from an input SVG file
    def __init__(self, total_time, file_path):
        """
        Parameters
        ----------
        total_time : float
        	desired duration of the trajectory in seconds 
        file_path : input SVG image path
            the directory of the SVG you want to be drawn
        """
        self.total_time = total_time
        self.file_path = file_path

    
    def parse_svg_to_waypoints(self, num_waypoints=150):
        #returns a np.array of waypoints, all you need so far for linear approximation method
        paths, _ = svg2paths(self.file_path)
        all_points = []

        for path in paths:
            for i in np.linspace(0, 1, num_waypoints // len(paths)):
                point = path.point(i)
                all_points.append([point.real, point.imag, 0])  # Add z=0 for 2D shapes

        return np.array(all_points)

    def load_svg(file_path):
        """Load paths from an SVG file."""
        paths, _ = svg2paths(file_path)
        return paths

    def simplify_path(path, num_points=500):
        """Approximate SVG path with a high-resolution polyline."""
        points = []
        for i in np.linspace(0, 1, num_points):
            points.append(path.point(i))
        points = np.array([[p.real, p.imag] for p in points])
        return points


    def parametrize_path(points):
        """Convert a set of points to parametric equations x(t) and y(t)."""
        t = np.linspace(0, 1, len(points))
        x = points[:, 0]
        y = points[:, 1]

        # Fit cubic splines
        x_spline = CubicSpline(t, x)
        y_spline = CubicSpline(t, y)

        return x_spline, y_spline


    def generate_parametric_function(paths):
        """Generate parametric functions for SVG paths."""
        parametric_functions = []

        for path in paths:
            points = simplify_path(path)
            x_spline, y_spline = parametrize_path(points)
            parametric_functions.append((x_spline, y_spline))

        return parametric_functions


    def plot_parametric_function_with_equations(functions, num_points=1000):
        """Plot the mathematical functions representation of the SVG with equations."""
        plt.figure(figsize=(12, 12))
        
        for idx, (x_spline, y_spline) in enumerate(functions):
            t = np.linspace(0, 1, num_points)
            x = x_spline(t)
            y = y_spline(t)
            plt.plot(x, y, linewidth=0.8, label=f'Path {idx + 1}')
            
            # Generate simplified equations as strings
            x_coeffs = x_spline.c
            y_coeffs = y_spline.c
            
            x_eq = " + ".join(
                [f"{coef:.2f}*t^{len(x_coeffs) - i - 1}" for i, coef in enumerate(x_coeffs.flatten()[:3])]
            )
            y_eq = " + ".join(
                [f"{coef:.2f}*t^{len(y_coeffs) - i - 1}" for i, coef in enumerate(y_coeffs.flatten()[:3])]
            )
            
            # Log full equations to the console
            print(f"Path {idx + 1} Full Equations:")
            print(f"x(t) = {' + '.join([f'{coef:.5f}*t^{len(x_coeffs) - i - 1}' for i, coef in enumerate(x_coeffs.flatten())])}")
            print(f"y(t) = {' + '.join([f'{coef:.5f}*t^{len(y_coeffs) - i - 1}' for i, coef in enumerate(y_coeffs.flatten())])}")
            print("-" * 60)
            
            # Display simplified equation in the plot
            plt.text(
                x[0], y[0],
                f"x(t) ≈ {x_eq}...\n"
                f"y(t) ≈ {y_eq}...",
                fontsize=8,
                bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray')
            )
        
        plt.axis('equal')
        plt.axis('off')
        plt.legend()
        plt.show()


    def save_parametric_svg(functions, output_path="output_parametric.svg", num_points=1000):
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
        
    # def get_path():
    # IGNORE THESE COMMENTS FOR NOW (Option for other parametric representation of path)
    #     input_file = "nike_logo.svg"  # Replace with your SVG file
    #     output_file = "output_parametric.svg"
        
    #     # Load SVG paths
    #     paths = load_svg(input_file)
        
    #     # Generate parametric functions
    #     parametric_functions = generate_parametric_function(paths)
    #     breakpoint()
    #     # Plot the parametric representation with equations
    #     plot_parametric_function_with_equations(parametric_functions)
        
    #     # Save the parametric SVG
    #     save_parametric_svg(parametric_functions, output_file)
    #     print(f"Parametric SVG saved to {output_file}")
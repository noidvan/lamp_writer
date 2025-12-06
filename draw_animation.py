import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import argparse


class PathDrawer:
    def __init__(self, output_file="data/ikea-lamp-drawn.json"):
        self.output_file = output_file
        self.points = []
        self.drawing = False  # Track if mouse is pressed
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        (self.line,) = self.ax.plot([], [], "b-", linewidth=2, label="Your path")
        (self.preview_line,) = self.ax.plot(
            [], [], "r--", alpha=0.3, label="Preview (scaled)"
        )

        # Set up the drawing area
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.axhline(y=0, color="k", linewidth=0.5)
        self.ax.axvline(x=0, color="k", linewidth=0.5)
        self.ax.set_title(
            "Hold mouse and drag to draw. Press ENTER when done. Press R to reset.",
            fontsize=14,
        )
        self.ax.legend()

        # Show target region (where the lamp will draw)
        target_circle = Circle(
            (0, 0),
            0.8,
            fill=False,
            edgecolor="green",
            linewidth=2,
            linestyle="--",
            label="Target region",
        )
        self.ax.add_patch(target_circle)

        # Connect events
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        print("=" * 70)
        print("Interactive Path Drawing Tool")
        print("=" * 70)
        print("\nInstructions:")
        print("  - Hold down mouse button and drag to draw")
        print("  - Release mouse to stop drawing")
        print("  - Press ENTER to finish and generate animation")
        print("  - Press R to reset and start over")
        print("  - Close window to cancel")
        print("\nTip: Draw within the green circle for best results")
        print("=" * 70)

    def on_press(self, event):
        """Handle mouse button press"""
        if event.inaxes != self.ax:
            return
        self.drawing = True
        # Add first point
        self.points.append([event.xdata, event.ydata])
        self.update_plot()

    def on_release(self, event):
        """Handle mouse button release"""
        self.drawing = False
        if len(self.points) > 0:
            print(f"Drew {len(self.points)} points")

    def on_motion(self, event):
        """Handle mouse motion while button is pressed"""
        if not self.drawing or event.inaxes != self.ax:
            return

        # Add point (with minimal spacing to avoid too many points)
        if len(self.points) > 0:
            last_point = self.points[-1]
            dx = event.xdata - last_point[0]
            dy = event.ydata - last_point[1]
            dist = np.sqrt(dx**2 + dy**2)

            # Only add point if it's far enough from last point
            if dist > 0.01:  # Minimum spacing
                self.points.append([event.xdata, event.ydata])
                self.update_plot()
        else:
            self.points.append([event.xdata, event.ydata])
            self.update_plot()

    def on_key(self, event):
        """Handle key presses"""
        if event.key == "enter":
            if len(self.points) < 2:
                print("Error: Need at least 2 points to create animation")
                return
            print("\nGenerating animation...")
            plt.close(self.fig)
            self.generate_animation()

        elif event.key == "r":
            print("\nReset - clearing all points")
            self.points = []
            self.update_plot()

    def update_plot(self):
        """Update the plot with current points"""
        if len(self.points) > 0:
            points_array = np.array(self.points)
            self.line.set_data(points_array[:, 0], points_array[:, 1])

            # Show preview of how it will be scaled/positioned
            if len(self.points) > 1:
                preview = self.get_preview_path()
                self.preview_line.set_data(preview[:, 0], preview[:, 1])
        else:
            self.line.set_data([], [])
            self.preview_line.set_data([], [])

        self.fig.canvas.draw()

    def get_preview_path(self):
        """Get preview of how the path will be scaled for the lamp"""
        if len(self.points) < 2:
            return np.array([])

        path = np.array(self.points)

        # Find bounding box
        min_vals = path.min(axis=0)
        max_vals = path.max(axis=0)
        center = (min_vals + max_vals) / 2
        size = max_vals - min_vals

        # Center the path
        centered = path - center

        # Scale to fit in target region (0.8 radius = 1.6 diameter)
        if size.max() > 0:
            scale = 1.6 / size.max()
            scaled = centered * scale
        else:
            scaled = centered

        return scaled

    def generate_animation(self):
        """Generate the IK animation from the drawn path"""
        if len(self.points) < 2:
            print("Error: Need at least 2 points")
            return

        # Load base JSON
        with open("data/ikea-lamp.json", "r") as f:
            data = json.load(f)

        # Base position
        base_x, base_y, base_z = 0.39287, 0.00056, 0.21949

        # Target position and size (same as we've been using)
        center_x = base_x - 0.6
        center_y = 0.55
        radius = 0.65

        # Process the drawn path
        path = np.array(self.points)

        # Find bounding box
        min_vals = path.min(axis=0)
        max_vals = path.max(axis=0)
        bbox_center = (min_vals + max_vals) / 2
        bbox_size = max_vals - min_vals

        # Center and scale the path
        centered = path - bbox_center

        # Scale to fit within the target radius
        if bbox_size.max() > 0:
            scale = (2 * radius * 0.8) / bbox_size.max()
            scaled = centered * scale
        else:
            scaled = centered

        # Position at target location
        # Map drawing coords: X -> X, Y -> Y
        positioned = scaled + np.array([center_x, center_y])

        # Create animation
        ik_animation = []

        def add_keyframe(t, x, y):
            ik_animation.append({"t": t, "target": [x, y, base_z]})

        time = 0.0
        speed = 0.035

        # Start at first point
        add_keyframe(time, positioned[0, 0], positioned[0, 1])

        # Draw through all points
        for i in range(1, len(positioned)):
            time += speed
            add_keyframe(time, positioned[i, 0], positioned[i, 1])

        # Hold at end
        time += 0.5
        add_keyframe(time, positioned[-1, 0], positioned[-1, 1])

        # Add to data
        data["ik_animation"] = ik_animation

        # Remove fk_animation if present
        if "fk_animation" in data:
            del data["fk_animation"]

        # Write output
        with open(self.output_file, "w") as f:
            json.dump(data, f, indent=2)

        print("\n" + "=" * 70)
        print("Animation Created!")
        print("=" * 70)
        print(f"\nAnimation details:")
        print(f"  - Original points drawn: {len(self.points)}")
        print(f"  - Target position: ({center_x:.3f}, {center_y:.3f})")
        print(f"  - Target size: {radius:.3f} units radius")
        print(f"  - Duration: {time:.1f} seconds")
        print(f"  - Keyframes: {len(ik_animation)}")
        print(f"  - Output: {self.output_file}")
        print("=" * 70)

        # Show final result
        self.show_final_result(positioned)

    def show_final_result(self, positioned_path):
        """Show the final scaled/positioned path"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

        # Original drawing
        if len(self.points) > 0:
            original = np.array(self.points)
            ax1.plot(original[:, 0], original[:, 1], "b-o", linewidth=2, markersize=8)
            ax1.set_xlim(-1, 1)
            ax1.set_ylim(-1, 1)
            ax1.set_aspect("equal")
            ax1.grid(True, alpha=0.3)
            ax1.set_title("Your Drawing", fontsize=14, fontweight="bold")
            ax1.axhline(y=0, color="k", linewidth=0.5)
            ax1.axvline(x=0, color="k", linewidth=0.5)

        # Final positioned path
        ax2.plot(
            positioned_path[:, 0],
            positioned_path[:, 1],
            "r-o",
            linewidth=2,
            markersize=6,
        )

        # Show lamp base location
        base_x = 0.39287
        ax2.plot(base_x, 0, "ks", markersize=15, label="Lamp base")

        # Show working area
        center_x = base_x - 0.6
        center_y = 0.55
        radius = 0.65
        working_circle = Circle(
            (center_x, center_y),
            radius,
            fill=False,
            edgecolor="green",
            linewidth=2,
            linestyle="--",
            label="Working area",
        )
        ax2.add_patch(working_circle)

        ax2.set_aspect("equal")
        ax2.grid(True, alpha=0.3)
        ax2.set_title("Final Animation Path", fontsize=14, fontweight="bold")
        ax2.legend()
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")

        plt.tight_layout()
        plt.show()

    def run(self):
        """Run the interactive drawing tool"""
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Interactive path drawing tool for lamp animations"
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="data/ikea-lamp-drawn.json",
        help="Output JSON file (default: data/ikea-lamp-drawn.json)",
    )

    args = parser.parse_args()

    drawer = PathDrawer(output_file=args.output)
    drawer.run()


if __name__ == "__main__":
    main()

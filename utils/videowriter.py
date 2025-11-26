import cv2
import numpy as np

class VideoWriter:
    def __init__(self, filename, fps=30, frame_size=(640, 480)):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(filename, fourcc, fps, frame_size)
        self.H = frame_size[1]
        self.W = frame_size[0]
        self.frames = []

    
    def write_frame(self, frame, postprocess=False, overlay_data=None, landing_metrics=None, recommendations=None):
        """
        overlay_data: dict with keys:
            - 'current_speed': float (m/s)
            - 'target_speed': float (m/s)
            - 'airtime': float (seconds)
            - 'is_airborne': bool
        landing_metrics: LandingMetrics object (optional, displays at landing)
        recommendations: list of str (optional, displays during flight)
        """
        if frame is not None:
            frame = np.reshape(frame, (self.H, self.W, 4))[:, :, :3].astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # Add text overlay if data provided
            if overlay_data is not None:
                frame = self._add_overlay(frame, overlay_data)
            # Add recommendations if provided and airborne
            if recommendations is not None and overlay_data and overlay_data.get('is_airborne', False):
                frame = self._add_recommendations_overlay(frame, recommendations)
            # Add landing evaluation metrics if provided
            if landing_metrics is not None:
                frame = self._add_landing_metrics_overlay(frame, landing_metrics)
            
        if postprocess:
            self.frames.append(frame)
        else:
            self.writer.write(frame)

    def _add_overlay(self, frame, data):
        """Add text overlay to frame"""
        frame_copy = frame.copy()
        
        # Font settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        text_color = (255, 255, 255)  # White
        bg_color = (0, 0, 0)  # Black background
        padding = 5
        line_height = 30
        start_y = self.H - 100  # Bottom left, leaving space for 3-4 lines
        
        # Prepare text lines
        current_speed = data.get('current_speed', 0)
        target_speed = data.get('target_speed', 0)
        airtime = data.get('airtime', 0)
        is_airborne = data.get('is_airborne', False)
        
        lines = [
            f"Speed: {current_speed:.1f} m/s",
            f"Target: {target_speed:.1f} m/s",
        ]
        
        if is_airborne:
            lines.append(f"AIRBORNE: {airtime:.2f}s")
        else:
            lines.append(f"Airtime: {airtime:.2f}s")
        
        # Draw background rectangle
        max_text_width = max([cv2.getTextSize(line, font, font_scale, font_thickness)[0][0] 
                              for line in lines])
        bg_height = len(lines) * line_height + padding * 2
        bg_width = max_text_width + padding * 2
        
        cv2.rectangle(frame_copy, 
                     (10, start_y - padding), 
                     (10 + bg_width, start_y + bg_height - padding),
                     bg_color, 
                     -1)  # Filled rectangle
        
        # Draw text lines
        for i, line in enumerate(lines):
            y_pos = start_y + i * line_height + 20
            
            # Special color for airborne status
            if "AIRBORNE" in line:
                color = (0, 255, 0)  # Green when airborne
            else:
                color = text_color
            
            cv2.putText(frame_copy, 
                       line, 
                       (10 + padding, y_pos), 
                       font, 
                       font_scale, 
                       color, 
                       font_thickness, 
                       cv2.LINE_AA)
        
        return frame_copy

    def _add_landing_metrics_overlay(self, frame, landing_metrics):
        """Add landing evaluation metrics overlay to frame"""
        frame_copy = frame.copy()
        
        # Font settings - same as _add_overlay for consistency
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        font_thickness = 1
        padding = 5
        line_height = 18
        
        # Determine colors based on success
        if landing_metrics.landing_success:
            status_color = (0, 255, 0)  # Green for success
            bg_color = (0, 0, 0)
            status_text = "LANDING: SUCCESS"
        else:
            status_color = (0, 0, 255)  # Red for failure
            bg_color = (0, 0, 0)
            status_text = "LANDING: FAILED"
        
        text_color = (255, 255, 255)  # White
        
        # Prepare compact text lines for top-right
        lines = [
            status_text,
            f"Uprightness: {landing_metrics.uprightness_score:.0f}/100",
            f"Velocity: {landing_metrics.velocity_at_impact:.1f} m/s",
            f"Stability: {landing_metrics.stability_score:.0f}/100",
        ]
        
        # Calculate dimensions
        max_text_width = max([cv2.getTextSize(line, font, font_scale, font_thickness)[0][0] 
                              for line in lines])
        bg_height = len(lines) * line_height + padding * 2
        bg_width = max_text_width + padding * 2
        
        # Position on top-right corner
        start_x = self.W - bg_width - 10
        start_y = 10
        
        # Draw background rectangle - ensure it covers all text
        cv2.rectangle(frame_copy, 
                     (start_x - padding, start_y - padding), 
                     (self.W - 5, start_y + bg_height + padding),
                     bg_color, 
                     -1)  # Filled rectangle
        
        # Draw text lines
        for i, line in enumerate(lines):
            y_pos = start_y + padding + i * line_height
            
            # First line (status) in color, others in white
            if i == 0:
                color = status_color
            else:
                color = text_color
            
            cv2.putText(frame_copy, 
                       line, 
                       (start_x, y_pos), 
                       font, 
                       font_scale, 
                       color, 
                       font_thickness, 
                       cv2.LINE_AA)
        
        return frame_copy

    def _add_recommendations_overlay(self, frame, recommendations):
        """Add improvement recommendations overlay to frame during flight"""
        frame_copy = frame.copy()
        
        # Font settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.35
        font_thickness = 1
        padding = 5
        line_height = 16
        
        # Colors
        header_color = (0, 165, 255)  # Orange for recommendations
        text_color = (255, 255, 255)  # White
        bg_color = (0, 0, 0)  # Black background
        
        # Prepare text lines with header
        lines = ["RECOMMENDATIONS:"]
        for i, rec in enumerate(recommendations[:4], 1):  # Max 4 recommendations
            lines.append(f"{i}. {rec[:50]}")  # Limit line length to 50 chars
        
        # Calculate dimensions
        max_text_width = max([cv2.getTextSize(line, font, font_scale, font_thickness)[0][0] 
                              for line in lines])
        bg_height = len(lines) * line_height + padding * 2
        bg_width = max_text_width + padding * 2
        
        # Position on center-right (below landing metrics if present)
        start_x = self.W - bg_width - 10
        start_y = 100  # Start below where landing metrics would be
        
        # Draw background rectangle
        cv2.rectangle(frame_copy, 
                     (start_x - padding, start_y - padding), 
                     (self.W - 5, start_y + bg_height + padding),
                     bg_color, 
                     -1)  # Filled rectangle
        
        # Draw text lines
        for i, line in enumerate(lines):
            y_pos = start_y + padding + i * line_height
            
            # Header in orange, recommendations in white
            if i == 0:
                color = header_color
            else:
                color = text_color
            
            cv2.putText(frame_copy, 
                       line, 
                       (start_x, y_pos), 
                       font, 
                       font_scale, 
                       color, 
                       font_thickness, 
                       cv2.LINE_AA)
        
        return frame_copy

    def release(self):
        if self.frames:
            for f in self.frames:
                self.writer.write(f)
        self.writer.release()
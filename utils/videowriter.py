import cv2
import numpy as np

class VideoWriter:
    def __init__(self, filename, fps=30, frame_size=(640, 480)):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(filename, fourcc, fps, frame_size)
        self.H = frame_size[1]
        self.W = frame_size[0]
        self.frames = []

    
    def write_frame(self, frame, postprocess=False, overlay_data=None):
        """
        overlay_data: dict with keys:
            - 'current_speed': float (m/s)
            - 'target_speed': float (m/s)
            - 'airtime': float (seconds)
            - 'is_airborne': bool
        """
        if frame is not None:
            frame = np.reshape(frame, (self.H, self.W, 4))[:, :, :3].astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # Add text overlay if data provided
            if overlay_data is not None:
                frame = self._add_overlay(frame, overlay_data)
            
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

    def release(self):
        if self.frames:
            for f in self.frames:
                self.writer.write(f)
        self.writer.release()
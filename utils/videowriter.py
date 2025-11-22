import cv2
import numpy as np

class VideoWriter:
    def __init__(self, filename, fps=30, frame_size=(640, 480)):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(filename, fourcc, fps, frame_size)
        self.H = frame_size[1]
        self.W = frame_size[0]
        self.frames = []

    
    def write_frame(self, frame, postprocess=False):
        # TODO : Feature to take a list of frames and concatenate them before writing
        if frame is not None:
            frame = np.reshape(frame, (self.H, self.W, 4))[:, :, :3].astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # TODO : Overlay text
        if postprocess:
            self.frames.append(frame)
        else:
            self.writer.write(frame)

    def save_and_release(self):
        if self.frames:
            for f in self.frames:
                self.writer.write(f)
        self.writer.release()
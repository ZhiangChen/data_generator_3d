import numpy as np
import os
import cv2

def exam_depth(data_id):
    folder = f"output/kubric_{data_id}/associations/depth"
    assert os.path.exists(folder), f"Folder '{folder}' does not exist."
    depth_files = [f for f in os.listdir(folder) if f.endswith(".png")]
    assert len(depth_files) > 0, f"No depth images found in '{folder}'."

    for depth_file in depth_files:
        depth_file_path = f"{folder}/{depth_file}"
        depth_image = cv2.imread(depth_file_path, cv2.IMREAD_UNCHANGED)
        # check if entire image is black
        if np.all(depth_image == 0):
            print(f"Depth image '{depth_file_path}' is all black.")


if __name__ == "__main__":
    for i in range(17, 64):
        exam_depth(i)

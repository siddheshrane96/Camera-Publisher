import numpy as np
import yaml

calibration_path = "/home/siddhesh/Downloads/camera_publisher_ws/src/webcam_publisher/calibration/calibration_data.npz"
camera_data = np.load(calibration_path)
camera_matrix = camera_data['camera_matrix']
dist_coeff = camera_data['dist_coeff'].flatten()
P = np.eye(3, 4)
P[:3, :3] = camera_matrix

output = {
    'image_width': 640,
    'image_height': 480,
    'camera_name': 'webcam',
    'camera_matrix': {
        'rows': 3, 'cols': 3, 'data': camera_matrix.flatten().tolist()
    },
    'distortion_model': 'plumb_bob',
    'distortion_coefficients': {
        'rows': 1, 'cols': 5, 'data': dist_coeff.tolist()
    },
    'rectification_matrix': {
        'rows': 3, 'cols': 3, 'data': [1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0]
    },
    'projection_matrix': {
        'rows': 3, 'cols': 4,
        'data': P.flatten().tolist()
    }
}

with open("calibration.yaml","w") as f:
    yaml.dump(output, f)
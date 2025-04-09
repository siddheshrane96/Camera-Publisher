import cv2
import numpy as np
import glob

checkerboard_size = (7,9) # number of inner corners (rows, col)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
square_size_cm = 1.09

objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1,2)
# print(f"{objp}")

objp *= square_size_cm

objpoints = []
imgpoints = []

images_path = "/home/siddhesh/Downloads/camera_scripts/calibration_frames/*.png"
images = glob.glob(images_path)

for frame in images:
    img = cv2.imread(frame)
    # cv2.imshow("frame",img)
    # cv2.waitKey(200)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1), criteria
        )
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)
        cv2.imshow("img", img)
        cv2.waitKey(40)

h, w = img.shape[:2]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(f"Camera Matrix\n {mtx}")
print(f"Dist Coeffs\n {dist}")
# print(f"rvecs\n {rvecs}")
# print(f"tvects\n {tvecs}")
cv2.destroyAllWindows()
total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    total_error += error

mean_error = total_error / len(objpoints)
print(f"\nMean Reprojection Error: {mean_error:.4f} pixels")

np.savez("calibration_data.npz", 
         camera_matrix=mtx, 
         dist_coeff=dist, 
         rvecs=rvecs, 
         tvecs=tvecs)
print("\nCalibration data saved to calibration_data.npz")
# img = cv2.imread(images[0])
# h, w = img.shape[:2]
# newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
# dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# cv2.imshow("Original", img)
# cv2.imshow("Undistorted", dst)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


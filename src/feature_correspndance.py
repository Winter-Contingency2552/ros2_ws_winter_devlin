import cv2
img1 = cv2.imread('eiffel1.jpg', 0)   # left image
img2 = cv2.imread('eiffel2.jpg', 0)   # right image

cv2.imshow('Eiffel 1', img1)
cv2.waitKey(0)

cv2.imshow('Eiffel 2', img2)
cv2.waitKey(0)

surf = cv2.xfeatures2d.SURF_create(400)   # Hessian threshold
kp1, des1 = surf.detectAndCompute(img1, None)
kp2, des2 = surf.detectAndCompute(img2, None)

# Return Values:
# keypoints: A list or vector of cv2.KeyPoint objects, each representing a 
#            detected keypoint with properties like its coordinates, size, 
#            orientation, and response.
# descriptors: An OutputArray (e.g., NumPy array in Python) containing the 
#              computed descriptors for each keypoint. The format and size of 
#              the descriptors depend on the specific feature algorithm used.

bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
matches = bf.knnMatch(des1, des2, k=2)

# Lowe's ratio test
good = [m for m,n in matches if m.distance < 0.7 * n.distance]
#good = [m for m,n in matches if m.distance < 0.2 * n.distance]

img_matches = cv2.drawMatches(img1, kp1, img2, kp2, good, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
cv2.imshow('SURF matches', img_matches)
cv2.waitKey(0)
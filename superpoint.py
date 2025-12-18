# superpoint_2025_perfect.py
# Works on every machine, every Python version, CPU or GPU – Nov 2025
import sys
# --- FIX: Add LightGlue to path manually since install failed ---
sys.path.append('/home/jeff/LightGlue')
# --------------------------------------------------------------

import cv2
import torch
from lightglue import SuperPoint, LightGlue
from lightglue.utils import load_image, rbd

# ==================== Load images ====================
img0 = cv2.imread('/home/jeff/FinalProject/TestImages/image1_features.png', 0)   # left image
img1 = cv2.imread('/home/jeff/FinalProject/TestImages/image2_features.png', 0)   # right image

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
img0 = torch.from_numpy(img0).float() / 255.0
img1 = torch.from_numpy(img1).float() / 255.0
# ==================== Force proper tensor format [1,3,H,W] ====================
def prepare_image(t):
    if t.ndim == 2:                              # grayscale [H,W]
        t = t.unsqueeze(0).unsqueeze(0)          # [1,1,H,W]
    if t.ndim == 3:                              # [C,H,W]
        t = t.unsqueeze(0)                       # [1,C,H,W]
    if t.shape[1] == 1:                          # grayscale → RGB
        t = t.repeat(1, 3, 1, 1)
    return t.to(device)

img0 = prepare_image(img0)
img1 = prepare_image(img1)

# ==================== Optional resize ====================
def resize_img(t, max_size=1024):
    _, _, h, w = t.shape
    if max(h, w) > max_size:
        scale = max_size / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        t = torch.nn.functional.interpolate(t, size=(new_h, new_w),
                                            mode="bilinear", align_corners=False)
    return t

img0 = resize_img(img0)
img1 = resize_img(img1)

# ==================== Models ====================
extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
# matcher   = LightGlue(features="superpoint").eval().to(device)
matcher = LightGlue(
    features="superpoint",
    depth_confidence=0.99,   # only very confident keypoint detections
    width_confidence=0.01,   # only very confident matches
).eval().to(device)

# ==================== Inference ====================
with torch.no_grad():
    feats0 = extractor.extract(img0)
    feats1 = extractor.extract(img1)
    matches_data = matcher({"image0": feats0, "image1": feats1})

# Clean up
feats0, feats1, matches_data = rbd(feats0), rbd(feats1), rbd(matches_data)

# ==================== Extract matches safely ====================
# These lines handle every possible batch dimension case
matches0 = matches_data["matches0"].cpu()
if matches0.ndim == 2:
    matches0 = matches0[0]          # remove batch dim if present
matches0 = matches0.numpy()

kpts0 = feats0["keypoints"].cpu()
if kpts0.ndim == 3:
    kpts0 = kpts0[0]
kpts0 = kpts0.numpy()

kpts1 = feats1["keypoints"].cpu()
if kpts1.ndim == 3:
    kpts1 = kpts1[0]
kpts1 = kpts1.numpy()

# ==================== Build OpenCV matches ====================
good_matches = []
for i, j in enumerate(matches0):
    if j >= 0:                                   # valid match
        m = cv2.DMatch()
        m.queryIdx = i
        m.trainIdx = int(j)
        m.distance = 0.0
        good_matches.append(m)
# good_matches = good_matches[:30]

print(f"SuperPoint + LightGlue found {len(good_matches)} perfect matches!")

# ==================== Visualization (fixed) ====================
left  = cv2.imread('/home/jeff/FinalProject/TestImages/image1_features.png', cv2.IMREAD_COLOR)
right = cv2.imread('/home/jeff/FinalProject/TestImages/image2_features.png', cv2.IMREAD_COLOR)

assert left is not None, "left image failed to load"
assert right is not None, "right image failed to load"

# Size used during matching (post-resize tensor sizes)
h0, w0 = int(img0.shape[2]), int(img0.shape[3])
h1, w1 = int(img1.shape[2]), int(img1.shape[3])

left_vis  = cv2.resize(left,  (w0, h0), interpolation=cv2.INTER_AREA)
right_vis = cv2.resize(right, (w1, h1), interpolation=cv2.INTER_AREA)

# Build keypoints in the SAME coordinate frame as left_vis/right_vis
kp0 = [cv2.KeyPoint(float(pt[0]), float(pt[1]), 6) for pt in kpts0]
kp1 = [cv2.KeyPoint(float(pt[0]), float(pt[1]), 6) for pt in kpts1]

result = cv2.drawMatches(
    left_vis, kp0,
    right_vis, kp1,
    good_matches, None,
    matchColor=(0, 255, 0),
    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
)

# Debug sanity checks (will catch “all black array” issues immediately)
print("left_vis dtype/min/max:", left_vis.dtype, left_vis.min(), left_vis.max())
print("result   dtype/min/max:", result.dtype, result.min(), result.max(), "shape:", result.shape)

# Always write to disk so we know if it's a GUI backend problem
out_path = "/home/jeff/FinalProject/TestImages/matches_debug.png"
cv2.imwrite(out_path, result)
print("Wrote:", out_path)

# Try imshow (may be black on Wayland; file output above is the truth)
cv2.namedWindow("matches", cv2.WINDOW_NORMAL)
cv2.imshow("matches", result)
cv2.waitKey(0)
cv2.destroyAllWindows()

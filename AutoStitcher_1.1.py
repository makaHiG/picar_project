from importlib.resources import path
import subprocess
import glob
import os

IMAGE_FOLDER = r"C:\Users\25caol04\Downloads\panorama0\panorama0"
PROJECT_FILE = "project.pto"

parts = IMAGE_FOLDER.split(os.sep)
OUTPUT_PREFIX = "_".join(parts[-2:])

def run(cmd):
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)

# 1️⃣ Load images
images = sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*.jpg")))

zenith_images = [img for img in images if "zenith" in img.lower()]
grid_images = [img for img in images if "zenith" not in img.lower()]

print("Total images:", len(images))
print("Grid:", len(grid_images), "Zenith:", len(zenith_images))

# 2️⃣ Angle model
pitches = [30, 60, 90, 120, 150]
yaw_step = 30
yaw_offset = -30

angles = [
    {"yaw": (yaw + yaw_offset) % 360, "pitch": pitch}
    for pitch in pitches
    for yaw in range(0, 360, yaw_step)
]

assert len(angles) == len(grid_images), "Mismatch grid/images!"

# 3️⃣ Create project
run([
    "pto_gen",
    "--fov=75",
    "--projection=2",
    "-o", PROJECT_FILE
] + images)

# 4️⃣ Apply geometry (IMPORTANT FIX: correct indexing via order assumption)
for i in range(len(grid_images)):
    ang = angles[i]
    run([
        "pto_var",
        "--set", f"y{i}={ang['yaw']}",
        "--set", f"p{i}={ang['pitch']}",
        PROJECT_FILE
    ])

# 5️⃣ Zenith handling
if zenith_images:
    zenith_index = len(grid_images)
    run([
        "pto_var",
        "--set", f"y{zenith_index}=0",
        "--set", f"p{zenith_index}=90",
        PROJECT_FILE
    ])

# 6️⃣ OPTIONAL cpfind (now safe mode only)
# ⚠️ You can comment this out completely for full robotic mode
run([
    "cpfind",
    "--prealigned",
    "--celeste",
    "-o", PROJECT_FILE,
    PROJECT_FILE
])

# 7️⃣ NO aggressive optimization (important fix)
run([
    "autooptimiser",
    "-n",   # NOT -p (prevents geometry corruption)
    "-o", PROJECT_FILE,
    PROJECT_FILE
])

# 8️⃣ Projection setup
run([
    "pano_modify",
    "--projection=2",
    "--straighten",
    "--canvas=8000x4000",
    "--ldr-compression=90",
    "--crop=0,8000,0,4000",
    "-o", PROJECT_FILE,
    PROJECT_FILE
])

# 9️⃣ Stitch (single clean pass)
run([
    "hugin_executor",
    "--stitching",
    "--prefix=" + OUTPUT_PREFIX,
    PROJECT_FILE
])

print("Panorama finished!")
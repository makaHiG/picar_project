from importlib.resources import path
import subprocess
import glob
import os

IMAGE_FOLDER = r"C:\Users\25caol04\Downloads\panorama0\panorama0"
PROJECT_FILE = "project.pto"
parts = IMAGE_FOLDER.split(os.sep)
OUTPUT_PREFIX = "_".join(parts[-2:])
#OUTPUT_PREFIX = "labpanorama3"

images = sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*.jpg")))

print("Images found:", images)
print(f"Total images: {len(images)}")

def run(cmd):
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)

# 1️⃣ Create panorama project
run(["pto_gen",
    #"-p",
    "--fov=75",
    "--projection=2",
    "-o", PROJECT_FILE] + images)

# --- Define your angle pattern ---
pitches = [30, 60, 90, 120, 150]
yaw_step = 30
yaw_offset = -30  # counterclockwise

angles = [
    {"yaw": (yaw + yaw_offset) % 360, "pitch": pitch}
    for pitch in pitches
    for yaw in range(0, 360, yaw_step)
]

# Safety check
assert len(angles) == len(images), "Mismatch between angles and images!"

# 2️⃣--- Apply angles to PTO project ---
for i, ang in enumerate(angles):
    run([
        "pto_var",
        "--set", f"y{i}={ang['yaw']}",
        "--set", f"p{i}={ang['pitch']}",
        PROJECT_FILE
    ])

#run(["pto_var", "--set", "f=28", "--set", "v=70", PROJECT_FILE])
#run(["pto_var", "--opt", "y,p,v,a,b,c", PROJECT_FILE])
# 3️⃣ Find control points (multi-row pano, more conservative)

run(["cpfind", 
    "--prealigned", 
    "--multirow", 
    "-o", PROJECT_FILE, 
    PROJECT_FILE])
# run(["linefind", "-o", PROJECT_FILE, PROJECT_FILE])

# 3.5 Remove bad control points
# run(["cpclean",
#      "--check-line-cp",
#       "-o", PROJECT_FILE, PROJECT_FILE])

# 4️⃣ Optimize camera alignment (positions only, no orientation changes)

# run([
#     "autooptimiser",
#     "-n",
#     "-p", 
#     "-m",
#     "-l",
#     "-s",
#     "-o", PROJECT_FILE,
#     PROJECT_FILE
# ])


# run([
#     "autooptimiser",
#     "-a",
#     #"-p", 
#     "-m",
#     "-l",
#     "-s",
#     "-o", PROJECT_FILE,
#     PROJECT_FILE
# ])

# run([
#     "autooptimiser",
#     "-p", 
#     #"-m",
#     #"-l",
#     "-o", PROJECT_FILE,
#     PROJECT_FILE
# ])
#run(["pano_modify", "--straighten", "--canvas=AUTO", "--crop=AUTO", "-o", PROJECT_FILE, PROJECT_FILE])

run([
    "autooptimiser",
    "-p",        # no lens changes
    "-o", PROJECT_FILE,
    PROJECT_FILE
])

# 6️⃣ Stitch panorama

#run([
#    "hugin_executor",
#    "--assistant",
#    #"--dry-run",
#    #"--stitching",
#    "--prefix=" + OUTPUT_PREFIX,
#    PROJECT_FILE
#])

# 5️⃣ Set projection and canvas for 360° panorama
run([
    "pano_modify",
    "--projection=2",   # equirectangular
    "--straighten",
    #"--ldr-file=JPG",
    "--canvas=8000x4000",
    "--ldr-compression=90",
    #"--crop=AUTO",
    "--crop=0,8000,0,4000",
    "-o", PROJECT_FILE,
    PROJECT_FILE
])

##run(["nona", "-o", PROJECT_FILE, PROJECT_FILE])
run([
    "hugin_executor",
    #"--assistant",
    #"--dry-run",
    "--stitching",
    "--prefix=" + OUTPUT_PREFIX,
    PROJECT_FILE
])
print("Panorama finished!")
import os
from PIL import Image

files = sorted(os.listdir('.'))
print("Files in directory:", files)
for f in files:
    if f.endswith('.png') or f.endswith('.jpg') or f.endswith('.jpeg'):
        im = Image.open(f)
        print(f, im.size)
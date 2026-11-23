# Hazmat image dataset

Create one subfolder per hazard label. Folder names become the model's output
labels. This is a simple closed-set classifier; it only chooses among the
hazard classes present in these folders.

```text
hazmat/
  <hazard_label_1>/
  <hazard_label_2>/
```

Put representative **full camera frames** in these folders, not only tightly
cropped sign pictures. The current model classifies each full frame and does
not locate or draw a box around a sign. Include variation in distance, angle,
lighting, background, and blur. Keep each class reasonably balanced. Avoid
placing near-identical frames from the same video in both training and
validation data, since that makes the validation score look better than real
performance. The training script skips byte-identical files automatically.

**Add a `None` folder.** The model always picks one of the folders, so without
a "no sign" class an empty wall still comes out as some hazard. Put ordinary
rover-camera frames with no sign in `hazmat/None/` (walls, floor, people, the
arena). `laptop/vision.py` treats a class named `None`, `background` or
`no_sign` as "nothing seen" and never shows it on the dashboard.

Before training, every source image is converted to grayscale and simulated at
320×240 with mild JPEG compression, random pixel noise, and a few bright/dark
pixels. Training keeps that main camera-style view and makes four copies: a
small positive rotation, a small negative rotation, a small shift, and a
mirrored image with a small rotation. Validation images get the camera-quality
simulation but no geometric copies.

Supported image types: JPG, JPEG, PNG, BMP, and WEBP.

After collecting images, install both dependency lists from the repository
root and run the training script:

```bash
pip install -r laptop/requirements.txt -r ImageProsseingTraining/training-requirements.txt
python ImageProsseingTraining/train_hazmat.py --data ImageProsseingTraining/data/hazmat --epochs 30
```

The script fine-tunes ImageNet-pretrained MobileNetV3-Small. It first trains
the new class head, then fine-tunes the last feature blocks at a smaller
learning rate. The first run downloads the pretrained weights. It reports
training and validation accuracy and writes
`laptop/models/hazmat.onnx` plus `laptop/models/hazmat.json`. Restart the laptop
server after training so the inference hook loads the model. Validation
accuracy is only a rough check; evaluate on new images and on actual rover
camera frames before relying on the label.

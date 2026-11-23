"""Fine-tune and export a pretrained MobileNetV3-Small hazmat classifier.

Dataset layout (one folder per hazard class):

    ImageProsseingTraining/data/hazmat/
      <hazard_class_1>/
      <hazard_class_2>/

Use representative full camera frames in every class. This is a closed-set
classifier: it always picks one of the class folders. Add a folder named
``None`` with ordinary rover-camera frames (no sign) so empty scenes have a
class of their own; laptop/vision.py never reports that class. Keep near-duplicate video frames together when
splitting a dataset; the built-in random split is only a starting point for
an independent-image dataset.

From the repository root, install ``ImageProsseingTraining/training-requirements.txt``
alongside ``laptop/requirements.txt``, then run:

    python ImageProsseingTraining/train_hazmat.py \
      --data ImageProsseingTraining/data/hazmat --epochs 30

The ONNX model and class metadata are written under ``laptop/models/`` and
loaded by ``laptop.vision.classify_hazmat``. The model is a classifier, so the
hazard should be visible and reasonably large in the frame; it does not draw boxes.
"""
import argparse
import copy
import hashlib
import json
import random
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small


IMAGE_SIZE = 128
CAMERA_SIZE = (320, 240)
TRAIN_VARIANTS_PER_IMAGE = 5  # original camera-style image plus four augmented copies
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
OUTPUT_DIR = Path(__file__).parent.parent / "laptop" / "models"


def make_model(class_count):
    """Use pretrained MobileNetV3-Small features with a new project head."""
    model = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.DEFAULT)
    input_features = model.classifier[-1].in_features
    model.classifier[-1] = nn.Linear(input_features, class_count)
    for parameter in model.features.parameters():
        parameter.requires_grad = False
    return model


class HazmatImages(Dataset):
    def __init__(self, records, training=False):
        self.records = records
        self.training = training

    def __len__(self):
        return len(self.records) * TRAIN_VARIANTS_PER_IMAGE if self.training else len(self.records)

    def __getitem__(self, index):
        variant_id = index // len(self.records) if self.training else 0
        path, label_id = self.records[index % len(self.records)]
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Could not read image: {path}")
        image = simulate_camera_quality(image, training=self.training, seed=index)
        if self.training:
            image = geometric_variant(image, variant_id)
        image = cv2.resize(image, (IMAGE_SIZE, IMAGE_SIZE), interpolation=cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB).astype(np.float32) / 255.0
        image = (image - np.asarray(IMAGENET_MEAN, np.float32)) / np.asarray(IMAGENET_STD, np.float32)
        tensor = torch.from_numpy(image.copy()).permute(2, 0, 1)
        return tensor, label_id


def simulate_camera_quality(image, training, seed):
    """Make a monochrome QVGA image with mild sensor and JPEG artifacts."""
    rng = np.random.default_rng(seed if not training else None)
    image = cv2.resize(image, CAMERA_SIZE, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    quality = int(rng.integers(58, 76)) if training else 70
    ok, encoded = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if ok:
        gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)

    noise_sigma = float(rng.uniform(1.5, 4.0)) if training else 2.5
    noise = rng.normal(0.0, noise_sigma, gray.shape).astype(np.float32)
    gray = np.clip(gray.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    # A few isolated bright/dark pixels approximate sensor and transmission noise.
    pixel_count = max(1, int(gray.size * 0.00015))
    ys = rng.integers(0, gray.shape[0], pixel_count)
    xs = rng.integers(0, gray.shape[1], pixel_count)
    gray[ys, xs] = rng.choice(np.array([0, 255], dtype=np.uint8), pixel_count)
    return gray


def geometric_variant(image, variant_id):
    """Keep the main view and make four copies: ±angle, shift, mirror + rotation."""
    if variant_id == 0:
        return image
    variant_id -= 1
    height, width = image.shape[:2]
    center = (width / 2.0, height / 2.0)
    max_x = width * 0.035
    max_y = height * 0.035
    if variant_id == 0:
        angle, dx, dy = random.uniform(1.5, 4.0), random.uniform(-max_x, max_x), random.uniform(-max_y, max_y)
        flip = False
    elif variant_id == 1:
        angle, dx, dy = random.uniform(-4.0, -1.5), random.uniform(-max_x, max_x), random.uniform(-max_y, max_y)
        flip = False
    elif variant_id == 2:
        angle, dx, dy = 0.0, random.uniform(-max_x, max_x), random.uniform(-max_y, max_y)
        flip = False
    else:
        angle, dx, dy = random.uniform(-3.0, 3.0), random.uniform(-max_x, max_x), random.uniform(-max_y, max_y)
        flip = True

    if flip:
        image = cv2.flip(image, 1)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    matrix[0, 2] += dx
    matrix[1, 2] += dy
    return cv2.warpAffine(image, matrix, (width, height), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REFLECT_101)


def drop_duplicates(files, seen):
    """Skip byte-identical images. A copy in both training and validation would
    make the validation score look better than it is."""
    unique = []
    for path in files:
        digest = hashlib.md5(path.read_bytes()).hexdigest()
        if digest in seen:
            print(f"skipping duplicate {path} (same as {seen[digest]})")
            continue
        seen[digest] = path
        unique.append(path)
    return unique


def load_records(data_dir, validation_fraction, seed):
    class_dirs = sorted(path for path in data_dir.iterdir() if path.is_dir())
    classes = [path.name for path in class_dirs]
    if len(classes) < 2:
        raise ValueError("Add at least two hazard class folders for classification.")

    rng = random.Random(seed)
    seen = {}                    # file hash -> first path, across all classes
    train_records, val_records = [], []
    for label_id, class_dir in enumerate(class_dirs):
        files = sorted(path for path in class_dir.rglob("*") if path.suffix.lower() in EXTENSIONS)
        files = drop_duplicates(files, seen)
        if len(files) < 2:
            raise ValueError(f"Class {class_dir.name!r} needs at least 2 images; found {len(files)}.")
        rng.shuffle(files)
        val_count = max(1, round(len(files) * validation_fraction))
        val_records.extend((path, label_id) for path in files[:val_count])
        train_records.extend((path, label_id) for path in files[val_count:])
    if not train_records:
        raise ValueError("No training images remain. Add images or lower --validation-fraction.")
    return classes, train_records, val_records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path(__file__).parent / "data" / "hazmat")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--warmup-epochs", type=int, default=5,
                        help="train the new class head before fine-tuning the final CNN blocks")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--confidence-threshold", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    if not 0.05 <= args.validation_fraction < 0.5:
        parser.error("--validation-fraction must be between 0.05 and 0.5")
    if args.epochs < 1 or args.batch_size < 1 or args.warmup_epochs < 0:
        parser.error("--epochs and --batch-size must be positive; warmup cannot be negative")
    if not 0.0 < args.confidence_threshold < 1.0:
        parser.error("--confidence-threshold must be between 0 and 1")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    classes, train_records, val_records = load_records(args.data, args.validation_fraction, args.seed)
    train_dataset = HazmatImages(train_records, training=True)
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size,
                              shuffle=True, num_workers=0)
    val_loader = DataLoader(HazmatImages(val_records), batch_size=args.batch_size,
                            shuffle=False, num_workers=0)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = make_model(len(classes)).to(device)
    optimizer = torch.optim.AdamW(
        [{"params": model.classifier.parameters(), "lr": 5e-4}], weight_decay=1e-4
    )
    loss_fn = nn.CrossEntropyLoss()
    best_val_accuracy = -1.0
    best_val_loss = float("inf")
    best_epoch = 0
    best_weights = None

    for epoch in range(1, args.epochs + 1):
        if epoch == args.warmup_epochs + 1:
            fine_tune_parameters = []
            for block in model.features[-3:]:
                for parameter in block.parameters():
                    parameter.requires_grad = True
                    fine_tune_parameters.append(parameter)
            if fine_tune_parameters:
                optimizer.add_param_group({"params": fine_tune_parameters, "lr": 1e-5})

        model.train()
        # Keep pretrained BatchNorm statistics stable on this small dataset.
        for module in model.features.modules():
            if isinstance(module, nn.BatchNorm2d):
                module.eval()
        train_loss = 0.0
        train_correct = 0
        train_total = 0
        for images, labels in train_loader:
            images, labels = images.to(device), labels.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(images)
            loss = loss_fn(logits, labels)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * labels.size(0)
            train_correct += (logits.argmax(1) == labels).sum().item()
            train_total += labels.size(0)

        model.eval()
        val_correct = 0
        val_total = 0
        val_loss = 0.0
        with torch.inference_mode():
            for images, labels in val_loader:
                logits = model(images.to(device))
                val_loss += loss_fn(logits, labels.to(device)).item() * labels.size(0)
                val_correct += (logits.argmax(1).cpu() == labels).sum().item()
                val_total += labels.size(0)
        val_accuracy = val_correct / max(val_total, 1)
        mean_val_loss = val_loss / max(val_total, 1)
        print(
            f"epoch {epoch:02d}/{args.epochs} "
            f"loss={train_loss / max(train_total, 1):.4f} "
            f"train_acc={train_correct / max(train_total, 1):.1%} "
            f"val_acc={val_accuracy:.1%} val_loss={mean_val_loss:.4f}"
        )
        # Keep the epoch with the lowest validation loss. With a small validation
        # set the accuracy reaches 100 % almost at once, so choosing by accuracy
        # kept an under-trained epoch-1 model that was only ~50 % sure of anything.
        if (mean_val_loss < best_val_loss or
                (mean_val_loss == best_val_loss and val_accuracy > best_val_accuracy)):
            best_val_accuracy = val_accuracy
            best_val_loss = mean_val_loss
            best_epoch = epoch
            best_weights = copy.deepcopy(model.state_dict())

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    model.load_state_dict(best_weights)
    model = model.cpu().eval()
    example = torch.zeros(1, 3, IMAGE_SIZE, IMAGE_SIZE, dtype=torch.float32)
    onnx_path = OUTPUT_DIR / "hazmat.onnx"
    torch.onnx.export(
        model, example, str(onnx_path), input_names=["image"], output_names=["logits"],
        opset_version=18, dynamo=True, external_data=False,
    )
    metadata = {
        "architecture": "mobilenet_v3_small",
        "pretrained_weights": "MobileNet_V3_Small_Weights.DEFAULT",
        "classes": classes,
        "image_size": IMAGE_SIZE,
        "normalization_mean": IMAGENET_MEAN,
        "normalization_std": IMAGENET_STD,
        "confidence_threshold": args.confidence_threshold,
        "validation_accuracy": best_val_accuracy,
        "validation_loss": best_val_loss,
        "best_epoch": best_epoch,
        "train_source_images": len(train_records),
        "train_augmented_images_per_epoch": len(train_dataset),
        "validation_images": len(val_records),
    }
    (OUTPUT_DIR / "hazmat.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"Saved ONNX model: {onnx_path}")
    print(f"Saved class metadata: {OUTPUT_DIR / 'hazmat.json'}")


if __name__ == "__main__":
    main()

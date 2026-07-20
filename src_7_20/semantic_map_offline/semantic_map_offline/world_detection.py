"""YOLO-World model loading and result conversion without ROS dependencies."""

from pathlib import Path
from typing import Iterable


def load_class_prompts(path: str | Path, excluded: Iterable[str] = ()) -> list[str]:
    """Load one text prompt per line while preserving stable class IDs."""
    class_path = Path(path).expanduser()
    if not class_path.is_file():
        raise FileNotFoundError(f"YOLO-World class list not found: {class_path}")

    excluded_names = {str(name).strip().casefold() for name in excluded if str(name).strip()}
    prompts: list[str] = []
    seen: set[str] = set()
    for raw_line in class_path.read_text(encoding="utf-8").splitlines():
        prompt = raw_line.strip()
        key = prompt.casefold()
        if not prompt or prompt.startswith("#") or key in excluded_names or key in seen:
            continue
        prompts.append(prompt)
        seen.add(key)

    if not prompts:
        raise ValueError(f"No usable YOLO-World prompts in: {class_path}")
    return prompts


def load_yolo_world(model_path: str | Path, prompts: list[str], clip_model_path: str | Path):
    """Load YOLO-World and configure Ultralytics to use a local CLIP checkpoint."""
    model_file = Path(model_path).expanduser().resolve()
    clip_file = Path(clip_model_path).expanduser().resolve()
    if not model_file.is_file():
        raise FileNotFoundError(f"YOLO-World model not found: {model_file}")
    if not clip_file.is_file():
        raise FileNotFoundError(f"CLIP model not found: {clip_file}")

    try:
        from ultralytics import YOLOWorld
        from ultralytics.nn import text_model
    except ImportError as exc:
        raise RuntimeError(
            "YOLO-World dependencies are missing. Install requirements-yolo-world.txt"
        ) from exc

    # Ultralytics resolves `clip/ViT-B-32.pt` below this directory.
    text_model.WEIGHTS_DIR = clip_file.parent.parent if clip_file.parent.name == "clip" else clip_file.parent
    model = YOLOWorld(str(model_file))
    model.set_classes(prompts)
    return model


def detections_from_result(result) -> list[dict]:
    """Convert an Ultralytics result into the recorder-compatible detection schema."""
    if result.boxes is None:
        return []
    names = getattr(result, "names", {}) or {}
    boxes = result.boxes
    xyxy = boxes.xyxy.detach().cpu().numpy()
    confidences = boxes.conf.detach().cpu().numpy()
    class_ids = boxes.cls.detach().cpu().numpy().astype(int)

    detections = []
    for box, confidence, class_id in zip(xyxy, confidences, class_ids):
        if isinstance(names, dict):
            class_name = names.get(int(class_id), str(class_id))
        else:
            class_name = names[int(class_id)] if int(class_id) < len(names) else str(class_id)
        detections.append({
            "class_id": int(class_id),
            "class_name": str(class_name),
            "confidence": float(confidence),
            "xyxy": [float(value) for value in box.tolist()],
        })
    return detections

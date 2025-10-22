# src/model.py
import torchvision
from torchvision.models.detection.anchor_utils import AnchorGenerator
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.rpn import RPNHead


def _as_tuple_of_tuples(x):
    """
        Normalize sizes into tuple-of-tuples.
        Accepts:
    (64,128,256,384,512)          -> ((64,),(128,),(256,),(384,),(512,))
    ((64,), (128,), (256,), ...)  -> used as-is
    """
    if isinstance(x, (list, tuple)):
        if len(x) == 0:
            raise ValueError("anchor_sizes cannot be empty")
        if isinstance(x[0], (list, tuple)):
            return tuple(tuple(int(v) for v in t) for t in x)
        else:
            return tuple((int(v),) for v in x)
    raise ValueError("anchor_sizes must be list/tuple")


def _broadcast_ratios(ratios, num_levels):
    """
        Normalize aspect_ratios to a tuple-of-tuples with length == num_levels.
        Accepts:
    (0.75, 1.0, 1.5)                  -> broadcast to all levels
    ((0.75, 1.0, 1.5),)               -> broadcast to all levels
    ((...), (...), ..., (...)) len=N  -> used as-is (must match num_levels)
    """
    if not isinstance(ratios, (list, tuple)) or len(ratios) == 0:
        raise ValueError("aspect_ratios must be a non-empty list/tuple")
    # Case A: flat list/tuple of floats -> one level -> broadcast
    if not isinstance(ratios[0], (list, tuple)):
        one = tuple(float(v) for v in ratios)
        return tuple([one] * num_levels)
    # Case B: list/tuple of tuples
    rt = tuple(tuple(float(v) for v in r) for r in ratios)
    if len(rt) == 1:
        # Single level provided -> broadcast to all FPN levels
        return tuple([rt[0]] * num_levels)
    if len(rt) != num_levels:
        raise ValueError(
            f"aspect_ratios has {len(rt)} levels but sizes has {num_levels}"
        )
    return rt


def get_model(
    num_classes,
    anchor_sizes=((64,), (128,), (256,), (384,), (512,)),
    anchor_ratios=((0.75, 1.0, 1.5),),
    freeze_backbone=False,
):
    """
    Build Faster R-CNN with anchors baked-in (RPN head rebuilt to match).
    """
    try:
        model = torchvision.models.detection.fasterrcnn_resnet50_fpn(
            weights="DEFAULT", pretrained=False
        )
    except TypeError:
        model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
    sizes = _as_tuple_of_tuples(anchor_sizes)
    ratios = _broadcast_ratios(anchor_ratios, num_levels=len(sizes))
    # Construct matching anchor generator + RPN head
    anchor_gen = AnchorGenerator(sizes=sizes, aspect_ratios=ratios)
    num_anchors_per_loc = anchor_gen.num_anchors_per_location()[0]
    out_channels = model.backbone.out_channels
    model.rpn.anchor_generator = anchor_gen
    model.rpn.head = RPNHead(out_channels, num_anchors_per_loc)
    # Replace ROI head for your class count
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
    if freeze_backbone:
        for _, p in model.backbone.named_parameters():
            p.requires_grad = False
    print(
        f"[model] anchors: sizes={sizes} | ratios={ratios} | levels={len(sizes)} | anchors/loc={num_anchors_per_loc}"
    )
    return model

# pyright: reportMissingImports=false
from maix import nn

# YOLO 靶纸搜索参数。
YOLO_MODEL_PATH = "/root/models/rect/model_3356.mud"
YOLO_CONF_TH = 0.50
YOLO_IOU_TH = 0.45
YOLO_MAX_CANDIDATES = 3
YOLO_ROI_MARGIN_RATIO = 0.25
YOLO_MIN_ROI_MARGIN = 12


def load_yolo_detector():
    """加载 YOLO 靶纸搜索模型，加载失败时返回 None，避免程序直接崩溃。"""
    try:
        detector = nn.YOLOv5(model=YOLO_MODEL_PATH, dual_buff=True)
        print("YOLO 模型加载成功:%s" % YOLO_MODEL_PATH)
        return detector
    except Exception as exc:
        print("YOLO 模型加载失败:%s" % exc)
        return None


def clamp_int(value, min_value, max_value):
    """将整数限制在指定范围内，避免 ROI 越界。"""
    value = int(value)
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def get_yolo_roi(obj, frame_w, frame_h):
    """根据 YOLO 检测框生成稍微放大的 ROI 区域。"""
    margin = max(YOLO_MIN_ROI_MARGIN, int(max(obj.w, obj.h) * YOLO_ROI_MARGIN_RATIO))
    x1 = clamp_int(obj.x - margin, 0, frame_w - 1)
    y1 = clamp_int(obj.y - margin, 0, frame_h - 1)
    x2 = clamp_int(obj.x + obj.w + margin, 1, frame_w)
    y2 = clamp_int(obj.y + obj.h + margin, 1, frame_h)

    if x2 <= x1 or y2 <= y1:
        return None

    return x1, y1, x2, y2


def find_rectangle_by_yolo_roi(detector, img, img_cv, find_best_rectangle, order_points):
    """先用 YOLO 找疑似靶纸，再在候选 ROI 内用 OpenCV 算法验证矩形。"""
    if detector is None:
        return None, None, None

    try:
        objs = detector.detect(img, conf_th=YOLO_CONF_TH, iou_th=YOLO_IOU_TH)
    except Exception as exc:
        print("YOLO 检测失败:%s" % exc)
        return None, None, None

    if not objs:
        return None, None, None

    # 优先验证置信度最高的几个目标，避免一个误检框直接挡住真正靶纸。
    objs = sorted(objs, key=lambda obj: obj.score, reverse=True)
    for obj in objs[:YOLO_MAX_CANDIDATES]:
        roi = get_yolo_roi(obj, img_cv.shape[1], img_cv.shape[0])
        if not roi:
            continue

        x1, y1, x2, y2 = roi
        roi_cv = img_cv[y1:y2, x1:x2]
        if roi_cv.size == 0:
            continue

        best = find_best_rectangle(roi_cv)
        if not best:
            continue

        # find_best_rectangle 返回的是 ROI 内坐标，这里换算回整张图坐标。
        ordered = order_points(best["approx"])
        ordered[:, 0] += x1
        ordered[:, 1] += y1
        return best, ordered, obj

    return None, None, None

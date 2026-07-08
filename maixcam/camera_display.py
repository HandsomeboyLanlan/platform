# pyright: reportMissingImports=false
from maix import app, camera, display, time, image
import cv2
import numpy as np
from my_uart import UartCom

# 相机分辨率，后续显示、中心点和误差计算都基于这个尺寸。
CAM_W = 512
CAM_H = 320

# OpenCV 识别使用较小分辨率，减少阈值分割和轮廓搜索的计算量。
PROC_W = 256
PROC_H = 160

# 矩形候选框筛选参数。
# MIN_AREA 用于过滤小噪声；
# APPROX_EPSILON_RATIO 用于控制轮廓拟合精度；
# MIN_ASPECT / MAX_ASPECT 用于过滤长宽比明显异常的框。
MIN_AREA = 1200     # 最小面积，低于此面积的几何框都会被过滤掉
MAX_AREA = int(CAM_W * CAM_H * 0.70)  # 最大面积，避免把背景大边框误当成靶框。
MIN_ASPECT = 0.45   # 最小长宽比
MAX_ASPECT = 2.8    # 最大长宽比
MIN_EXTENT = 0.45    # 轮廓面积 / 外接矩形面积，过小说明形状不像完整矩形。
APPROX_EPSILON_RATIO = 0.02
ADAPTIVE_BLOCK_SIZE = 31
ADAPTIVE_C = 8

# 跟踪稳定性参数：
# 目标中心单帧跳变太大时，优先认为是误识别，连续多次跳变后再允许重新锁定。
MAX_CENTER_JUMP = 120
MAX_JUMP_REJECTS = 3
MAX_LOST_FRAMES = 5
FPS_PRINT_INTERVAL = 10


def find_best_rectangle(img_cv):
    """在一帧 OpenCV 图像中寻找最可能的矩形靶框。"""
    frame_h, frame_w = img_cv.shape[:2]
    area_scale = (frame_w * frame_h) / (CAM_W * CAM_H)
    min_area = MIN_AREA * area_scale
    max_area = MAX_AREA * area_scale

    # 转灰度后只保留亮度信息，降低颜色变化对识别的影响。
    gray = cv2.cvtColor(img_cv, cv2.COLOR_RGB2GRAY)

    # 高斯滤波用于压制小噪点，让后面的二值化结果更稳定。
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # 自适应阈值适合现场光照不均匀的情况；
    # THRESH_BINARY_INV 会把较暗的边框变成白色前景，便于找轮廓。
    binary = cv2.adaptiveThreshold(
        blur,
        255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY_INV,
        ADAPTIVE_BLOCK_SIZE,
        ADAPTIVE_C)
    
    # 只提取外轮廓，减少靶面内部线条、文字或噪点的干扰。
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None

    for contour in contours:
        # 面积太小的轮廓大概率是噪声，直接丢弃。
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue

        # 将轮廓拟合成多边形，矩形目标应当近似为 4 个角点。
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, APPROX_EPSILON_RATIO * perimeter, True)
        if len(approx) != 4:
            continue

        # 正常矩形应该是凸四边形，凹四边形或自交形状容易来自杂乱边缘。
        if not cv2.isContourConvex(approx):
            continue
        
        # 用外接矩形计算候选区域的位置、宽高和长宽比。
        x, y, w, h = cv2.boundingRect(approx)
        rect_area = w * h
        if h == 0 or rect_area <= 0:
            continue

        aspect = w / h
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue

        # 填充率太低时，说明轮廓很可能不是完整靶框。
        extent = area / rect_area
        if extent < MIN_EXTENT:
            continue

        # 保存候选矩形信息，后续可用于画角点、计算中心和串口发送。
        candidate = {
            "area": area,
            "rect": (x, y, w, h),
            "center": (x + w // 2, y + h // 2),
            "extent": extent,
            "approx": approx,
        }
        
        # 当前策略：从所有候选矩形中选面积最大的一个作为靶框。
        if best is None or area > best["area"]:
            best = candidate
    
    return best


def draw_points(img, points):
    """在 Maix 图像上绘制一组角点。"""
    for point in points:
        x, y = int(point[0]), int(point[1])
        img.draw_circle(x, y, 5, image.COLOR_RED, thickness=-1)


def order_points(points):
    """将四个角点排序为：左上、右上、右下、左下。"""
    # OpenCV 的角点形状通常是 (4, 1, 2)，先整理成 (4, 2) 方便计算。
    pts = points.reshape(4, 2).astype("float32")
    ordered = np.zeros((4, 2), dtype="float32")

    # x + y 最小的是左上角，最大的是右下角。
    s = pts.sum(axis=1)
    ordered[0] = pts[np.argmin(s)]
    ordered[2] = pts[np.argmax(s)]

    # y - x 最小的是右上角，最大的是左下角。
    diff = np.diff(pts, axis=1).reshape(4)
    ordered[1] = pts[np.argmin(diff)]
    ordered[3] = pts[np.argmax(diff)]

    return ordered


def draw_ordered_points(img, ordered):
    """在 Maix 图像上绘制一组排序后的角点，并标注角点名称。"""
    labels = ["TL", "TR", "BR", "BL"]
    colors = [
        image.COLOR_RED,    # 左上角
        image.COLOR_GREEN,  # 右上角
        image.COLOR_BLUE,   # 右下角
        image.COLOR_YELLOW, # 左下角
    ]

    for i, point in enumerate(ordered):
        x, y = int(point[0]), int(point[1])
        img.draw_circle(x, y, 5, colors[i], thickness=-1)
        img.draw_string(x + 4, y - 4, labels[i], colors[i], scale=1)

def draw_polygon(img, ordered):
    """在 Maix 图像上绘制一个多边形，连接四个角点。"""
    pts = ordered.astype("int32")
    for i in range(4):
        x1, y1 = int(pts[i][0]), int(pts[i][1])
        x2, y2 = int(pts[(i + 1) % 4][0]), int(pts[(i + 1) % 4][1])
        img.draw_line(x1, y1, x2, y2, image.COLOR_GREEN, thickness=2)

def get_polygon_center(ordered):
    """计算多边形的中心点坐标。"""
    tl, tr, br, bl = ordered
    x1, y1 = tl
    x2, y2 = br
    x3, y3 = tr
    x4, y4 = bl

    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-6:
        # 如果分母接近零，说明四个点可能共线，无法计算交点。
        center = ordered.mean(axis=0)
        return int(center[0]), int(center[1])
    
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
    return int(px), int(py)

def warp_perspective_target(img_cv, ordered, make_image=False):
    """计算透视矩阵；需要调试或后续识别靶面细节时再生成拉正图。"""
    ordered = ordered.astype("float32")

    # 先检查四边形面积，面积太小说明角点异常。
    area = abs(cv2.contourArea(ordered))
    if area < MIN_AREA:
        return None
    
    tl, tr, br, bl = ordered

    width_top = np.linalg.norm(tr - tl)
    width_bottom = np.linalg.norm(br - bl)
    height_left = np.linalg.norm(tl - bl)
    height_right = np.linalg.norm(tr - br)

    warp_w = int(max(width_top, width_bottom))
    warp_h = int(max(height_left, height_right))

    if warp_w < 20 or warp_h < 20:
        return None
    
    dst = np.array(
        [
            [0, 0],
            [warp_w - 1, 0],
            [warp_w - 1, warp_h - 1],
            [0, warp_h - 1]
        ],
        dtype="float32",
    )

    # 正透视：原图靶纸 -> 拉正图
    matrix = cv2.getPerspectiveTransform(ordered, dst)
    # 逆透视：拉正图 -> 原图靶纸
    matrix_inv = cv2.getPerspectiveTransform(dst, ordered)
    warped = None
    if make_image:
        warped = cv2.warpPerspective(img_cv, matrix, (warp_w, warp_h))
    
    return {
        "image": warped,
        "matrix": matrix,
        "matrix_inv": matrix_inv,
        "size": (warp_w, warp_h),
    }


def center_jump_too_large(center, last_center):
    """判断目标中心是否发生不合理的大跳变。"""
    if last_center is None:
        return False

    dx = center[0] - last_center[0]
    dy = center[1] - last_center[1]
    return dx * dx + dy * dy > MAX_CENTER_JUMP * MAX_CENTER_JUMP


def run_find_rects():
    """启动相机预览，并在循环中持续查找矩形靶框。"""
    uart_com = UartCom()
    cam = camera.Camera(CAM_W, CAM_H)
    disp = display.Display()
    last_center = None
    lost_frames = 0
    jump_rejects = 0
    frame_count = 0

    while not app.need_exit():
        # 读取一帧 Maix 图像，并转换成 OpenCV 图像用于算法处理。
        img = cam.read()
        img_cv = image.image2cv(img, False, False)
        frame_count += 1

        # 在小图上做矩形识别，显著减少 OpenCV 计算量。
        proc_cv = cv2.resize(img_cv, (PROC_W, PROC_H), interpolation=cv2.INTER_NEAREST)
        best = find_best_rectangle(proc_cv)
        if not best:
            # 无目标
            uart_com.send_no_target()
            lost_frames += 1
            if lost_frames >= MAX_LOST_FRAMES:
                last_center = None
                jump_rejects = 0
            disp.show(img)
            continue

        # 将小图中识别到的角点放大回原图坐标，用于显示、逆透视和串口误差计算。
        ordered = order_points(best["approx"])
        scale_x = img_cv.shape[1] / PROC_W
        scale_y = img_cv.shape[0] / PROC_H
        ordered[:, 0] *= scale_x
        ordered[:, 1] *= scale_y

        # 当前追踪只需要矩阵和中心点，不生成拉正图，节省一大块计算量。
        warp_result = warp_perspective_target(img_cv, ordered, make_image=False)
        if not warp_result:
            # 无目标
            uart_com.send_no_target()
            lost_frames += 1
            if lost_frames >= MAX_LOST_FRAMES:
                last_center = None
                jump_rejects = 0
            disp.show(img)
            continue
        
        # 在标准图中拿到目标点的坐标
        std_w, std_h = warp_result["size"]
        center_std = (std_w // 2, std_h // 2)
        
        # 映射回原图
        std_center_points = np.array([[center_std]], dtype=np.float32)
        M_inv = warp_result["matrix_inv"]
        original_center_point = cv2.perspectiveTransform(std_center_points, M_inv)[0][0]
        # 如果逆透视有问题，就丢弃这一帧
        if not np.all(np.isfinite(original_center_point)):
            uart_com.send_no_target()
            lost_frames += 1
            if lost_frames >= MAX_LOST_FRAMES:
                last_center = None
                jump_rejects = 0
            disp.show(img)
            continue
        ox, oy = int(original_center_point[0]), int(original_center_point[1])

        # 如果中心点突然跳得很远，优先认为是误识别；连续多次跳变后放弃旧目标重新锁定。
        if center_jump_too_large((ox, oy), last_center):
            jump_rejects += 1
            uart_com.send_no_target()
            img.draw_string(8, 8, "jump reject", image.COLOR_YELLOW, scale=1)
            if jump_rejects >= MAX_JUMP_REJECTS:
                last_center = None
                jump_rejects = 0
            disp.show(img)
            continue

        last_center = (ox, oy)
        lost_frames = 0
        jump_rejects = 0

        # 计算目标点与屏幕中心的误差，用于传给电机主控板，进行PID控制
        screen_center = (img.width() // 2, img.height() // 2)
        err_x = ox - screen_center[0]
        err_y = oy - screen_center[1]

        # 有目标，发送误差
        uart_com.send_error(err_x, err_y)

        draw_polygon(img, ordered)
        img.draw_circle(ox, oy, 4, image.COLOR_RED, thickness=-1)
        img.draw_circle(screen_center[0], screen_center[1], 4, image.COLOR_BLUE, thickness=-1)
        img.draw_string(8, 8, f"err:({err_x}, {err_y})", image.COLOR_GREEN, scale=1)
        disp.show(img)

        # 打印当前处理耗时和 FPS，用于调参时观察性能。
        fps = time.fps()
        if fps > 0 and frame_count % FPS_PRINT_INTERVAL == 0:
            print(f"time:{1000 / fps:.02f}ms, fps:{fps:.02f}")

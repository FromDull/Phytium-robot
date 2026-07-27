#!/usr/bin/env python3
"""Low-rate YOLO ONNX detector for the dashboard RGB frame endpoint."""

import argparse
import io
import json
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import onnxruntime as ort
from PIL import Image

COCO_NAMES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
    "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
)

VIEW_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>YOLO 实时识别</title>
<style>
  :root{color-scheme:dark;font-family:system-ui,-apple-system,"Segoe UI",sans-serif}
  *{box-sizing:border-box}body{margin:0;background:#0b0f14;color:#e7edf4}
  main{width:min(1100px,100%);margin:auto;padding:18px}
  header{display:flex;align-items:end;justify-content:space-between;gap:12px;margin-bottom:12px}
  h1{font-size:20px;margin:0}.hint{color:#8fa3b8;font-size:13px}
  .stage{position:relative;width:100%;aspect-ratio:4/3;background:#05070a;border:1px solid #263342;border-radius:10px;overflow:hidden}
  #stream,#overlay{position:absolute;inset:0;width:100%;height:100%}
  #stream{object-fit:contain}#overlay{pointer-events:none}
  .panel{display:grid;grid-template-columns:1fr auto;gap:12px;margin-top:12px;padding:12px;border:1px solid #263342;border-radius:10px;background:#111821}
  #status{font-variant-numeric:tabular-nums;color:#b9c8d8}#objects{white-space:pre-wrap;text-align:right;color:#65e6a7}
  .bad{color:#ff7b86!important}@media(max-width:650px){main{padding:10px}.panel{grid-template-columns:1fr}#objects{text-align:left}}
</style>
</head>
<body><main>
<header><div><h1>YOLO 实时识别</h1><div class="hint">RGB 实时画面与浏览器端检测框叠加</div></div><div class="hint" id="clock"></div></header>
<div class="stage"><img id="stream" alt="RGB 实时画面"><canvas id="overlay" width="320" height="240"></canvas></div>
<div class="panel"><div id="status">正在连接识别服务…</div><div id="objects">等待目标</div></div>
</main>
<script>
const stream=document.querySelector('#stream'),canvas=document.querySelector('#overlay'),ctx=canvas.getContext('2d');
const statusEl=document.querySelector('#status'),objectsEl=document.querySelector('#objects'),clockEl=document.querySelector('#clock');
let frameTimer=0;
function pullLatestFrame(){clearTimeout(frameTimer);stream.src='/frame.jpg?t='+Date.now()}
stream.addEventListener('load',()=>{statusEl.classList.remove('bad');frameTimer=setTimeout(pullLatestFrame,100)});
stream.addEventListener('error',()=>{statusEl.textContent='实时画面断开，正在重连…';statusEl.classList.add('bad');frameTimer=setTimeout(pullLatestFrame,1000)});
function draw(data){
  const w=data.source_width||320,h=data.source_height||240;if(canvas.width!==w)canvas.width=w;if(canvas.height!==h)canvas.height=h;
  ctx.clearRect(0,0,w,h);ctx.font='12px system-ui';ctx.textBaseline='top';ctx.lineWidth=2;
  const rows=[];
  for(const d of data.detections||[]){
    const [x1,y1,x2,y2]=d.box,label=d.label+' '+Math.round(d.confidence*100)+'%';
    ctx.strokeStyle='#39f59d';ctx.fillStyle='rgba(57,245,157,.16)';ctx.fillRect(x1,y1,x2-x1,y2-y1);ctx.strokeRect(x1,y1,x2-x1,y2-y1);
    const tw=ctx.measureText(label).width+8,ty=Math.max(0,y1-18);ctx.fillStyle='rgba(3,18,13,.9)';ctx.fillRect(x1,ty,tw,18);ctx.fillStyle='#8bffc5';ctx.fillText(label,x1+4,ty+2);
    rows.push(label);
  }
  objectsEl.textContent=rows.length?rows.join('\\n'):'当前未发现目标';
  const age=data.updated_at?Math.max(0,Date.now()/1000-data.updated_at):0;
  statusEl.classList.toggle('bad',!data.online);
  statusEl.textContent=data.online
    ?`识别 ${data.input_width}x${data.input_height} · 推理 ${data.inference_ms} ms · 周期 ${data.cycle_ms} ms · 结果延迟 ${age.toFixed(1)} s`
    :`识别离线：${data.error||'未知错误'}`;
}
async function poll(){try{const r=await fetch('/detections',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);draw(await r.json())}catch(e){statusEl.textContent='识别结果连接失败：'+e.message;statusEl.classList.add('bad')}finally{setTimeout(poll,200)}}
setInterval(()=>clockEl.textContent=new Date().toLocaleTimeString(),1000);pullLatestFrame();poll();
</script></body></html>""".encode("utf-8")


def iou_one(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    area_a = max(0, box[2] - box[0]) * max(0, box[3] - box[1])
    area_b = np.maximum(0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0, boxes[:, 3] - boxes[:, 1])
    return intersection / np.maximum(area_a + area_b - intersection, 1e-6)


class Detector:
    def __init__(
        self, model_path, rgb_url, confidence, interval, input_size, stream_fps, jpeg_quality
    ):
        options = ort.SessionOptions()
        # Keep one CPU available for camera, USB audio, and kernel I/O work on
        # the three-core Phytium Pi. Two inference threads can otherwise
        # saturate the board and cause isochronous USB buffer overruns.
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(
            model_path, sess_options=options, providers=["CPUExecutionProvider"]
        )
        self.input = self.session.get_inputs()[0]
        shape = self.input.shape
        self.input_height = int(shape[2]) if isinstance(shape[2], int) else input_size
        self.input_width = int(shape[3]) if isinstance(shape[3], int) else input_size
        self.rgb_url = rgb_url
        self.confidence = confidence
        self.interval = interval
        self.stream_fps = stream_fps
        self.jpeg_quality = jpeg_quality
        self.lock = threading.Lock()
        self.result = {
            "online": False,
            "detections": [],
            "inference_ms": None,
            "input_width": self.input_width,
            "input_height": self.input_height,
            "updated_at": None,
            "error": "waiting for RGB frame",
        }

    @staticmethod
    def fetch_frame(url):
        with urllib.request.urlopen(url, timeout=3) as response:
            body = response.read()
        header_length = int.from_bytes(body[:4], "big")
        header = json.loads(body[4 : 4 + header_length])
        data = body[4 + header_length :]
        if header["encoding"] != "rgb8":
            raise ValueError(f'expected rgb8, received {header["encoding"]}')
        image = np.frombuffer(data, dtype=np.uint8).reshape(
            header["height"], header["width"], 3
        )
        return header, image

    def fetch_jpeg(self):
        header, image = self.fetch_frame(self.rgb_url)
        output = io.BytesIO()
        Image.fromarray(image).save(output, format="JPEG", quality=self.jpeg_quality)
        return header, output.getvalue()

    def preprocess(self, image):
        source_height, source_width = image.shape[:2]
        scale = min(self.input_width / source_width, self.input_height / source_height)
        resized_width = max(1, round(source_width * scale))
        resized_height = max(1, round(source_height * scale))
        if (resized_width, resized_height) == (source_width, source_height):
            resized = image
        else:
            resized = np.asarray(
                Image.fromarray(image).resize(
                    (resized_width, resized_height), Image.Resampling.BILINEAR
                )
            )
        canvas = np.full((self.input_height, self.input_width, 3), 114, dtype=np.uint8)
        offset_x = (self.input_width - resized_width) // 2
        offset_y = (self.input_height - resized_height) // 2
        canvas[offset_y : offset_y + resized_height, offset_x : offset_x + resized_width] = resized
        tensor = canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        return tensor, scale, offset_x, offset_y

    def postprocess(self, output, source_width, source_height, scale, offset_x, offset_y):
        predictions = np.squeeze(output)
        if predictions.ndim != 2:
            return []
        # YOLO26 end-to-end ONNX output is already decoded and NMS-filtered:
        # [x1, y1, x2, y2, confidence, class_id].
        if predictions.shape[1] == 6:
            scores = predictions[:, 4]
            keep = scores >= self.confidence
            predictions, scores = predictions[keep], scores[keep]
            class_ids = predictions[:, 5].astype(np.int64)
            if not len(scores):
                return []
            boxes = predictions[:, :4].astype(np.float32, copy=True)
            boxes[:, [0, 2]] = (boxes[:, [0, 2]] - offset_x) / scale
            boxes[:, [1, 3]] = (boxes[:, [1, 3]] - offset_y) / scale
            boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, source_width)
            boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, source_height)
            selected = np.argsort(scores)[::-1][:30]
            return [
                {
                    "class_id": int(class_ids[index]),
                    "label": COCO_NAMES[int(class_ids[index])],
                    "confidence": round(float(scores[index]), 3),
                    "box": [round(float(value), 1) for value in boxes[index]],
                }
                for index in selected
                if 0 <= int(class_ids[index]) < len(COCO_NAMES)
            ]
        if predictions.shape[0] in (84, 85) and predictions.shape[1] > predictions.shape[0]:
            predictions = predictions.T
        if predictions.shape[1] == 85:
            class_scores = predictions[:, 5:] * predictions[:, 4:5]
        else:
            class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(len(class_ids)), class_ids]
        keep = scores >= self.confidence
        predictions, class_ids, scores = predictions[keep], class_ids[keep], scores[keep]
        if not len(scores):
            return []
        centers = predictions[:, :4]
        boxes = np.empty((len(centers), 4), dtype=np.float32)
        boxes[:, 0] = (centers[:, 0] - centers[:, 2] / 2 - offset_x) / scale
        boxes[:, 1] = (centers[:, 1] - centers[:, 3] / 2 - offset_y) / scale
        boxes[:, 2] = (centers[:, 0] + centers[:, 2] / 2 - offset_x) / scale
        boxes[:, 3] = (centers[:, 1] + centers[:, 3] / 2 - offset_y) / scale
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, source_width)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, source_height)
        selected = []
        for class_id in np.unique(class_ids):
            indices = np.where(class_ids == class_id)[0]
            indices = indices[np.argsort(scores[indices])[::-1]]
            while len(indices):
                current = indices[0]
                selected.append(current)
                if len(indices) == 1:
                    break
                indices = indices[1:][iou_one(boxes[current], boxes[indices[1:]]) < 0.45]
        selected = sorted(selected, key=lambda index: scores[index], reverse=True)[:30]
        return [
            {
                "class_id": int(class_ids[index]),
                "label": COCO_NAMES[int(class_ids[index])],
                "confidence": round(float(scores[index]), 3),
                "box": [round(float(value), 1) for value in boxes[index]],
            }
            for index in selected
            if int(class_ids[index]) < len(COCO_NAMES)
        ]

    def run_once(self):
        cycle_started = time.monotonic()
        header, image = self.fetch_frame(self.rgb_url)
        fetched_at = time.monotonic()
        tensor, scale, offset_x, offset_y = self.preprocess(image)
        preprocessed_at = time.monotonic()
        output = self.session.run(None, {self.input.name: tensor})[0]
        inferred_at = time.monotonic()
        detections = self.postprocess(
            output, header["width"], header["height"], scale, offset_x, offset_y
        )
        completed_at = time.monotonic()
        with self.lock:
            self.result = {
                "online": True,
                "detections": detections,
                "fetch_ms": round((fetched_at - cycle_started) * 1000),
                "preprocess_ms": round((preprocessed_at - fetched_at) * 1000),
                "inference_ms": round((inferred_at - preprocessed_at) * 1000),
                "postprocess_ms": round((completed_at - inferred_at) * 1000),
                "cycle_ms": round((completed_at - cycle_started) * 1000),
                "updated_at": time.time(),
                "source_width": header["width"],
                "source_height": header["height"],
                "input_width": self.input_width,
                "input_height": self.input_height,
                "error": None,
            }

    def run_forever(self):
        while True:
            started = time.monotonic()
            try:
                self.run_once()
            except Exception as error:
                with self.lock:
                    self.result = {
                        "online": False,
                        "detections": [],
                        "inference_ms": None,
                        "updated_at": time.time(),
                        "error": str(error)[:160],
                    }
            time.sleep(max(0.05, self.interval - (time.monotonic() - started)))

    def snapshot(self):
        with self.lock:
            result = dict(self.result)
        updated_at = result.get("updated_at")
        if updated_at and time.time() - updated_at > max(5.0, self.interval * 4):
            result["online"] = False
            result["error"] = "detector result is stale"
        return result


DETECTOR = None


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/view", "/view/"):
            self._serve_view()
            return
        if path == "/stream.mjpg":
            self._serve_stream()
            return
        if path == "/frame.jpg":
            self._serve_frame()
            return
        if path != "/detections":
            self.send_error(404)
            return
        self._serve_detections()

    def _serve_view(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(VIEW_HTML)))
        self.end_headers()
        self.wfile.write(VIEW_HTML)

    def _serve_detections(self):
        body = json.dumps(DETECTOR.snapshot(), separators=(",", ":")).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_frame(self):
        header, jpeg = DETECTOR.fetch_jpeg()
        sequence = header.get("sequence", header.get("seq", ""))
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("X-Sequence", str(sequence))
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Connection", "close")
        self.end_headers()
        frame_period = 1.0 / DETECTOR.stream_fps
        try:
            while True:
                started = time.monotonic()
                header, jpeg = DETECTOR.fetch_jpeg()
                sequence = header.get("sequence", header.get("seq", ""))
                part = (
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\nX-Sequence: {sequence}\r\n\r\n".encode()
                    + jpeg
                    + b"\r\n"
                )
                self.wfile.write(part)
                self.wfile.flush()
                time.sleep(max(0.0, frame_period - (time.monotonic() - started)))
        except (
            BrokenPipeError,
            ConnectionAbortedError,
            ConnectionResetError,
            TimeoutError,
            urllib.error.URLError,
            ValueError,
        ):
            return

    def log_message(self, fmt, *args):
        return


def main():
    global DETECTOR
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--rgb-url", default="http://127.0.0.1:8080/api/camera/rgb/frame")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--interval", type=float, default=0.8)
    parser.add_argument("--input-size", type=int, default=320)
    parser.add_argument("--stream-fps", type=float, default=10.0)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    args = parser.parse_args()
    if args.input_size < 32 or args.input_size % 32:
        parser.error("--input-size must be a multiple of 32 and at least 32")
    if not 1 <= args.stream_fps <= 30:
        parser.error("--stream-fps must be between 1 and 30")
    if not 40 <= args.jpeg_quality <= 95:
        parser.error("--jpeg-quality must be between 40 and 95")
    DETECTOR = Detector(
        args.model,
        args.rgb_url,
        args.confidence,
        args.interval,
        args.input_size,
        args.stream_fps,
        args.jpeg_quality,
    )
    threading.Thread(target=DETECTOR.run_forever, daemon=True).start()
    print(f"YOLO detector listening on http://0.0.0.0:{args.port}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()

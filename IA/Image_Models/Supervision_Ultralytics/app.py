# min 1:04.00h -> https://www.youtube.com/watch?v=RvVLqXdhdVQ
import cv2
import supervision as sv
import argparse
import numpy as np
from ultralytics import YOLO

POLYGON = np.array([
    [171.4431818181818, 2.4797077922077837],
    [161.02110389610388, 107.74269480519479],
    [0.5211038961038961, 304.7199675324675],
    [0.5211038961038961, 362.0413961038961],
    [559.6655844155844, 360.9991883116883],
    [588.8474025974026, 247.91964285714283],
    [640.4366883116883, 198.9358766233766],
    [641.4788961038961, 109.82711038961037],
    [403.3344155844156, 186.950487012987],
    [379.8847402597402, 179.13392857142856],
    [317.8733766233766, 76.47646103896102],
    [232.93344155844153, 3.0008116883116798]
], dtype=np.int32)
CLASSES = [2,3]

model = YOLO("yolo11n.pt")
tracker = sv.ByteTrack(minimum_consecutive_frames=3)
tracker.reset()

polygon_zone = sv.PolygonZone(polygon=POLYGON, triggering_anchors=(sv.Position.CENTER,))
box_annotator = sv.BoxAnnotator()
label_annotator = sv.LabelAnnotator(text_color=sv.Color.BLACK)
trace_annotator = sv.TraceAnnotator(trace_length=15)

def main(video_file_path):
    frame_generator = sv.get_video_frames_generator(source_path=video_file_path)
    for i, frame in enumerate(frame_generator):
        result = model(frame, device="cpu", verbose=False, imgsz=1280)[0] #change to cuda
        
        detections = sv.Detections.from_ultralytics(result)
        detections = detections[polygon_zone.trigger(detections)]
        detections = detections[np.isin(detections.class_id, CLASSES)]
        detections = tracker.update_with_detections(detections)
        
        labels = [
            f"#{tracker_id}"
            for tracker_id in detections.tracker_id
        ]
        
        annotated_frame = frame.copy()
        annotated_frame = sv.draw_polygon(
            scene=annotated_frame,
            polygon=POLYGON,
            color=sv.Color.RED, 
            thickness=2
        )
        annotated_frame = box_annotator.annotate(
            scene=annotated_frame,
            detections=detections
        )
        annotated_frame = label_annotator.annotate(
            scene=annotated_frame,
            detections=detections,
            labels=labels
        )
        annotated_frame = trace_annotator.annotate(
            scene=annotated_frame,
            detections=detections
        )
        
        cv2.imshow("Processed video", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--video_file_path")
    
    args = parser.parse_args()
    
    main(args.video_file_path)
    
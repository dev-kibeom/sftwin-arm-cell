string snapshot_id
uint64 revision
uint8 event_kind
uint8 CALIBRATION_COMPLETE=0
uint8 DETECT_TARGET_PROGRESS=1
uint8 DETECT_TARGET_SUCCESS=2
uint8 DETECT_TARGET_FAILURE=3
uint8 CALIBRATION_PROGRESS=4
string vision_run_id
string calibration_id
string acquisition_id
sensor_msgs/CompressedImage image_png
string metadata_json
# Independent per-Acquisition diagnostic payload for Hub presentation; empty outside terminal DetectTarget events.
string diagnostic_record_json

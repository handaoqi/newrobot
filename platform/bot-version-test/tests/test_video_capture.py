from bike_bot.config import VideoConfig
from bike_bot.detector import build_gstreamer_rtsp_pipeline


def test_gstreamer_pipeline_uses_jetson_decoder_and_latest_frame_sink():
    config = VideoConfig(
        source="rtsp://camera.example/test",
        width=1280,
        height=720,
        rtsp_transport="tcp",
        hardware_decode_drop_frame_interval=3,
    )

    pipeline = build_gstreamer_rtsp_pipeline(config)

    assert "rtph264depay" in pipeline
    assert "nvv4l2decoder" in pipeline
    assert "drop-frame-interval=3" in pipeline
    assert "appsink drop=true max-buffers=1 sync=false" in pipeline


def test_gstreamer_pipeline_supports_h265():
    config = VideoConfig(source="rtsp://camera.example/test", rtsp_codec="h265")

    pipeline = build_gstreamer_rtsp_pipeline(config)

    assert "rtph265depay" in pipeline
    assert "h265parse" in pipeline

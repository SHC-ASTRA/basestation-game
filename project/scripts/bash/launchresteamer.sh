# ffmpeg -i "http://localhost:1984/api/stream.mp4?src=Cams&mp4=all" -c:v libtheora stream.ogv

GST_DEBUG=4 GST_PLUGIN_PATH=$(gst-inspect-1.0 --print-plugin-path) gst-launch-1.0 -e uridecodebin uri="rtsp://192.168.1.12:554/user=admin&password=&channel=1&stream=0.sdp?" ! \
videoconvert ! theoraenc ! oggmux ! \
multifilesink location=video_segment_%03d.ogg

ffmpeg -re -stream_loop -1 -i ./scripts/bash/streamservertest/6603134-hd_1920_1080_25fps.mov -c copy -f rtsp rtsp://localhost:8554/mystream # spawn a teststream

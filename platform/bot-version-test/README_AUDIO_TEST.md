# Remote audio test copy

Isolated from production `bot-version`. It starts only the cloud `play_audio` poller.

Check: `/usr/bin/python3 run_audio_test.py --config config.audio-test.yaml --check`

Poll once: `/usr/bin/python3 run_audio_test.py --config config.audio-test.yaml --once`

Continuous foreground run: `./start_audio_test.sh`

Log: `data/logs/audio-test.log`. No systemd service is installed.

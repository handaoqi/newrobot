#!/usr/bin/env python3
"""Deprecated local HTTP mapping API.

The real mapping path is:
  cloud mapping.start -> Edge MappingAdapter -> roamerx-mapping.service
  or /home/dogrobot/robot/script/robot/start_mapping_real.sh
"""

import sys

sys.stderr.write(
    "map_api_server.py is deprecated and disabled.\n"
    "Use /home/dogrobot/robot/script/robot/start_mapping_real.sh "
    "or the cloud mapping.start command.\n"
)
sys.exit(2)

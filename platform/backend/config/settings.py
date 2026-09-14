import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent

ASR_ENABLED = os.getenv("ASR_ENABLED", "true").lower() == "true"
ASR_MODEL_PATH = os.getenv("ASR_MODEL_PATH", "/opt/roamerx/shared/models/faster-whisper-base")
ASR_LANGUAGE = os.getenv("ASR_LANGUAGE", "zh")
INSPECTION_SPEECH_CATEGORY_NAME = os.getenv("INSPECTION_SPEECH_CATEGORY_NAME", "巡检智能播报")
ENABLE_DEMO_SEED = os.getenv("ENABLE_DEMO_SEED", "false").lower() == "true"

SECRET_KEY = os.getenv("DJANGO_SECRET_KEY", "django-insecure-demo-inspection-platform-key")
DEBUG = os.getenv("DJANGO_DEBUG", "true").lower() == "true"
ALLOWED_HOSTS = [
    item.strip()
    for item in os.getenv(
        "DJANGO_ALLOWED_HOSTS",
        "127.0.0.1,localhost,testserver,192.168.234.8,192.168.234.12,192.168.234.14,192.168.234.16",
    ).split(",")
    if item.strip()
]

DEFAULT_ROBOT_CONTROL_ENDPOINT = ""
ROBOT_CONTROL_ENDPOINTS = {
    "ZSL-1A-07": "http://192.168.234.1:9100/commands",
}

INSTALLED_APPS = [
    "django.contrib.admin",
    "django.contrib.auth",
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "django.contrib.messages",
    "django.contrib.staticfiles",
    "django.contrib.humanize",
    "rest_framework",
    "rest_framework.authtoken",
    "corsheaders",
    "monitoring.apps.MonitoringConfig",
]

MIDDLEWARE = [
    "corsheaders.middleware.CorsMiddleware",
    "django.middleware.security.SecurityMiddleware",
    "django.contrib.sessions.middleware.SessionMiddleware",
    "django.middleware.common.CommonMiddleware",
    "django.middleware.csrf.CsrfViewMiddleware",
    "django.contrib.auth.middleware.AuthenticationMiddleware",
    "django.contrib.messages.middleware.MessageMiddleware",
    "django.middleware.clickjacking.XFrameOptionsMiddleware",
]

ROOT_URLCONF = "config.urls"

TEMPLATES = [
    {
        "BACKEND": "django.template.backends.django.DjangoTemplates",
        "DIRS": [],
        "APP_DIRS": True,
        "OPTIONS": {
            "context_processors": [
                "django.template.context_processors.request",
                "django.contrib.auth.context_processors.auth",
                "django.contrib.messages.context_processors.messages",
            ],
        },
    },
]

WSGI_APPLICATION = "config.wsgi.application"
ASGI_APPLICATION = "config.asgi.application"

if os.getenv("POSTGRES_DB"):
    DATABASES = {
        "default": {
            "ENGINE": "django.db.backends.postgresql",
            "NAME": os.environ["POSTGRES_DB"],
            "USER": os.getenv("POSTGRES_USER", ""),
            "PASSWORD": os.getenv("POSTGRES_PASSWORD", ""),
            "HOST": os.getenv("POSTGRES_HOST", "127.0.0.1"),
            "PORT": os.getenv("POSTGRES_PORT", "5432"),
            "CONN_MAX_AGE": 60,
        }
    }
else:
    DATABASES = {
        "default": {
            "ENGINE": "django.db.backends.sqlite3",
            "NAME": os.getenv("SQLITE_DB_PATH", BASE_DIR / "db.sqlite3"),
            # The center receives frequent telemetry writes concurrently with
            # task dispatch and scheduler updates.  Give a SQLite writer time
            # to acquire the lock instead of failing a robot command at the
            # default five-second timeout.
            "OPTIONS": {
                "timeout": max(1, int(os.getenv("SQLITE_BUSY_TIMEOUT_SECONDS", "30"))),
                "transaction_mode": "IMMEDIATE",
            },
            "CONN_MAX_AGE": int(os.getenv("SQLITE_CONN_MAX_AGE", "60")),
        }
    }

AUTH_PASSWORD_VALIDATORS = []

LANGUAGE_CODE = "zh-hans"
TIME_ZONE = "Asia/Shanghai"
USE_I18N = True
USE_TZ = True

STATIC_URL = "static/"
MEDIA_URL = "media/"
MEDIA_ROOT = Path(os.getenv("DJANGO_MEDIA_ROOT", BASE_DIR / "media"))
PUBLIC_BASE_URL = os.getenv("PUBLIC_BASE_URL", "").rstrip("/")
TTS_VOICE = os.getenv("TTS_VOICE", "zh-CN-XiaoxiaoNeural")
TTS_RATE = os.getenv("TTS_RATE", "-5%")
TTS_VOLUME = os.getenv("TTS_VOLUME", "+0%")
BICYCLE_AUTO_SPEECH_ENABLED = os.getenv("BICYCLE_AUTO_SPEECH_ENABLED", "true").lower() == "true"
BICYCLE_AUTO_SPEECH_TEMPLATE_NAME = os.getenv("BICYCLE_AUTO_SPEECH_TEMPLATE_NAME", "驶离提醒")
# One bicycle incident is confirmed on the Edge after three consecutive frames.
# Keep the cloud-side record and speech gate on the same 10 second window so a
# retry, restart, or alternate ingress cannot turn that one incident into a
# burst of operator alerts.
BICYCLE_ALERT_COOLDOWN_SECONDS = int(os.getenv("BICYCLE_ALERT_COOLDOWN_SECONDS", "10"))
BICYCLE_AUTO_SPEECH_COOLDOWN_SECONDS = int(
    os.getenv("BICYCLE_AUTO_SPEECH_COOLDOWN_SECONDS", str(BICYCLE_ALERT_COOLDOWN_SECONDS))
)
SECURE_PROXY_SSL_HEADER = ("HTTP_X_FORWARDED_PROTO", "https")
CSRF_TRUSTED_ORIGINS = [
    item.strip()
    for item in os.getenv("DJANGO_CSRF_TRUSTED_ORIGINS", "https://39.107.250.69").split(",")
    if item.strip()
]
DEFAULT_AUTO_FIELD = "django.db.models.BigAutoField"

CORS_ALLOW_ALL_ORIGINS = True

MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "8883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_CA_FILE = os.getenv("MQTT_CA_FILE", "")
MQTT_TLS_ENABLED = os.getenv("MQTT_TLS_ENABLED", "true").lower() == "true"
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", "roamerx-platform-worker")
MQTT_KEEPALIVE_SECONDS = int(os.getenv("MQTT_KEEPALIVE_SECONDS", "20"))
DEVICE_OFFLINE_TIMEOUT_SECONDS = int(os.getenv("DEVICE_OFFLINE_TIMEOUT_SECONDS", "30"))
COMMAND_ACK_TIMEOUT_SECONDS = int(os.getenv("COMMAND_ACK_TIMEOUT_SECONDS", "5"))
# task.start remains valid for the whole patrol, but delivery acknowledgement
# must arrive promptly or the active execution blocks every later loop round.
TASK_START_ACK_TIMEOUT_SECONDS = int(os.getenv("TASK_START_ACK_TIMEOUT_SECONDS", "60"))
COMMAND_RESULT_TIMEOUT_SECONDS = int(os.getenv("COMMAND_RESULT_TIMEOUT_SECONDS", "1800"))
COMMAND_START_EXPIRY_SECONDS = int(os.getenv("COMMAND_START_EXPIRY_SECONDS", "30"))
COMMAND_CONTROL_EXPIRY_SECONDS = int(os.getenv("COMMAND_CONTROL_EXPIRY_SECONDS", "15"))
TASK_MAX_DURATION_SECONDS = int(os.getenv("TASK_MAX_DURATION_SECONDS", "1800"))
TASK_RECOVERY_EXPIRY_SECONDS = int(os.getenv("TASK_RECOVERY_EXPIRY_SECONDS", "300"))
PATROL_LOOP_SELF_HEAL_ENABLED = os.getenv("PATROL_LOOP_SELF_HEAL_ENABLED", "true").lower() == "true"
LOW_BATTERY_STOP_PERCENT = int(os.getenv("LOW_BATTERY_STOP_PERCENT", "20"))
LOW_BATTERY_REARM_PERCENT = int(os.getenv("LOW_BATTERY_REARM_PERCENT", "25"))

# Raw MQTT packets exist for short-term de-duplication and troubleshooting;
# their business results are persisted separately.  Keep terminal packet data
# bounded so a high-rate robot cannot exhaust the SQLite volume.
# The legacy all-terminal window remains available for one-off management
# command overrides. Scheduled cleanup uses the type-specific windows below.
INBOUND_MESSAGE_RETENTION_DAYS = int(os.getenv("INBOUND_MESSAGE_RETENTION_DAYS", "30"))
INBOUND_MESSAGE_TELEMETRY_RETENTION_DAYS = int(
    os.getenv("INBOUND_MESSAGE_TELEMETRY_RETENTION_DAYS", "3")
)
INBOUND_MESSAGE_OPERATIONAL_RETENTION_DAYS = int(
    os.getenv("INBOUND_MESSAGE_OPERATIONAL_RETENTION_DAYS", "30")
)
INBOUND_MESSAGE_FAILED_RETENTION_DAYS = int(os.getenv("INBOUND_MESSAGE_FAILED_RETENTION_DAYS", "180"))
INBOUND_TELEMETRY_FULL_PAYLOAD_SAMPLE_EVERY = max(
    0, int(os.getenv("INBOUND_TELEMETRY_FULL_PAYLOAD_SAMPLE_EVERY", "150"))
)
ROBOT_TELEMETRY_RETENTION_DAYS = max(1, int(os.getenv("ROBOT_TELEMETRY_RETENTION_DAYS", "14")))
ROBOT_TELEMETRY_CLEANUP_DAYS_PER_RUN = max(
    1, int(os.getenv("ROBOT_TELEMETRY_CLEANUP_DAYS_PER_RUN", "1"))
)
ROBOT_TELEMETRY_WEEKLY_CLEANUP_MAX_DAYS = max(
    1, int(os.getenv("ROBOT_TELEMETRY_WEEKLY_CLEANUP_MAX_DAYS", "10000"))
)
ROBOT_TELEMETRY_WEEKLY_CLEANUP_TIME_BUDGET_SECONDS = max(
    30, int(os.getenv("ROBOT_TELEMETRY_WEEKLY_CLEANUP_TIME_BUDGET_SECONDS", "600"))
)
SYSTEM_LOG_DEBUG_RETENTION_DAYS = int(os.getenv("SYSTEM_LOG_DEBUG_RETENTION_DAYS", "7"))
SYSTEM_LOG_INFO_RETENTION_DAYS = int(os.getenv("SYSTEM_LOG_INFO_RETENTION_DAYS", "30"))
SYSTEM_LOG_WARNING_ERROR_RETENTION_DAYS = int(os.getenv("SYSTEM_LOG_WARNING_ERROR_RETENTION_DAYS", "180"))
SYSTEM_LOG_CLEANUP_BATCH_SIZE = max(1, int(os.getenv("SYSTEM_LOG_CLEANUP_BATCH_SIZE", "500")))
INBOUND_MESSAGE_CLEANUP_BATCH_SIZE = max(1, int(os.getenv("INBOUND_MESSAGE_CLEANUP_BATCH_SIZE", "500")))
INBOUND_MESSAGE_CLEANUP_INTERVAL_SECONDS = max(
    60, int(os.getenv("INBOUND_MESSAGE_CLEANUP_INTERVAL_SECONDS", "3600"))
)
INBOUND_MESSAGE_STALE_PENDING_DAYS = max(0, int(os.getenv("INBOUND_MESSAGE_STALE_PENDING_DAYS", "7")))
INBOUND_MESSAGE_WEEKLY_CLEANUP_ENABLED = os.getenv("INBOUND_MESSAGE_WEEKLY_CLEANUP_ENABLED", "true").lower() == "true"
INBOUND_MESSAGE_WEEKLY_CLEANUP_WEEKDAY = int(os.getenv("INBOUND_MESSAGE_WEEKLY_CLEANUP_WEEKDAY", "0")) % 7
INBOUND_MESSAGE_WEEKLY_CLEANUP_HOUR = min(23, max(0, int(os.getenv("INBOUND_MESSAGE_WEEKLY_CLEANUP_HOUR", "3"))))
INBOUND_MESSAGE_WEEKLY_CLEANUP_GRACE_HOURS = max(
    1, int(os.getenv("INBOUND_MESSAGE_WEEKLY_CLEANUP_GRACE_HOURS", "24"))
)
INBOUND_MESSAGE_WEEKLY_CLEANUP_MAX_BATCHES = max(
    1, int(os.getenv("INBOUND_MESSAGE_WEEKLY_CLEANUP_MAX_BATCHES", "10000"))
)
INBOUND_MESSAGE_WEEKLY_CLEANUP_TIME_BUDGET_SECONDS = max(
    30, int(os.getenv("INBOUND_MESSAGE_WEEKLY_CLEANUP_TIME_BUDGET_SECONDS", "600"))
)
SQLITE_LOCK_RETRY_ATTEMPTS = max(1, int(os.getenv("SQLITE_LOCK_RETRY_ATTEMPTS", "4")))
SQLITE_LOCK_RETRY_BACKOFF_SECONDS = max(
    0.0, float(os.getenv("SQLITE_LOCK_RETRY_BACKOFF_SECONDS", "0.25"))
)

# Validation recordings and results are large immutable blobs. Production uses
# the self-hosted S3-compatible service from runtime/platform/compose.yaml;
# tests and a developer checkout can use the local backend without credentials.
VALIDATION_OBJECT_STORE_BACKEND = os.getenv("VALIDATION_OBJECT_STORE_BACKEND", "local").lower()
VALIDATION_OBJECT_STORE_ENDPOINT = os.getenv("VALIDATION_OBJECT_STORE_ENDPOINT", "")
VALIDATION_OBJECT_STORE_PUBLIC_ENDPOINT = os.getenv("VALIDATION_OBJECT_STORE_PUBLIC_ENDPOINT", "")
VALIDATION_OBJECT_STORE_BUCKET = os.getenv("VALIDATION_OBJECT_STORE_BUCKET", "roamerx-validation")
VALIDATION_OBJECT_STORE_ACCESS_KEY = os.getenv("VALIDATION_OBJECT_STORE_ACCESS_KEY", "")
VALIDATION_OBJECT_STORE_SECRET_KEY = os.getenv("VALIDATION_OBJECT_STORE_SECRET_KEY", "")
VALIDATION_OBJECT_STORE_REGION = os.getenv("VALIDATION_OBJECT_STORE_REGION", "us-east-1")
VALIDATION_OBJECT_STORE_ROOT = Path(
    os.getenv("VALIDATION_OBJECT_STORE_ROOT", MEDIA_ROOT / "validation-objects")
)
VALIDATION_SIGNED_URL_TTL_SECONDS = int(os.getenv("VALIDATION_SIGNED_URL_TTL_SECONDS", "900"))
VALIDATION_RUNNER_TOKEN = os.getenv("VALIDATION_RUNNER_TOKEN", "")
VALIDATION_GATEWAY_TOKEN = os.getenv("VALIDATION_GATEWAY_TOKEN", "")
VALIDATION_LIVE_TICKET_TTL_SECONDS = int(os.getenv("VALIDATION_LIVE_TICKET_TTL_SECONDS", "60"))
VALIDATION_RUNNER_LEASE_SECONDS = int(os.getenv("VALIDATION_RUNNER_LEASE_SECONDS", "45"))
VALIDATION_MAX_ATTEMPTS = int(os.getenv("VALIDATION_MAX_ATTEMPTS", "2"))

REST_FRAMEWORK = {
    "DEFAULT_AUTHENTICATION_CLASSES": [
        "rest_framework.authentication.TokenAuthentication",
    ],
    "DEFAULT_PERMISSION_CLASSES": [
        "rest_framework.permissions.IsAuthenticated",
    ],
}

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
    "monitoring",
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
            "OPTIONS": {"timeout": 30},
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
BICYCLE_AUTO_SPEECH_COOLDOWN_SECONDS = int(os.getenv("BICYCLE_AUTO_SPEECH_COOLDOWN_SECONDS", "30"))
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
COMMAND_RESULT_TIMEOUT_SECONDS = int(os.getenv("COMMAND_RESULT_TIMEOUT_SECONDS", "1800"))
COMMAND_START_EXPIRY_SECONDS = int(os.getenv("COMMAND_START_EXPIRY_SECONDS", "30"))
COMMAND_CONTROL_EXPIRY_SECONDS = int(os.getenv("COMMAND_CONTROL_EXPIRY_SECONDS", "15"))
TASK_MAX_DURATION_SECONDS = int(os.getenv("TASK_MAX_DURATION_SECONDS", "1800"))

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

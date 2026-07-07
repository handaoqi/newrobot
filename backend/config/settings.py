import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent

SECRET_KEY = os.getenv("DJANGO_SECRET_KEY", "django-insecure-demo-inspection-platform-key")
DEBUG = os.getenv("DJANGO_DEBUG", "true").lower() == "true"
ALLOWED_HOSTS = [
    item.strip()
    for item in os.getenv(
        "DJANGO_ALLOWED_HOSTS",
        "127.0.0.1,localhost,testserver,192.168.234.8,192.168.234.12,192.168.234.14",
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

REST_FRAMEWORK = {
    "DEFAULT_AUTHENTICATION_CLASSES": [
        "rest_framework.authentication.TokenAuthentication",
    ],
    "DEFAULT_PERMISSION_CLASSES": [
        "rest_framework.permissions.IsAuthenticated",
    ],
}

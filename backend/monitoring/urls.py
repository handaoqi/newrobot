from django.urls import path

from .views import (
    DashboardAnalyticsView,
    DashboardOverviewView,
    EventDetailView,
    EventHandleView,
    EventListView,
    LoginView,
    LogoutView,
    ProfileView,
    RobotDetailView,
    RobotListView,
    TaskListView,
    TelemetryIngestView,
    health_check,
)

urlpatterns = [
    path("health/", health_check),
    path("auth/login/", LoginView.as_view()),
    path("auth/logout/", LogoutView.as_view()),
    path("auth/profile/", ProfileView.as_view()),
    path("dashboard/overview/", DashboardOverviewView.as_view()),
    path("dashboard/analytics/", DashboardAnalyticsView.as_view()),
    path("robots/", RobotListView.as_view()),
    path("robots/<int:robot_id>/", RobotDetailView.as_view()),
    path("events/", EventListView.as_view()),
    path("events/<int:event_id>/", EventDetailView.as_view()),
    path("events/<int:event_id>/handle/", EventHandleView.as_view()),
    path("tasks/", TaskListView.as_view()),
    path("telemetry/ingest/", TelemetryIngestView.as_view()),
]

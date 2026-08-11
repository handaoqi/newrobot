from django.urls import path

from .dev_views import (
    DevelopmentAgentListView,
    DevelopmentTaskCancelView,
    DevelopmentTaskDetailView,
    DevelopmentTaskListCreateView,
    development_task_stream,
)


urlpatterns = [
    path("agents/", DevelopmentAgentListView.as_view()),
    path("tasks/", DevelopmentTaskListCreateView.as_view()),
    path("tasks/<uuid:task_id>/", DevelopmentTaskDetailView.as_view()),
    path("tasks/<uuid:task_id>/cancel/", DevelopmentTaskCancelView.as_view()),
    path("tasks/<uuid:task_id>/stream/", development_task_stream),
]


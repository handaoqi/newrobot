from django.urls import path

from .dev_views import (
    DevelopmentAgentListView,
    DevelopmentConversationDetailView,
    DevelopmentTaskCancelView,
    DevelopmentTaskDetailView,
    DevelopmentTaskListCreateView,
    VoiceRecognitionListView,
    development_task_stream,
)


urlpatterns = [
    path("agents/", DevelopmentAgentListView.as_view()),
    path("voice-recognitions/", VoiceRecognitionListView.as_view()),
    path("conversations/main/", DevelopmentConversationDetailView.as_view()),
    path("tasks/", DevelopmentTaskListCreateView.as_view()),
    path("tasks/<uuid:task_id>/", DevelopmentTaskDetailView.as_view()),
    path("tasks/<uuid:task_id>/cancel/", DevelopmentTaskCancelView.as_view()),
    path("tasks/<uuid:task_id>/stream/", development_task_stream),
]

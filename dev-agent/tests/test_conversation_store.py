from pathlib import Path

from roamerx_dev_agent.conversation_store import ConversationStore


def test_conversation_thread_persists(tmp_path: Path):
    path = tmp_path / "state" / "conversation.json"
    store = ConversationStore(str(path))
    assert store.thread_id() is None
    store.save("thread-123")
    assert ConversationStore(str(path)).thread_id() == "thread-123"

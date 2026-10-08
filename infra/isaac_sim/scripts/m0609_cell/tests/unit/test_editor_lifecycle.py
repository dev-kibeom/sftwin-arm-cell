from types import SimpleNamespace

from pnp_validation.editor_lifecycle import ValidationSessionOwner


class _Subscription:
    def __init__(self, callback):
        self.callback = callback
        self.unsubscribe_calls = 0

    def unsubscribe(self):
        self.unsubscribe_calls += 1


class _EventStream:
    def __init__(self):
        self.subscriptions = []

    def create_subscription_to_pop(self, callback):
        subscription = _Subscription(callback)
        self.subscriptions.append(subscription)
        return subscription

    def emit(self, event):
        for subscription in tuple(self.subscriptions):
            if subscription.unsubscribe_calls == 0:
                subscription.callback(event)


class _Timeline:
    def __init__(self):
        self.stopped = False
        self.events = _EventStream()

    def get_timeline_event_stream(self):
        return self.events

    def is_stopped(self):
        return self.stopped


def test_pause_does_not_teardown_and_stop_tears_down_after_stopped():
    timeline = _Timeline()
    calls = []
    coordinator = ValidationSessionOwner(timeline.events, stopped_event_type="STOP")
    coordinator.register_cleanup("validation", lambda: calls.append("cleanup"))
    coordinator.start()

    timeline.events.emit(SimpleNamespace(type="PAUSE"))
    assert calls == []

    timeline.events.emit(SimpleNamespace(type="STOP"))
    assert calls == ["cleanup"]


def test_repeated_stop_teardown_is_idempotent_and_keeps_stop_watcher():
    timeline = _Timeline()
    calls = []
    coordinator = ValidationSessionOwner(timeline.events, stopped_event_type="STOP")
    coordinator.register_cleanup("validation", lambda: calls.append("cleanup"))
    coordinator.start()

    timeline.stopped = True
    timeline.events.emit(SimpleNamespace(type="STOP"))
    timeline.events.emit(SimpleNamespace(type="STOP"))

    assert calls == ["cleanup"]
    assert len(timeline.events.subscriptions) == 1
    assert timeline.events.subscriptions[0].unsubscribe_calls == 0


def test_false_cleanup_result_propagates_as_teardown_failure():
    coordinator = ValidationSessionOwner(_EventStream(), stopped_event_type="STOP")
    coordinator.register_cleanup("failed-cleanup", lambda: False)
    coordinator.start()

    assert coordinator.teardown() is False


def test_stop_teardown_diagnostic_is_emitted_once(capsys):
    coordinator = ValidationSessionOwner(_EventStream(), stopped_event_type="STOP")

    assert coordinator.teardown() is True
    assert coordinator.teardown() is True
    output = capsys.readouterr().out

    assert output.count("VALIDATION_TEARDOWN ") == 1
    assert "result=SUCCESS joint=0 fixture=0 callbacks=0 artifacts=0" in output


def test_stop_keeps_runtime_session_until_after_play_rerun():
    class Session:
        def __init__(self):
            self.shutdown_calls = 0

        def shutdown(self):
            self.shutdown_calls += 1
            return True

    session = Session()
    owner = ValidationSessionOwner(_EventStream(), stopped_event_type="STOP")
    namespace = {"runtime_session": session}
    owner.bind_namespace(namespace)
    owner.register_session(session)
    owner.teardown()

    assert session.shutdown_calls == 0
    assert namespace["runtime_session"] is session

    owner.prepare_for_rerun()

    assert session.shutdown_calls == 1
    assert namespace["runtime_session"] is None

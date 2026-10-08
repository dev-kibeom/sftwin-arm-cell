"""Single owner for Isaac validation state across Script Editor Play/Stop."""


class ValidationSessionOwner:
    """Own active runtime, validation state, callbacks, and teardown material."""

    ARTIFACT_KEYS = (
        "SFTWIN_PNP_REGISTRATION_MANIFEST",
        "SFTWIN_PNP_PLACE_EVENT_CONSUMED",
        "SFTWIN_PNP_PLACE_EVENT",
        "SFTWIN_PNP_MISSION_COMPLETE",
        "SFTWIN_PNP_PLACE_CONFIRMATION",
        "SFTWIN_PNP_DELETE_REQUEST",
        "SFTWIN_PNP_DELETE_ACK",
        "SFTWIN_PNP_PLANNING_SCENE_STATE",
    )
    RETAINED_STATE_NAMES = (
        "pnp_validation_run",
        "pnp_validation_handoff",
        "pnp_validation_grasp_manager",
        "pnp_validation_cleanup_worker",
        "pnp_validation_place_event_worker",
        "pnp_validation_mission_completion_worker",
        "pnp_validation_runtime",
        "pnp_validation_environment",
        "pnp_validation_session_owner",
    )

    def __init__(self, simulation_event_stream, stopped_event_type):
        self.simulation_event_stream = simulation_event_stream
        self.stopped_event_type = stopped_event_type
        self._namespace = None
        self._subscriptions = {}
        self._cleanup_callbacks = {}
        self._session = None
        self._runtime = None
        self._environment = None
        self._teardown_complete = False
        self._simulation_subscription = None

    def bind_namespace(self, namespace):
        self._namespace = namespace
        return self

    def register_session(self, session):
        self._session = session
        return session

    def register_validation(self, runtime, environment):
        self._clear_artifacts(self._environment)
        self._runtime = runtime
        self._environment = environment
        self._clear_artifacts(environment)
        self._teardown_complete = False
        return runtime

    def register_subscription(self, name, subscription):
        previous = self._subscriptions.get(name)
        if previous is not None:
            self._unsubscribe(previous)
        self._subscriptions[name] = subscription
        return subscription

    def register_cleanup(self, name, callback):
        self._cleanup_callbacks[name] = callback
        return callback

    def start(self):
        if self._simulation_subscription is None:
            self._simulation_subscription = (
                self.simulation_event_stream.create_subscription_to_pop(
                    self._on_simulation_event
                )
            )
        return self

    def prepare_for_rerun(self):
        """Release callbacks/runtime for after_play rebootstrap, preserving the fixture."""
        self._close_subscriptions()
        # STOP leaves the regular gripper runtime alive so Play can resume
        # without re-running after_play. A deliberate after_play rerun replaces
        # that session here, while it is safe to do so exactly once.
        if self._session is not None:
            self._session.shutdown()
        self._session = None
        if self._namespace is not None:
            for name in (
                "runtime_session",
                "grasp_manager",
                "gripper",
                "sim_adapter",
                "on_physics_step",
                "physics_sub",
            ):
                self._namespace[name] = None
        self._clear_registry()
        self._cleanup_callbacks.clear()
        self._clear_retained_state()
        self._teardown_complete = False

    def _on_simulation_event(self, event):
        event_type = getattr(event, "type", None)
        try:
            is_stopped = int(event_type) == int(self.stopped_event_type)
        except (TypeError, ValueError):
            is_stopped = event_type == self.stopped_event_type
        if is_stopped:
            self.teardown()

    def teardown(self):
        """Complete validation teardown once, after SimulationEvent.STOPPED."""
        if self._teardown_complete:
            return True
        self._teardown_complete = True
        result = True
        # Keep the regular gripper session and its physics-step subscription
        # across timeline STOP. They belong to the Kit app runtime and are
        # needed when Play resumes; after_play reruns replace them explicitly.
        session = self._session
        joint_remaining = (
            self._joint_remaining(session) if self._runtime is not None else False
        )
        if joint_remaining:
            result = False
        if self._runtime is not None:
            self._clear_registry()
        if not self._remove_owned_fixture():
            result = False
        fixture_remaining = self._fixture_remaining()
        if fixture_remaining:
            result = False
        if not self._clear_artifacts(self._environment):
            result = False
        for callback in tuple(self._cleanup_callbacks.values()):
            try:
                if callback() is False:
                    result = False
            except Exception as error:
                result = False
                print(f"[WU-14] validation cleanup callback failed: {error}")
        self._cleanup_callbacks.clear()
        for name, subscription in tuple(self._subscriptions.items()):
            if name != "runtime_physics":
                self._unsubscribe(subscription)
                self._subscriptions.pop(name, None)
        callbacks_remaining = bool(self._subscriptions)
        if callbacks_remaining:
            result = False
        self._clear_validation_state()
        if self._namespace is not None:
            self._namespace["pnp_validation_session_owner"] = self
        print(
            "VALIDATION_TEARDOWN "
            f"result={'SUCCESS' if result else 'FAILED'} "
            f"joint={int(joint_remaining)} "
            f"fixture={int(fixture_remaining)} "
            f"callbacks={int(callbacks_remaining)} "
            f"artifacts={int(self._artifacts_remaining())}"
        )
        return result

    def _close_subscriptions(self):
        for subscription in tuple(self._subscriptions.values()):
            self._unsubscribe(subscription)
        self._subscriptions.clear()
        if self._simulation_subscription is not None:
            self._unsubscribe(self._simulation_subscription)
            self._simulation_subscription = None

    @staticmethod
    def _unsubscribe(subscription):
        unsubscribe = getattr(subscription, "unsubscribe", None)
        if callable(unsubscribe):
            unsubscribe()

    def _clear_registry(self):
        registry = getattr(self._session, "registry", None)
        paths = getattr(registry, "paths", lambda: ())()
        unregister = getattr(registry, "unregister", None)
        if callable(unregister):
            for path in paths:
                unregister(path)

    @staticmethod
    def _joint_remaining(session):
        manager = getattr(session, "grasp_manager", None)
        return bool(getattr(manager, "attach_joint_path", None))

    def _remove_owned_fixture(self):
        runtime = self._runtime
        if runtime is None:
            return True
        owned_path = getattr(runtime, "_owned_path", None)
        if owned_path is None:
            canonical_path = "/World/SFTwin/ValidationFixture/Cube"
            prim = runtime.stage.GetPrimAtPath(canonical_path)
            if prim.IsValid() and runtime._is_owned(prim):
                try:
                    runtime.adopt_existing_fixture(canonical_path)
                    owned_path = canonical_path
                except Exception as error:
                    print(
                        f"[WU-14] validation fixture ownership recovery failed: {error}"
                    )
                    return False
        if owned_path is None:
            return True
        prim = runtime.stage.GetPrimAtPath(owned_path)
        if not prim.IsValid() or not runtime._is_owned(prim):
            return False
        try:
            return bool(runtime.delete_prim(owned_path))
        except Exception as error:
            print(f"[WU-14] validation fixture teardown failed: {error}")
            return False

    def _fixture_remaining(self):
        runtime = self._runtime
        if runtime is None:
            return False
        path = getattr(runtime, "_owned_path", None)
        if path is None:
            return False
        return bool(runtime.stage.GetPrimAtPath(path).IsValid())

    def _clear_retained_state(self):
        if self._namespace is None:
            return
        for name in self.RETAINED_STATE_NAMES:
            self._namespace[name] = None

    def _clear_validation_state(self):
        if self._namespace is None:
            return
        for name in self.RETAINED_STATE_NAMES:
            self._namespace[name] = None
        self._namespace["pnp_validation_session_owner"] = self

    def _artifacts_remaining(self):
        if self._environment is None:
            return False
        return any(
            self._path_exists(self._environment.get(key)) for key in self.ARTIFACT_KEYS
        )

    @classmethod
    def _clear_artifacts(cls, environment):
        if environment is None:
            return True
        result = True
        for key in cls.ARTIFACT_KEYS:
            path = environment.get(key)
            if path is None:
                continue
            try:
                from pathlib import Path

                Path(path).unlink(missing_ok=True)
            except OSError as error:
                result = False
                print(f"[WU-14] validation artifact teardown failed: {error}")
        return result

    @staticmethod
    def _path_exists(path):
        if path is None:
            return False
        from pathlib import Path

        return Path(path).is_file()

from scene_builder.material_supply_run import MaterialSupplyRun


def _run(quantity=1, deadline_ns=100):
    return MaterialSupplyRun(
        delivery_id="delivery-1",
        material_type="RawPart",
        requested_quantity=quantity,
        deadline_ns=deadline_ns,
    )


def test_quantity_one_completes_after_spawn_without_spatial_observation():
    run = _run()
    run.record_supplied("part-1", completed_ns=9)

    result = run.finish(now_ns=9, pending_spawns=0)

    assert result["completed"] is True
    assert result["supplied_quantity"] == result["requested_quantity"] == 1
    assert result["failure_reason"] == "NONE"


def test_operational_profile_has_no_post_spawn_supply_delay():
    from pathlib import Path

    import yaml

    repository = Path(__file__).resolve().parents[6]
    profile_path = (
        repository
        / "ros2_ws/src/arm_cell_bringup/config/operational_profile.yaml"
    )
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))

    assert "post_spawn_vision_delay_ms" not in profile["material_handoff"]


def test_quantity_two_completes_after_both_spawns():
    run = _run(quantity=2)
    run.record_supplied("part-1", completed_ns=10)
    run.record_supplied("part-2", completed_ns=10)

    result = run.finish(now_ns=10, pending_spawns=0)

    assert result["completed"] is True
    assert result["supplied_quantity"] == result["requested_quantity"] == 2


def test_partial_supply_and_execution_failure_fail_closed():
    partial = _run(quantity=2)
    partial.record_supplied("part-1", completed_ns=10)
    partial.fail_supply("sequence=1 spawn failed")
    result = partial.finish(now_ns=10, pending_spawns=1)
    assert result["completed"] is False
    assert result["supplied_quantity"] == 1
    assert result["uncompleted_spawn_count"] == 1
    assert result["failure_reason"] == "SUPPLY"
    assert result["execution_error"] == "sequence=1 spawn failed"


def test_completed_spawn_before_deadline_succeeds_when_terminalized_later():
    run = _run(deadline_ns=10)
    run.record_supplied("part-1", completed_ns=9)

    result = run.finish(now_ns=11, pending_spawns=0)

    assert result["completed"] is True
    assert result["failure_reason"] == "NONE"
    assert result["supplied_quantity"] == 1


def test_spawn_completed_at_deadline_is_not_successful_supply():
    run = _run(deadline_ns=10)
    run.record_supplied("part-1", completed_ns=10)

    result = run.finish(now_ns=10, pending_spawns=0)

    assert result["completed"] is False
    assert result["failure_reason"] == "SUPPLY"
    assert result["supplied_quantity"] == 1


def test_deadline_terminal_prevents_remaining_spawn():
    run = _run(quantity=2, deadline_ns=10)
    run.record_supplied("part-1", completed_ns=10)

    result = run.finish(now_ns=10, pending_spawns=1)

    assert result["completed"] is False
    assert result["failure_reason"] == "SUPPLY"
    assert result["uncompleted_spawn_count"] == 1
    assert run.finish(now_ns=11, pending_spawns=1) is None


def test_stop_cancellation_reports_supplied_count_once():
    run = _run(quantity=2)
    run.record_supplied("part-1", completed_ns=10)

    result = run.cancel(now_ns=10, pending_spawns=1)

    assert result["completed"] is False
    assert result["valid"] is True
    assert result["supplied_quantity"] == 1
    assert result["failure_reason"] == "CANCELED"
    assert run.cancel(now_ns=11, pending_spawns=0) is None

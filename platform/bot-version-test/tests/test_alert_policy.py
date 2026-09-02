from bike_bot.alert_policy import BicycleAlertPolicy


def test_requires_three_consecutive_bicycle_frames() -> None:
    policy = BicycleAlertPolicy(confirm_frames=3, cooldown_seconds=10)

    assert policy.observe(True, 1.0) is False
    assert policy.streak == 1
    assert policy.observe(True, 1.5) is False
    assert policy.streak == 2
    assert policy.observe(True, 2.0) is True
    assert policy.streak == 0
    assert policy.in_cooldown(2.0)
    assert policy.cooldown_remaining(2.0) == 10.0


def test_missed_frame_resets_confirm_streak() -> None:
    policy = BicycleAlertPolicy(confirm_frames=3, cooldown_seconds=10)

    assert policy.observe(True, 1.0) is False
    assert policy.observe(True, 1.5) is False
    assert policy.observe(False, 2.0) is False
    assert policy.streak == 0
    assert policy.observe(True, 2.5) is False
    assert policy.observe(True, 3.0) is False
    assert policy.observe(True, 3.5) is True


def test_cooldown_blocks_until_expiry_then_needs_three_frames_again() -> None:
    policy = BicycleAlertPolicy(confirm_frames=3, cooldown_seconds=10)

    assert policy.observe(True, 10.0) is False
    assert policy.observe(True, 10.5) is False
    assert policy.observe(True, 11.0) is True
    assert policy.observe(True, 11.5) is False
    assert policy.observe(True, 20.9) is False
    assert policy.in_cooldown(20.9)
    assert policy.observe(True, 21.0) is False
    assert policy.streak == 1
    assert not policy.in_cooldown(21.0)
    assert policy.observe(True, 21.5) is False
    assert policy.observe(True, 22.0) is True
    assert policy.cooldown_remaining(22.0) == 10.0

from bike_bot.rate_control import InferenceRateLimiter, selected_inference_rate_hz


def test_selects_normal_and_person_follow_rates() -> None:
    assert selected_inference_rate_hz(2.0, 5.0, False) == 2.0
    assert selected_inference_rate_hz(2.0, 5.0, True) == 5.0


def test_non_positive_rate_disables_throttling() -> None:
    assert selected_inference_rate_hz(-1.0, 5.0, False) == 0.0


def test_limiter_uses_start_to_start_deadline_without_catchup() -> None:
    limiter = InferenceRateLimiter()
    assert limiter.delay_seconds(10.0) == 0.0
    limiter.mark_started(10.0, 2.0)
    assert round(limiter.delay_seconds(10.2), 6) == 0.3
    assert limiter.delay_seconds(10.5) == 0.0
    limiter.mark_started(12.0, 5.0)
    assert round(limiter.delay_seconds(12.1), 6) == 0.1

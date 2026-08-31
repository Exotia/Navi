from navi_autonomy.path_summary import (MAX_POINTS, TOLERANCE_M, decimate,
                                        polyline_length, summary_payload)


def test_a_straight_line_decimates_to_its_two_ends():
    line = [(x * 0.05, 0.0) for x in range(200)]
    # 199 * 0.05 is 9.950000000000001 in floats; the decimator keeps the raw
    # endpoint untouched, so the assertion must too.
    assert decimate(line) == [(0.0, 0.0), (199 * 0.05, 0.0)]


def test_a_corner_is_kept_because_dropping_it_moves_the_path():
    corner = ([(x * 0.05, 0.0) for x in range(100)]
              + [(4.95, y * 0.05) for y in range(1, 100)])
    points = decimate(corner)
    assert (4.95, 0.0) in points
    assert len(points) == 3


def test_a_deviation_under_the_tolerance_is_dropped():
    line = [(0.0, 0.0), (1.0, TOLERANCE_M / 2.0), (2.0, 0.0)]
    assert decimate(line) == [(0.0, 0.0), (2.0, 0.0)]


def test_the_point_cap_is_never_exceeded_however_wiggly_the_plan():
    import math
    wiggle = [(i * 0.05, math.sin(i * 0.4)) for i in range(1200)]
    points = decimate(wiggle)
    assert len(points) <= MAX_POINTS
    assert points[0] == wiggle[0] and points[-1] == wiggle[-1]


def test_the_ends_always_survive():
    assert decimate([(0.0, 0.0), (5.0, 5.0)]) == [(0.0, 0.0), (5.0, 5.0)]
    assert decimate([(1.0, 2.0)]) == [(1.0, 2.0)]
    assert decimate([]) == []


def test_length_is_the_sum_of_the_segments():
    assert abs(polyline_length([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]) - 7.0) < 1e-9


def test_the_payload_reports_the_raw_length_it_decimated_from():
    payload = summary_payload("gs-1", [(x * 0.05, 0.0) for x in range(200)],
                              [(9.95, 0.0)], now_s=12.0)
    assert payload["source_points"] == 200
    assert payload["points"] == [[0.0, 0.0], [199 * 0.05, 0.0]]
    assert payload["waypoints"] == [[9.95, 0.0]]
    assert payload["run_id"] == "gs-1" and payload["frame_id"] == "map"
    assert abs(payload["length_m"] - 9.95) < 1e-9

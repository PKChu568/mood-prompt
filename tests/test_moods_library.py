"""Validate the emotions library: metadata, trajectories, and selector."""

import numpy as np
import pytest

from mood_prompt.moods.library import (
    library_dir,
    load_metadata,
    load_trajectory,
)
from mood_prompt.moods.selector import FALLBACK_MOOD, select_mood

METADATA = load_metadata()


def test_metadata_nonempty():
    # The dataset's metadata.jsonl drives what is playable; it must parse.
    assert len(METADATA) > 0


@pytest.mark.parametrize("entry", METADATA, ids=[e.title for e in METADATA])
def test_entry_files_exist_and_are_consistent(entry):
    base = library_dir()
    assert (base / entry.motion_file).exists(), f"missing motion {entry.motion_file}"
    assert (base / entry.file_name).exists(), f"missing audio {entry.file_name}"

    traj = load_trajectory(entry.motion_file)
    n = len(traj.time)
    assert n > 0
    # time and every per-frame array must be the same length.
    assert traj.head.shape == (n, 4, 4)
    assert traj.antennas.shape == (n, 2)
    assert traj.body_yaw.shape == (n,)
    # No NaNs anywhere.
    for arr in (traj.time, traj.head, traj.antennas, traj.body_yaw):
        assert not np.isnan(arr).any(), f"NaN in {entry.motion_file}"
    # time should be non-decreasing.
    assert np.all(np.diff(traj.time) >= 0)


def test_head_frames_are_valid_poses():
    traj = load_trajectory(METADATA[0].motion_file)
    # Bottom row of each homogeneous pose is [0, 0, 0, 1].
    assert np.allclose(traj.head[:, 3, :], [0, 0, 0, 1])
    # Rotation blocks are orthonormal (R R^T = I).
    for R in traj.head[:, :3, :3]:
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-3)


def test_selector_matches_title():
    assert select_mood("amazed1") == "amazed1"


def test_selector_falls_back():
    assert select_mood("zzzz-no-such-mood-xyzzy") == FALLBACK_MOOD


def test_selector_empty_is_fallback():
    assert select_mood("   ") == FALLBACK_MOOD

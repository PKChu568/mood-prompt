"""Tests for the mood audio helper.

Does not play sound (CI has no audio device). Checks that audio files are
resolved correctly and that playback degrades gracefully when the optional
dependencies are absent.
"""

import builtins

import pytest

from mood_prompt.moods import audio
from mood_prompt.moods.library import load_metadata


def test_audio_path_resolves_for_every_mood():
    for entry in load_metadata():
        path = audio.audio_path(entry.title)
        assert path is not None
        assert path.exists()
        assert path.name == entry.file_name


def test_audio_path_unknown_mood_is_none():
    assert audio.audio_path("zzzz-no-such-mood") is None


def test_play_without_deps_raises_audio_unavailable(monkeypatch):
    # Simulate the 'audio' extra not being installed: make importing
    # sounddevice/soundfile fail, and confirm a clear error is raised.
    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name in ("sounddevice", "soundfile"):
            raise ImportError(f"no module named {name}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    path = audio.audio_path("curious1")
    with pytest.raises(audio.AudioUnavailable):
        audio.play(path)

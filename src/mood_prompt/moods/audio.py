"""Play a mood's audio track (optional ``audio`` dependency group).

Kept separate from library.py so the core loader has no audio dependency;
only callers that actually want sound import this. Playback is
non-blocking (starts the clip and returns) so it can run alongside the
motion loop, synced by starting both at the same moment.
"""

from __future__ import annotations

from pathlib import Path

from .library import library_dir, load_metadata


class AudioUnavailable(RuntimeError):
    """Raised when the optional audio dependencies are not installed."""


def audio_path(mood: str) -> Path | None:
    """Return the audio file path for a mood title, or None if unknown."""
    for entry in load_metadata():
        if entry.title == mood or entry.motion_file == mood:
            return library_dir() / entry.file_name
    return None


def play(path: Path, blocking: bool = False) -> None:
    """Start playing an audio file. Non-blocking unless ``blocking`` is set.

    Raises :class:`AudioUnavailable` if sounddevice/soundfile are missing.
    """
    try:
        import sounddevice as sd
        import soundfile as sf
    except ImportError as exc:  # optional dep group not installed
        raise AudioUnavailable(
            "audio playback needs the 'audio' extra: uv sync --extra audio"
        ) from exc

    data, samplerate = sf.read(str(path))
    sd.play(data, samplerate)
    if blocking:
        sd.wait()


def stop() -> None:
    """Stop any playback started by :func:`play` (no-op if unavailable)."""
    try:
        import sounddevice as sd
    except ImportError:
        return
    sd.stop()

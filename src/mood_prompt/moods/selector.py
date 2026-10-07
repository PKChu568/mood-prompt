"""Map free text to a mood name.

STUB: a case-insensitive substring match against mood titles and
descriptions. This is deliberately simple -- the real behavior-cloning
approach (text -> expression) is a later phase described in PLAN.md. The
seam is here so callers can depend on select_mood() now and have it get
smarter later without changing its signature.
"""

from __future__ import annotations

from .library import MoodEntry, load_metadata

# Returned when nothing matches, so callers always get a playable mood.
FALLBACK_MOOD = "curious1"


def select_mood(text: str, entries: list[MoodEntry] | None = None) -> str:
    """Return the best-matching mood title for ``text``.

    Scores each mood by case-insensitive substring overlap: a title match
    beats a description match. Falls back to ``FALLBACK_MOOD`` if nothing
    matches.
    """
    if entries is None:
        entries = load_metadata()
    query = text.lower().strip()
    if not query:
        return FALLBACK_MOOD

    best_score = 0
    best_title = FALLBACK_MOOD
    for entry in entries:
        score = 0
        title = entry.title.lower()
        if query in title or title in query:
            score = 3
        elif query in entry.description.lower():
            score = 1
        if score > best_score:
            best_score = score
            best_title = entry.title
    return best_title

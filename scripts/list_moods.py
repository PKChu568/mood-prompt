"""Print all moods with their descriptions.

    uv run python scripts/list_moods.py
"""

from mood_prompt.moods.library import load_metadata


def main() -> None:
    entries = load_metadata()
    width = max(len(e.title) for e in entries)
    for entry in sorted(entries, key=lambda e: e.title):
        print(f"{entry.title:<{width}}  {entry.description}")
    print(f"\n{len(entries)} moods.")


if __name__ == "__main__":
    main()

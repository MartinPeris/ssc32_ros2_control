#!/usr/bin/env python3
"""Check or advance an exact, committed coverage baseline."""

import argparse
import json
import os
from pathlib import Path
import tempfile


METRICS = ("line", "branch")
METRIC_LABELS = {"line": "lines", "branch": "branches"}


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return document


def positive_integer(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label}: expected a positive integer")
    return value


def nonnegative_integer(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label}: expected a non-negative integer")
    return value


def baseline_counts(document: dict, metric: str) -> tuple[int, int]:
    value = document.get(metric)
    if not isinstance(value, dict):
        raise ValueError(f"baseline.{metric}: expected an object")
    covered = nonnegative_integer(value.get("covered"), f"baseline.{metric}.covered")
    total = positive_integer(value.get("total"), f"baseline.{metric}.total")
    if covered > total:
        raise ValueError(f"baseline.{metric}: covered exceeds total")
    return covered, total


def summary_counts(document: dict, metric: str) -> tuple[int, int]:
    covered = nonnegative_integer(
        document.get(f"{metric}_covered"), f"summary.{metric}_covered"
    )
    total = positive_integer(document.get(f"{metric}_total"), f"summary.{metric}_total")
    if covered > total:
        raise ValueError(f"summary.{metric}: covered exceeds total")
    return covered, total


def percent(counts: tuple[int, int]) -> float:
    return 100.0 * counts[0] / counts[1]


def compare(baseline: dict, summary: dict) -> list[str]:
    if baseline.get("schema") != 1:
        raise ValueError("baseline.schema: expected 1")
    failures = []
    for metric in METRICS:
        old = baseline_counts(baseline, metric)
        new = summary_counts(summary, metric)
        if new[0] * old[1] < old[0] * new[1]:
            failures.append(
                f"{metric} coverage regressed: {percent(new):.3f}% "
                f"({new[0]}/{new[1]}) < {percent(old):.3f}% ({old[0]}/{old[1]})"
            )
    return failures


def check(baseline_path: Path, summary_path: Path) -> int:
    baseline = load_json(baseline_path)
    summary = load_json(summary_path)
    failures = compare(baseline, summary)
    if failures:
        for failure in failures:
            print(f"Coverage ratchet failed: {failure}")
        return 1
    measurements = []
    for metric in METRICS:
        current = summary_counts(summary, metric)
        floor = baseline_counts(baseline, metric)
        measurements.append(
            f"{METRIC_LABELS[metric]} {percent(current):.3f}% >= {percent(floor):.3f}%"
        )
    print("Coverage ratchet passed: " + "; ".join(measurements))
    return 0


def update(baseline_path: Path, summary_path: Path) -> int:
    baseline = load_json(baseline_path)
    summary = load_json(summary_path)
    failures = compare(baseline, summary)
    if failures:
        for failure in failures:
            print(f"Coverage baseline not updated: {failure}")
        return 1
    replacement = {"schema": 1}
    for metric in METRICS:
        covered, total = summary_counts(summary, metric)
        replacement[metric] = {"covered": covered, "total": total}
    baseline_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=baseline_path.parent, delete=False
    ) as stream:
        json.dump(replacement, stream, indent=2)
        stream.write("\n")
        temporary_path = Path(stream.name)
    os.replace(temporary_path, baseline_path)
    print(f"Updated coverage baseline: {baseline_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("check", "update"))
    parser.add_argument("baseline", type=Path)
    parser.add_argument("summary", type=Path)
    arguments = parser.parse_args()
    try:
        if arguments.action == "check":
            return check(arguments.baseline, arguments.summary)
        return update(arguments.baseline, arguments.summary)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.exit(2, f"coverage ratchet: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())

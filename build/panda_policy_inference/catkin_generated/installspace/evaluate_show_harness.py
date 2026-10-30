#!/usr/bin/env python3
"""Summarize completed JSONL trials without ROS, an API key, or robot access."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


OPPOSITES = {"MV_LEFT": "MV_RIGHT", "MV_RIGHT": "MV_LEFT",
             "MV_FWD": "MV_BACK", "MV_BACK": "MV_FWD"}


def summarize(paths, labels=None):
    labels = labels or {}
    trials = []
    api_latencies = []
    tokens = Counter()
    for path in paths:
        events = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
        starts = [event for event in events if event.get("outcome") == "episode_start"]
        ends = [event for event in events if event.get("outcome") == "episode_end"]
        if len(starts) != 1 or len(ends) != 1:
            raise ValueError("{} must contain one completed episode".format(path))
        start, end = starts[0], ends[0]
        episode_id = start["episode_id"]
        if any(event.get("episode_id") != episode_id for event in events):
            raise ValueError("mixed episode IDs in {}".format(path))
        label = labels.get(episode_id)
        if label is not None and not isinstance(label, bool):
            raise ValueError("success labels must be Boolean")
        executed = [event for event in events if event.get("action") in OPPOSITES
                    and event.get("outcome") == "executed_and_settled"]
        reversals = sum(OPPOSITES.get(previous["action"]) == current["action"]
                        for previous, current in zip(executed, executed[1:]))
        models = [event for event in events if event.get("outcome") == "model_response"]
        api_latencies.extend(event["api_seconds"] for event in models)
        for event in models:
            usage = event.get("usage") or {}
            for key in ("input_tokens", "output_tokens", "total_tokens"):
                tokens[key] += usage.get(key, 0)
        trials.append({"path": str(path), "episode_id": episode_id,
                       "dry_run": end["dry_run"], "termination": end["termination"],
                       "model_claimed_success": end["model_claimed_success"],
                       "independent_success": label, "actions": end["executed_actions"],
                       "api_calls": end["api_calls"], "seconds": end["episode_seconds"],
                       "recovery_events": sum(event.get("outcome") == "grasp_recovery" for event in events),
                       "horizontal_reversals": reversals})
    live = [trial for trial in trials if not trial["dry_run"]]
    scored = [trial for trial in live if trial["independent_success"] is not None]
    return {"trials": trials, "live_episodes": len(live), "dry_run_previews": len(trials) - len(live),
            "terminations": dict(Counter(trial["termination"] for trial in live)),
            "independently_scored_episodes": len(scored),
            "independent_success_rate": (sum(trial["independent_success"] for trial in scored) / len(scored)
                                         if scored else None),
            "model_done_rate": (sum(trial["model_claimed_success"] for trial in live) / len(live)
                                if live else None),
            "mean_actions_live": statistics.mean(trial["actions"] for trial in live) if live else None,
            "mean_seconds_live": statistics.mean(trial["seconds"] for trial in live) if live else None,
            "median_api_seconds_all": statistics.median(api_latencies) if api_latencies else None,
            "tokens_all": dict(tokens)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="JSONL episode files")
    parser.add_argument("--labels", type=Path, help='JSON object: {"episode_id": true/false}')
    args = parser.parse_args()
    try:
        labels = json.loads(args.labels.read_text()) if args.labels else None
        if labels is not None and not isinstance(labels, dict):
            raise ValueError("labels must be a JSON object")
        print(json.dumps(summarize(args.logs, labels), indent=2, allow_nan=False))
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

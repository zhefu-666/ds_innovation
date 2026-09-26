#!/usr/bin/env python3
"""Generate a small JSON-schema MCAP sample using the official mcap Python API."""

import argparse
import json
import time

from mcap.writer import Writer


SCHEMA = json.dumps({"type": "object", "additionalProperties": True}).encode("utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", nargs="?", default="rescue_sample.mcap")
    args = parser.parse_args()
    now = time.time_ns()
    with open(args.output, "wb") as stream:
        writer = Writer(stream)
        writer.start()
        schema_id = writer.register_schema("rescue.FsmState", "jsonschema", SCHEMA)
        channel_id = writer.register_channel("/fsm/state", "json", schema_id)
        for sequence in range(30):
            timestamp = now + sequence * 100_000_000
            data = json.dumps({"name": "SEARCH" if sequence < 15 else "APPROACH", "sequence": sequence}).encode("utf-8")
            writer.add_message(channel_id, log_time=timestamp, publish_time=timestamp, data=data)
        writer.finish()
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()

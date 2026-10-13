"""Emit bounded software-test failures; complete output stays in CI artifacts."""

import argparse
from pathlib import Path
from xml.etree import ElementTree


def annotation(message):
    escaped = message.replace("%", "%25").replace("\r", "%0D").replace("\n", "%0A")
    print(f"::error title=SDK software test failure::{escaped}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--junit", type=Path)
    parser.add_argument("--log", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.junit is not None and arguments.junit.is_file():
        root = ElementTree.parse(arguments.junit).getroot()
        failures = []
        for case in root.iter("testcase"):
            errors = [child for child in case if child.tag in ("failure", "error")]
            if errors:
                name = f"{case.get('classname', '')}::{case.get('name', '')}"
                failures.append(name)
                if len(failures) <= 5:
                    annotation(
                        name
                        + "\n"
                        + (errors[0].text or errors[0].get("message", ""))[:6000]
                    )
        if failures:
            annotation(f"{len(failures)} failed cases\n" + "\n".join(failures)[:12000])
    if arguments.log.is_file():
        with arguments.log.open("rb") as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 12000))
            tail = stream.read(12000).decode("utf-8", errors="replace")
        annotation("Original test log tail:\n" + tail)


if __name__ == "__main__":
    main()

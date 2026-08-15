"""Combine FMUs that carry different platforms' binaries into one.

FMI 3.0 expects a single archive to hold every platform it supports, under
``binaries/<platform>/``. Anything that builds one platform at a time leaves a
set of archives that each describe the same model and carry one binary apiece,
and this puts them back together.

That is what packaging on several machines produces, which is why it exists:
each agent writes an FMU carrying only the binary for the platform it ran on.
Nothing here depends on that, and any set of FMUs meeting the same description
merges the same way.

The sources are grouped by file name, which is the model's library name and so
identifies which archives belong together, and each group becomes one merged
FMU. A directory holding several models' FMUs therefore merges all of them.

Everything outside ``binaries/`` is shared within a group, and identical
between its members but for ``generationDateAndTime``: the instantiation token
is a UUID derived from the model's name rather than from the build. So one
copy is kept and the rest are checked against it.

Point it at whatever holds the builds and it searches for them:

    python scripts/merge_fmus.py downloaded-fmus --out-dir merged

Both default to the shell's working directory, so standing in the folder the
builds were downloaded into needs no arguments at all. The search reaches down
into whatever subdirectories they sit in, and each merged FMU is written to
that working directory:

    cd downloaded-fmus
    python ../continuo/python/scripts/merge_fmus.py

The only thing the layout has to satisfy is that each build sits somewhere of
its own, which it does by construction: builds of one FMU share a file name,
so they cannot be in the same directory. How they got there does not matter,
whether that is `gh run download`, an unzipped download, or four folders
arranged by hand.
"""

from __future__ import annotations

import argparse
import re
import sys
import zipfile
from collections import defaultdict
from pathlib import Path

# What every archive in a group agrees on except this, which is a timestamp
# and so differs between two packagings of the same source.
_GENERATED_AT = re.compile(rb'\s*generationDateAndTime="[^"]*"')

BINARIES = "binaries/"
MODEL_DESCRIPTION = "modelDescription.xml"


class MergeError(Exception):
    """Something about the inputs that no merged FMU could come from."""


def find_fmus(source_fmu_dir: Path) -> list[Path]:
    """Every archive under one directory, at any depth, in path order.

    Searched rather than listed, so the caller names one folder instead of
    each build inside it. That keeps a shell out of it: a glob would leave
    this working everywhere but Windows, where neither the shell nor Python
    expands one. Builds spread across several places want their common parent.
    """
    if not source_fmu_dir.is_dir():
        raise MergeError(f"{source_fmu_dir} is not a directory")
    found = sorted(source_fmu_dir.rglob("*.fmu"))
    if not found:
        raise MergeError(f"no .fmu under {source_fmu_dir}")

    # Return them in path order, so a run reports what it found the same way
    # however the filesystem lists it.
    return found


def group_by_fmu(fmus: list[Path]) -> dict[str, list[Path]]:
    """The archives gathered into one group per FMU.

    Grouped by file name, which is the crate's library name, so one FMU's
    builds come together wherever they were found. A workspace packaging two
    FMU crates puts a build of each beside every platform's other, so the
    groups run across the directories rather than within them.
    """
    groups: dict[str, list[Path]] = defaultdict(list)
    for fmu in fmus:
        groups[fmu.name].append(fmu)

    # Return the groups in name order, so a run reports them the same way
    # whichever order they were found in.
    return {name: groups[name] for name in sorted(groups)}


def inputs_to_merge(fmus: list[Path], out: Path) -> list[Path]:
    """The group's archives, less the one this run is about to write.

    Writing into the directory being searched is the ordinary case, since
    both default to the current directory, so a second run finds what the
    first one wrote. It is
    a merge of the others and would collide with all of them, and it is about
    to be replaced anyway, so it is passed over.

    An archive at that path carrying one platform is a different thing: an
    input that happens to sit where the output goes. Overwriting it would drop
    its platform from the result without saying so, and it is the only file a
    run could destroy, so it stops instead.
    """
    keep = []
    for fmu in fmus:
        if fmu.resolve() != out.resolve():
            keep.append(fmu)
            continue
        with zipfile.ZipFile(fmu) as archive:
            if len(platforms_in(archive)) < 2:
                raise MergeError(
                    f"{fmu} is where the merged FMU would go and carries one "
                    "platform, so merging would overwrite an input; write "
                    "somewhere else with --out-dir"
                )

    # Return what is left, which is every archive that is not this run's own
    # output from a previous one.
    return keep


def platforms_in(archive: zipfile.ZipFile) -> set[str]:
    """The platform directories an archive carries binaries for."""
    return {
        name[len(BINARIES) :].split("/")[0]
        for name in archive.namelist()
        if name.startswith(BINARIES) and name.count("/") >= 2
    }


def _comparable(model_description: bytes) -> bytes:
    """The description without the one attribute that is a timestamp."""
    return _GENERATED_AT.sub(b"", model_description)


def merge(fmus: list[Path], out: Path) -> dict[str, list[str]]:
    """Writes one FMU carrying every input's binaries. Returns what it took.

    The first archive supplies everything outside ``binaries/``. The rest
    supply only their binaries, after their description is checked against the
    first, since two archives describing different models would merge into one
    that misdescribes both.
    """
    contents: dict[str, bytes] = {}
    description: bytes | None = None
    taken: dict[str, list[str]] = {}

    for index, fmu in enumerate(fmus):
        with zipfile.ZipFile(fmu) as source:
            if MODEL_DESCRIPTION not in source.namelist():
                raise MergeError(f"{fmu} has no {MODEL_DESCRIPTION}")
            here = source.read(MODEL_DESCRIPTION)
            if description is None:
                description = here
            elif _comparable(here) != _comparable(description):
                raise MergeError(
                    f"{fmu} describes a different model from {fmus[0]}; "
                    "these are not builds of one FMU"
                )

            platforms = platforms_in(source)
            if not platforms:
                raise MergeError(f"{fmu} carries no {BINARIES} entries")
            # Keyed by path, since every build of one FMU has the same name.
            taken[str(fmu)] = sorted(platforms)

            for item in source.infolist():
                if item.is_dir():
                    continue
                if item.filename.startswith(BINARIES):
                    if item.filename in contents:
                        raise MergeError(
                            f"{item.filename} appears in more than one input"
                        )
                    contents[item.filename] = source.read(item)
                elif index == 0:
                    contents[item.filename] = source.read(item)

    out.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as merged:
        # Sorted, so the layout comes from the merge rather than from the
        # order the artifacts happened to arrive in.
        for name in sorted(contents):
            merged.writestr(name, contents[name])

    return taken


def main(argv: list[str] | None = None) -> int:
    # Spelled out rather than taken from `__doc__`, which is None under `-OO`.
    parser = argparse.ArgumentParser(
        description="Combine FMUs that carry different platforms' binaries into one."
    )
    parser.add_argument(
        "source_fmu_dir",
        nargs="?",
        default=Path("."),
        type=Path,
        help="the root of a directory holding the FMUs to merge, searched at "
        "any depth (default: the current directory)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("."),
        help="where to write the merged FMUs (default: the current directory)",
    )
    args = parser.parse_args(argv)

    try:
        groups = group_by_fmu(find_fmus(args.source_fmu_dir))
        for name, fmus in groups.items():
            out = args.out_dir / name
            taken = merge(inputs_to_merge(fmus, out), out)
            print(f"{name}")
            for source, platforms in sorted(taken.items()):
                print(f"  {source}: {', '.join(platforms)}")
    except MergeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    print(f"wrote {len(groups)} FMU(s) to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

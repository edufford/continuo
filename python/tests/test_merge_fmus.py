"""The multi-platform FMU merge."""

import zipfile

import pytest
from merge_fmus import MergeError, find_fmus, group_by_fmu, inputs_to_merge, merge

# A stand-in for what a packager writes. The token is invented: the merge
# never reads one, and only cares that a group's descriptions match.
DESCRIPTION = (
    b'<?xml version="1.0" encoding="UTF-8"?>'
    b'<fmiModelDescription fmiVersion="3.0" modelName="demo" '
    b'instantiationToken="11111111-2222-3333-4444-555555555555" '
    b'generationDateAndTime="{stamp}"/>'
)


def write_fmu(path, platform, stamp="2026-01-01T00:00:00Z", model=DESCRIPTION):
    """One platform's FMU, as the packager writes it."""
    with zipfile.ZipFile(path, "w") as fmu:
        fmu.writestr("modelDescription.xml", model.replace(b"{stamp}", stamp.encode()))
        fmu.writestr(f"binaries/{platform}/demo.bin", f"code for {platform}".encode())
        fmu.writestr("sources/buildDescription.xml", b"<fmiBuildDescription/>")
    return path


def test_the_merged_fmu_carries_every_platforms_binary(tmp_path):
    platforms = ["aarch64-darwin", "aarch64-linux", "x86_64-linux", "x86_64-windows"]
    fmus = [write_fmu(tmp_path / f"{p}.fmu", p) for p in platforms]

    out = tmp_path / "merged.fmu"
    merge(fmus, out)

    with zipfile.ZipFile(out) as merged:
        names = merged.namelist()
        for platform in platforms:
            assert f"binaries/{platform}/demo.bin" in names
            assert merged.read(f"binaries/{platform}/demo.bin") == (
                f"code for {platform}".encode()
            )
        # Everything outside binaries/ is carried once, from the first input.
        assert names.count("modelDescription.xml") == 1
        assert names.count("sources/buildDescription.xml") == 1


def test_the_merged_fmu_keeps_one_description(tmp_path):
    # The packager stamps each build with the time it ran, so the four
    # descriptions differ there and nowhere else. That is not a disagreement.
    fmus = [
        write_fmu(tmp_path / "a.fmu", "x86_64-linux", stamp="2026-01-01T00:00:00Z"),
        write_fmu(tmp_path / "b.fmu", "x86_64-windows", stamp="2026-06-30T12:34:56Z"),
    ]

    out = tmp_path / "merged.fmu"
    merge(fmus, out)

    with zipfile.ZipFile(out) as merged:
        assert merged.read("modelDescription.xml") == zipfile.ZipFile(fmus[0]).read(
            "modelDescription.xml"
        )


def test_the_entries_are_ordered_the_same_whichever_order_the_inputs_come_in(tmp_path):
    # CI hands over one directory per platform and promises nothing about
    # the order, so the layout has to come from the merge rather than from
    # the arguments.
    platforms = ["x86_64-windows", "aarch64-linux", "x86_64-linux"]
    fmus = [write_fmu(tmp_path / f"{p}.fmu", p) for p in platforms]

    forward = tmp_path / "forward.fmu"
    backward = tmp_path / "backward.fmu"
    merge(fmus, forward)
    merge(list(reversed(fmus)), backward)

    with zipfile.ZipFile(forward) as a, zipfile.ZipFile(backward) as b:
        assert a.namelist() == b.namelist()
        assert a.namelist() == sorted(a.namelist())


def test_two_builds_of_different_models_are_refused(tmp_path):
    # Merging these would produce one FMU whose description fits neither.
    other = DESCRIPTION.replace(b'modelName="demo"', b'modelName="something-else"')
    fmus = [
        write_fmu(tmp_path / "a.fmu", "x86_64-linux"),
        write_fmu(tmp_path / "b.fmu", "x86_64-windows", model=other),
    ]

    with pytest.raises(MergeError, match="different model"):
        merge(fmus, tmp_path / "merged.fmu")


def test_each_fmu_is_merged_from_its_own_builds(tmp_path):
    # A workspace packaging two FMU crates puts both in every platform's
    # artifact, so the sets to merge run across the directories rather than
    # within them. Grouping by anything but the name would cross them.
    platforms = ["x86_64-linux", "x86_64-windows"]
    for platform in platforms:
        artifact = tmp_path / f"fmus-{platform}"
        artifact.mkdir()
        write_fmu(artifact / "controller_idm.fmu", platform, model=DESCRIPTION)
        write_fmu(
            artifact / "controller_ai.fmu",
            platform,
            model=DESCRIPTION.replace(b'modelName="demo"', b'modelName="learned"'),
        )

    groups = group_by_fmu(find_fmus(tmp_path))

    assert list(groups) == ["controller_ai.fmu", "controller_idm.fmu"]
    for name, fmus in groups.items():
        merged = tmp_path / "merged" / name
        merge(fmus, merged)
        with zipfile.ZipFile(merged) as archive:
            assert sorted(platforms_of(archive)) == platforms


def platforms_of(archive):
    """The platforms an archive carries binaries for."""
    return {
        name.split("/")[1]
        for name in archive.namelist()
        if name.startswith("binaries/")
    }


def test_two_builds_for_the_same_platform_are_refused(tmp_path):
    # One would silently overwrite the other, so the merged FMU would carry
    # a binary from a build nobody chose.
    fmus = [
        write_fmu(tmp_path / "a.fmu", "x86_64-linux"),
        write_fmu(tmp_path / "b.fmu", "x86_64-linux"),
    ]

    with pytest.raises(MergeError, match="more than one input"):
        merge(fmus, tmp_path / "merged.fmu")


def test_merging_in_place_passes_over_the_last_run(tmp_path):
    # Both paths default to here, so a second run finds the first run's
    # output. It is a merge of the others, would collide with all of them,
    # and is about to be replaced.
    platforms = ["x86_64-linux", "x86_64-windows"]
    for platform in platforms:
        (tmp_path / platform).mkdir()
        write_fmu(tmp_path / platform / "demo.fmu", platform)

    out = tmp_path / "demo.fmu"
    merge(inputs_to_merge(group_by_fmu(find_fmus(tmp_path))["demo.fmu"], out), out)
    again = group_by_fmu(find_fmus(tmp_path))["demo.fmu"]
    merge(inputs_to_merge(again, out), out)

    with zipfile.ZipFile(out) as merged:
        assert sorted(platforms_of(merged)) == platforms


def test_an_input_where_the_output_goes_is_not_overwritten(tmp_path):
    # One platform's FMU sitting at the output path is an input rather than
    # a previous merge, and overwriting it would drop its platform from the
    # result without saying so.
    (tmp_path / "linux").mkdir()
    write_fmu(tmp_path / "linux" / "demo.fmu", "x86_64-linux")
    in_the_way = write_fmu(tmp_path / "demo.fmu", "x86_64-windows")

    fmus = group_by_fmu(find_fmus(tmp_path))["demo.fmu"]
    with pytest.raises(MergeError, match="would overwrite an input"):
        inputs_to_merge(fmus, tmp_path / "demo.fmu")

    with zipfile.ZipFile(in_the_way) as untouched:
        assert platforms_of(untouched) == {"x86_64-windows"}

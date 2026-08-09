import importlib.machinery
import os
import re
import sys
from pathlib import Path

PAYLOAD_MODULE_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*Payload$")
COMPILED_EXTENSION_SUFFIXES = tuple(sorted({
    *importlib.machinery.EXTENSION_SUFFIXES,
    ".so",
    ".pyd",
    ".dylib",
}))


def read_payload_manifest(manifest_path):
    """Read and validate payload module names from a CMake manifest.

    :param manifest_path: manifest containing one payload module name per line
    :return: sorted payload module names
    :raises ValueError: if the manifest contains an invalid or duplicate name
    """
    payload_modules = []
    payload_lines = {}
    with Path(manifest_path).open(encoding="utf-8") as manifest_file:
        for line_number, line in enumerate(manifest_file, start=1):
            payload_module = line.strip()
            if not payload_module:
                continue
            if not PAYLOAD_MODULE_PATTERN.fullmatch(payload_module):
                raise ValueError(
                    f"Invalid payload module '{payload_module}' on line "
                    f"{line_number} of '{manifest_path}'."
                )
            if payload_module in payload_lines:
                raise ValueError(
                    f"Duplicate payload module '{payload_module}' on lines "
                    f"{payload_lines[payload_module]} and {line_number} of "
                    f"'{manifest_path}'."
                )
            payload_lines[payload_module] = line_number
            payload_modules.append(payload_module)
    return sorted(payload_modules)


def payload_artifact_paths(payload_module, module_output_path, auto_source_path):
    """Return the exact generated artifact paths owned by one payload module.

    :param payload_module: validated payload module name
    :param module_output_path: generated Python messaging package directory
    :param auto_source_path: generated SWIG and metadata directory
    :return: sorted generated artifact paths
    """
    if not PAYLOAD_MODULE_PATTERN.fullmatch(payload_module):
        raise ValueError(f"Invalid payload module '{payload_module}'.")

    module_output_directory = Path(module_output_path)
    auto_source_directory = Path(auto_source_path)
    artifact_paths = {
        module_output_directory / f"{payload_module}.py",
        module_output_directory / f"{payload_module}PYTHON_wrap.cxx",
        auto_source_directory / f"{payload_module}.i",
        auto_source_directory / f"{payload_module}_equality.h",
        auto_source_directory / "cMsgMeta" / f"{payload_module}.json",
        auto_source_directory / "cMsgMeta" / f"{payload_module}.d",
    }
    artifact_paths.update(
        module_output_directory / f"_{payload_module}{extension_suffix}"
        for extension_suffix in COMPILED_EXTENSION_SUFFIXES
    )
    return sorted(artifact_paths)


def remove_stale_payload_artifacts(
        current_payloads,
        previous_manifest_path,
        module_output_path,
        auto_source_path,
):
    """Remove known generated files for payloads absent from the current build.

    :param current_payloads: payload modules selected by the current CMake run
    :param previous_manifest_path: retained manifest from the prior generation
    :param module_output_path: generated Python messaging package directory
    :param auto_source_path: generated SWIG and metadata directory
    :return: sorted names of removed payload modules
    """
    previous_manifest = Path(previous_manifest_path)
    if not previous_manifest.exists():
        return []

    previous_payloads = read_payload_manifest(previous_manifest)
    removed_payloads = sorted(set(previous_payloads) - set(current_payloads))
    for payload_module in removed_payloads:
        for artifact_path in payload_artifact_paths(
                payload_module,
                module_output_path,
                auto_source_path,
        ):
            artifact_path.unlink(missing_ok=True)
    return removed_payloads


def write_text_atomically(output_path, content):
    """Atomically replace a generated text file.

    :param output_path: file to replace
    :param content: complete new file content
    """
    output_file = Path(output_path)
    temporary_file = output_file.with_name(f".{output_file.name}.tmp")
    try:
        temporary_file.write_text(content, encoding="utf-8")
        os.replace(temporary_file, output_file)
    finally:
        temporary_file.unlink(missing_ok=True)


def generate_package_init(
        module_output_path,
        manifest_path,
        previous_manifest_path,
):
    """Regenerate package imports and remove artifacts for deleted payloads.

    :param module_output_path: directory where the package initializer is written
    :param manifest_path: current CMake-generated payload module manifest
    :param previous_manifest_path: retained manifest from the prior generation
    :return: sorted names of removed payload modules
    """
    output_directory = Path(module_output_path)
    output_directory.mkdir(parents=True, exist_ok=True)
    current_payloads = read_payload_manifest(manifest_path)
    removed_payloads = remove_stale_payload_artifacts(
        current_payloads,
        previous_manifest_path,
        output_directory,
        Path(manifest_path).parent,
    )

    init_content = (
        "from Basilisk.architecture.messaging.messagingSupport import *\n"
    )
    init_content += "".join(
        f"from Basilisk.architecture.messaging.{payload_module} import *\n"
        for payload_module in current_payloads
    )
    init_content += "from Basilisk.architecture.messagingBase import *\n"
    output_path = output_directory / "__init__.py"
    write_text_atomically(output_path, init_content)

    manifest_content = "".join(
        f"{payload_module}\n" for payload_module in current_payloads
    )
    write_text_atomically(previous_manifest_path, manifest_content)
    return removed_payloads


if __name__ == "__main__":
    removed_modules = generate_package_init(sys.argv[1], sys.argv[2], sys.argv[3])
    if removed_modules:
        print(
            "Removed stale generated artifacts for payloads: "
            + ", ".join(removed_modules)
        )

    # XXX: Disabled: don't make a symbolic link here, because when we build a
    # Python wheel, the contents of the folder get copied, effectively doubling
    # the size. Instead, see the new messaging/cMsgCInterface directory.
    # os.symlink(module_output_path, legacy_output_path)

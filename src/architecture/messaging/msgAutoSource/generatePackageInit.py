import re
import sys
from pathlib import Path

PAYLOAD_MODULE_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*Payload$")


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


def generate_package_init(module_output_path, manifest_path):
    """Generate imports for the payload modules selected by CMake.

    :param module_output_path: directory where the package initializer is written
    :param manifest_path: CMake-generated payload module manifest
    """
    output_directory = Path(module_output_path)
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / "__init__.py"
    with output_path.open("w", encoding="utf-8") as main_import_file:
        main_import_file.write(
            "from Basilisk.architecture.messaging.messagingSupport import *\n"
        )
        for payload_module in read_payload_manifest(manifest_path):
            main_import_file.write(
                f"from Basilisk.architecture.messaging.{payload_module} import *\n"
            )
        main_import_file.write(
            "from Basilisk.architecture.messagingBase import *\n"
        )


if __name__ == "__main__":
    generate_package_init(sys.argv[1], sys.argv[2])

    # XXX: Disabled: don't make a symbolic link here, because when we build a
    # Python wheel, the contents of the folder get copied, effectively doubling
    # the size. Instead, see the new messaging/cMsgCInterface directory.
    # os.symlink(module_output_path, legacy_output_path)

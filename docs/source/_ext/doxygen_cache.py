"""Generate and cache Doxygen XML independently for each documented project."""

import hashlib
import json
import re
import shutil
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

from generated_documentation import sync_generated_tree


# Increment this when cache semantics change without changing the generated
# Doxygen configuration or its source inputs.
_CACHE_FORMAT_VERSION = "4"
# These commands can read files outside the C/C++ include graph. Match both
# Doxygen command prefixes and legacy forms, including commands in aliases.
_DOCUMENTATION_FILE_COMMAND = re.compile(
    r"[@\\](?:include(?:doc|lineno)?|dontinclude|snippet(?:doc|lineno)?|"
    r"(?:verb|html|latex|rtf|man|docbook|xml)include|example|image|"
    r"(?:dot|msc|dia|plantuml|mermaid)file)\b"
)
_DOXYGEN_CONFIG_TEMPLATE = r"""
PROJECT_NAME     = {project_name}
GENERATE_LATEX   = NO
GENERATE_MAN     = NO
GENERATE_RTF     = NO
CASE_SENSE_NAMES = NO
INPUT            = {input_files}
ENABLE_PREPROCESSING = YES
QUIET            = YES
JAVADOC_AUTOBRIEF = YES
JAVADOC_AUTOBRIEF = NO
GENERATE_HTML = NO
GENERATE_XML = YES
ALIASES = rst="\verbatim embed:rst"
ALIASES += endrst="\endverbatim"
ALIASES += inlinerst="\verbatim embed:rst:inline"
{extra_options}
OUTPUT_DIRECTORY = {output_directory}
XML_OUTPUT = xml
""".strip()


def _quote_doxygen_value(value):
    """Quote one Doxygen configuration value."""
    return '"' + str(value).replace("\\", "/").replace('"', '\\"') + '"'


def _create_doxygen_config(project_name, input_paths, options, aliases, output_directory):
    """Create a deterministic Doxygen configuration for one project."""
    # Doxygen processes includes and overrides in order. A mapping's insertion
    # order is deterministic and preserves the caller's intended precedence.
    option_lines = [f"{name}={value}" for name, value in options.items()]
    alias_lines = [
        f'ALIASES += {name}="{value}"' for name, value in sorted(aliases.items())
    ]
    extra_options = "\n".join(option_lines + alias_lines)
    quoted_inputs = " ".join(_quote_doxygen_value(path) for path in input_paths)
    return _DOXYGEN_CONFIG_TEMPLATE.format(
        project_name=_quote_doxygen_value(project_name),
        input_files=quoted_inputs,
        extra_options=extra_options,
        output_directory=_quote_doxygen_value(output_directory),
    ) + "\n"


def _project_dependencies(input_paths, options, configuration_directory):
    """Collect literal local includes, or return ``None`` to disable reuse.

    Includes in all conditional branches are tracked conservatively, including
    headers resolved from Doxygen's configuration-directory working directory.
    Rescanning on every build also detects newly created, previously unresolved
    headers.
    Custom search paths, filters, bibliography files, computed include names,
    and file-inclusion documentation commands require Doxygen's own resolver,
    so projects using them are regenerated instead of guessed at.
    """
    external_input_options = (
        "INPUT", "INCLUDE_PATH", "INPUT_FILTER", "FILTER_PATTERNS", "TAGFILES",
        "EXAMPLE_PATH", "IMAGE_PATH", "CITE_BIB_FILES", "@INCLUDE", "@INCLUDE_PATH",
        "INPUT_ENCODING", "INPUT_FILE_ENCODING",
    )
    if any(options.get(name) for name in external_input_options):
        return None
    preprocessing_enabled = str(options.get("ENABLE_PREPROCESSING", "YES")).upper() != "NO"
    configuration_root = Path(configuration_directory).resolve()

    # SEARCH_INCLUDES=NO still allows Doxygen to preprocess nearby quoted
    # headers, so keep scanning local dependencies regardless of that setting.
    dependencies = set()
    pending_paths = list(input_paths)
    while pending_paths:
        source_path = pending_paths.pop()
        if source_path in dependencies:
            continue
        dependencies.add(source_path)
        source = source_path.read_text(encoding="utf8", errors="replace")
        source = re.sub(r"\\\r?\n", "", source)
        # Inspect documentation before stripping comments, even when C/C++
        # preprocessing is disabled. Conservative matches in literal examples
        # may regenerate unnecessarily but cannot leave included content stale.
        if _DOCUMENTATION_FILE_COMMAND.search(source):
            return None
        if not preprocessing_enabled:
            continue
        # Preserve quoted strings while removing comments between directive
        # tokens; a comment or URL inside a string must not hide an include.
        source = re.sub(
            r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|/\*.*?\*/|//[^\n]*',
            lambda match: " " if match[0].startswith("/") else match[0],
            source,
            flags=re.DOTALL,
        )
        for directive in re.finditer(
            r"^\s*#\s*(include|include_next|import)\b([^\n]*)", source, re.MULTILINE
        ):
            include = re.fullmatch(r'\s*[<"]([^>"\n]+)[>"]\s*', directive[2])
            if include is None or directive[1] == "include_next":
                return None
            include_name = include[1]
            # Doxygen searches its working directory even without INCLUDE_PATH
            # and with SEARCH_INCLUDES=NO. Use its configured cwd, not our own.
            candidates = {
                (source_path.parent / include_name).resolve(),
                (configuration_root / include_name).resolve(),
            }
            # Doxygen also resolves includes against the project's INPUT files.
            candidates.update(
                path for path in input_paths
                if path.as_posix().endswith("/" + include_name)
            )
            pending_paths.extend(path for path in candidates if path.is_file())
    return sorted(dependencies)


def _project_fingerprint(config, input_paths, doxygen_version, configuration_directory):
    """Hash the configuration, working directory, version, and dependencies."""
    digest = hashlib.sha256()
    digest.update(_CACHE_FORMAT_VERSION.encode("utf8"))
    digest.update(b"\0")
    digest.update(doxygen_version.encode("utf8"))
    digest.update(b"\0")
    digest.update(config.encode("utf8"))
    digest.update(b"\0")
    digest.update(configuration_directory.as_posix().encode("utf8"))

    for input_path in input_paths:
        digest.update(b"\0")
        digest.update(input_path.as_posix().encode("utf8"))
        digest.update(b"\0")
        digest.update(hashlib.sha256(input_path.read_bytes()).digest())

    return digest.hexdigest()


def _project_cache_name(project_name):
    """Return a readable, collision-resistant directory name for a project."""
    readable_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", project_name).strip("._")
    if not readable_name:
        readable_name = "project"
    name_digest = hashlib.sha256(project_name.encode("utf8")).hexdigest()[:12]
    return f"{readable_name[:60]}-{name_digest}"


def _write_text_if_changed(path, contents):
    """Write text only when its content differs from the existing file."""
    if path.is_file() and path.read_text(encoding="utf8") == contents:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf8")


def _load_cache_state(state_path):
    """Read cache state, returning ``None`` when it is missing or invalid."""
    try:
        return json.loads(state_path.read_text(encoding="utf8"))
    except (OSError, ValueError):
        return None


def _cache_is_current(project_root, fingerprint):
    """Check the fingerprint and output manifest for one cached project."""
    state = _load_cache_state(project_root / "cache-state.json")
    if not isinstance(state, dict) or state.get("fingerprint") != fingerprint:
        return False

    xml_root = project_root / "xml"
    output_files = state.get("output_files")
    return (
        isinstance(output_files, list)
        and bool(output_files)
        and all((xml_root / relative_path).is_file() for relative_path in output_files)
    )


def _is_sigbus_return_code(return_code):
    """Return whether a process status represents termination by ``SIGBUS``."""
    sigbus = getattr(signal, "SIGBUS", None)
    return sigbus is not None and return_code in (-sigbus, 128 + sigbus)


def _run_doxygen(doxygen_executable, config_path, configuration_directory, max_attempts=3):
    """Run from the Sphinx configuration directory, retrying only ``SIGBUS``."""
    command = [doxygen_executable, str(config_path.resolve())]
    for attempt in range(1, max_attempts + 1):
        completed_process = subprocess.run(command, cwd=configuration_directory)
        if completed_process.returncode == 0:
            return
        if (
            not _is_sigbus_return_code(completed_process.returncode)
            or attempt == max_attempts
        ):
            raise subprocess.CalledProcessError(completed_process.returncode, command)
        print(
            f"Doxygen terminated with SIGBUS while processing {config_path}; "
            f"retrying ({attempt + 1}/{max_attempts})",
            file=sys.stderr,
        )
        shutil.rmtree(config_path.parent / "xml", ignore_errors=True)


def _get_doxygen_version(doxygen_executable):
    """Return the version string reported by the selected Doxygen executable."""
    completed_process = subprocess.run(
        [doxygen_executable, "--version"],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed_process.stdout.strip()


def _resolve_project_inputs(configuration_directory, source_directory, files):
    """Resolve and validate the configured input files for one project."""
    source_root = (configuration_directory / source_directory).resolve()
    input_paths = [(source_root / file_name).resolve() for file_name in files]
    missing_inputs = [path for path in input_paths if not path.is_file()]
    if missing_inputs:
        missing_list = ", ".join(str(path) for path in missing_inputs)
        raise FileNotFoundError(f"Missing Doxygen project input files: {missing_list}")
    return input_paths


def build_doxygen_projects(
    projects_source,
    configuration_directory,
    cache_directory,
    options=None,
    aliases=None,
    doxygen_executable=None,
):
    """Update per-project Doxygen XML caches and return Breathe project paths.

    :param projects_source: Mapping of project names to source directories and files.
    :param configuration_directory: Base for source paths and relative Doxygen settings.
    :param cache_directory: Persistent directory containing per-project XML caches.
    :param options: Additional Doxygen configuration values.
    :param aliases: Additional Doxygen aliases.
    :param doxygen_executable: Optional explicit path to the Doxygen executable.
    :return: Mapping suitable for Breathe's ``breathe_projects`` setting.
    """
    configuration_root = Path(configuration_directory).resolve()
    cache_root = Path(cache_directory).resolve()
    cache_root.mkdir(parents=True, exist_ok=True)
    options = options or {}
    aliases = aliases or {}

    if not projects_source:
        return {}

    if doxygen_executable is None:
        doxygen_executable = shutil.which("doxygen")
    if not doxygen_executable:
        raise FileNotFoundError("Unable to find the Doxygen executable")
    doxygen_version = _get_doxygen_version(doxygen_executable)

    breathe_projects = {}
    generated_projects = 0
    reused_projects = 0

    for project_name, (source_directory, files) in sorted(projects_source.items()):
        cache_name = _project_cache_name(project_name)
        project_root = cache_root / cache_name
        xml_root = project_root / "xml"
        input_paths = _resolve_project_inputs(
            configuration_root, Path(source_directory), files
        )
        # Hash a stable output path; the temporary staging name must not affect
        # cache reuse. Doxygen owns relative input resolution, not output placement.
        config = _create_doxygen_config(
            project_name, input_paths, options, aliases, project_root
        )
        # Environment references and file-inclusion commands in aliases depend
        # on external content. Let Doxygen resolve them on every build.
        dependencies = (
            None if "$(" in config or _DOCUMENTATION_FILE_COMMAND.search(config)
            else _project_dependencies(input_paths, options, configuration_root)
        )
        fingerprint = (
            _project_fingerprint(config, dependencies, doxygen_version, configuration_root)
            if dependencies is not None else None
        )

        if fingerprint is not None and _cache_is_current(project_root, fingerprint):
            reused_projects += 1
        else:
            with tempfile.TemporaryDirectory(
                prefix=".doxygen-staging-", dir=cache_root
            ) as staging_directory:
                staged_project_root = Path(staging_directory) / cache_name
                staged_project_root.mkdir(parents=True)
                staged_config = staged_project_root / "Doxyfile"
                staged_config.write_text(
                    _create_doxygen_config(
                        project_name, input_paths, options, aliases, staged_project_root
                    ),
                    encoding="utf8",
                )
                _run_doxygen(doxygen_executable, staged_config, configuration_root)

                staged_xml_root = staged_project_root / "xml"
                if not (staged_xml_root / "index.xml").is_file():
                    raise RuntimeError(
                        f"Doxygen did not generate index.xml for project {project_name}"
                    )
                # Breathe's saved environment tracks individual XML paths. Keep
                # obsolete XML files until ``make clean`` so its outdated-file
                # check can still inspect them before purging removed documents.
                # Invalidate the old state before touching live XML. If copying
                # is interrupted, even reverting the source must regenerate it.
                state_path = project_root / "cache-state.json"
                state_path.unlink(missing_ok=True)
                sync_generated_tree(staged_xml_root, xml_root, remove_stale=False)

                output_files = sorted(
                    path.relative_to(staged_xml_root).as_posix()
                    for path in staged_xml_root.rglob("*")
                    if path.is_file()
                )
                cache_state = json.dumps(
                    {
                        "fingerprint": fingerprint,
                        "output_files": output_files,
                        "project": project_name,
                    },
                    indent=2,
                    sort_keys=True,
                ) + "\n"
                _write_text_if_changed(state_path, cache_state)
                generated_projects += 1

        _write_text_if_changed(project_root / "Doxyfile", config)
        breathe_projects[project_name] = str(xml_root)

    print(
        f"Doxygen XML cache: {generated_projects} generated, "
        f"{reused_projects} reused"
    )
    return breathe_projects


def _configure_doxygen_projects(app, config):
    """Generate XML using effective settings before Sphinx loads its environment."""
    # Make-mode and BSK's single-page target both put builder output beneath
    # BUILDDIR. Keep XML beside that output, never inside the published HTML.
    cache_directory = Path(app.outdir).parent / "doxygen-cache"
    projects = build_doxygen_projects(
        config.bsk_doxygen_projects_source,
        app.confdir,
        cache_directory,
        options=config.breathe_doxygen_config_options,
        aliases=config.breathe_doxygen_aliases,
    )
    config.breathe_projects = {**config.breathe_projects, **projects}


def _merge_breathe_file_state(app, env, docnames, other):
    """Merge XML dependencies recorded by Breathe in one parallel worker.

    Breathe 4.36 does not merge its own file state. Its existing purge and
    outdated-file handlers can manage the merged records without modification.
    """
    worker_state = getattr(other, "breathe_file_state", {})
    if not worker_state:
        return
    if not hasattr(env, "breathe_file_state"):
        env.breathe_file_state = {}
    worker_docnames = set(docnames)
    for filename, (mtime, dependent_docs) in worker_state.items():
        # Workers inherit other documents' records. Only merge documents that
        # this worker actually read, so stale inherited records stay purged.
        contributed_docs = dependent_docs & worker_docnames
        if not contributed_docs:
            continue
        previous_mtime, previous_docs = env.breathe_file_state.get(filename, (mtime, set()))
        # Preserve every consumer and the oldest observation if times differ;
        # a newer observation must not hide another document's stale XML.
        env.breathe_file_state[filename] = (
            min(previous_mtime, mtime), previous_docs | contributed_docs
        )


def setup(app):
    """Register per-project XML generation after configuration overrides apply."""
    app.setup_extension("breathe")
    app.add_config_value("bsk_doxygen_projects_source", {}, "env")
    app.connect("config-inited", _configure_doxygen_projects, priority=800)
    app.connect("env-merge-info", _merge_breathe_file_state)
    # Reread older Sphinx environments that may have lost worker dependencies.
    # This does not invalidate the independently versioned Doxygen XML cache.
    return {"env_version": 1, "parallel_read_safe": True, "parallel_write_safe": True}

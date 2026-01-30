from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from typing import Any, Optional

try:
    import tomllib  # py3.11+
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib


@dataclass(frozen=True)
class _ModuleSpec:
    name: str
    type: str
    import_path: str
    aliases: tuple[str, ...] = ()
    factory_attr: Optional[str] = None
    callable_attr: Optional[str] = None
    register_payloads_attr: Optional[str] = None


def _read_manifest_text(manifest: Any) -> str:
    if hasattr(manifest, "read_text"):
        return manifest.read_text(encoding="utf-8")
    with manifest.open("r", encoding="utf-8") as f:
        return f.read()


def _parse_manifest(text: str, source: str = "") -> list[_ModuleSpec]:
    cfg = tomllib.loads(text)

    version = cfg.get("version", 1)
    if version != 1:
        raise ValueError(f"Unsupported manifest version {version} ({source})")

    modules = cfg.get("modules")
    if not isinstance(modules, list) or not modules:
        raise ValueError(f"Manifest has no [[modules]] entries ({source})")

    out: list[_ModuleSpec] = []
    for i, m in enumerate(modules):
        if not isinstance(m, dict):
            raise TypeError(f"modules[{i}] must be a table/dict ({source})")

        name = m.get("name")
        mtype = m.get("type")
        import_path = m.get("import")

        if not name or not isinstance(name, str):
            raise ValueError(f"modules[{i}].name must be a non-empty string ({source})")
        if mtype not in ("pybind", "python"):
            raise ValueError(
                f"modules[{i}].type must be 'pybind' or 'python' ({source})"
            )
        if not import_path or not isinstance(import_path, str):
            raise ValueError(
                f"modules[{i}].import must be a non-empty string ({source})"
            )

        aliases = m.get("aliases") or []
        if isinstance(aliases, str):
            aliases = [aliases]
        if not isinstance(aliases, list) or any(
            (not isinstance(a, str) or not a) for a in aliases
        ):
            raise ValueError(
                f"modules[{i}].aliases must be a list of non-empty strings ({source})"
            )

        factory_attr = m.get("factory")
        callable_attr = m.get("callable")
        register_payloads_attr = m.get("register_payloads")

        out.append(
            _ModuleSpec(
                name=name,
                type=mtype,
                import_path=import_path,
                aliases=tuple(aliases),
                factory_attr=factory_attr if isinstance(factory_attr, str) else None,
                callable_attr=callable_attr if isinstance(callable_attr, str) else None,
                register_payloads_attr=register_payloads_attr
                if isinstance(register_payloads_attr, str)
                else None,
            )
        )
    return out


def register_from_manifest(manifest: Any, registry: Any, source: str = "") -> None:
    """
    Read a TOML manifest and populate registry with canonical load() names.

    `registry` is expected to be Basilisk.plugins.PluginRegistry (kept as Any to avoid import cycles).
    """
    text = _read_manifest_text(manifest)
    specs = _parse_manifest(text, source=source)

    for spec in specs:
        mod = import_module(spec.import_path)

        def _register_name(n: str, obj: Any) -> None:
            # In the new world, prefer everything as a factory so load() is uniform.
            registry.register_factory(n, obj)

        if spec.type == "pybind":
            # optional: payload type registry hook
            if spec.register_payloads_attr:
                hook = getattr(mod, spec.register_payloads_attr, None)
                if hook is not None and callable(hook):
                    hook()

            factory_attr = spec.factory_attr or "create_factory"
            create_factory = getattr(mod, factory_attr, None)
            if not callable(create_factory):
                raise TypeError(
                    f"{source}: '{spec.import_path}.{factory_attr}' is not callable"
                )

            # IMPORTANT: create_factory() must return a zero-arg factory that returns instances
            factory = create_factory()
            if not callable(factory):
                raise TypeError(
                    f"{source}: '{spec.import_path}.{factory_attr}()' did not return a callable factory"
                )

            registry.register_factory(spec.name, factory)
            for a in spec.aliases:
                registry.register_factory(a, factory)

        elif spec.type == "python":
            # python modules are still registered as factories (callable returning instance)
            callable_attr = spec.callable_attr
            if not callable_attr:
                raise ValueError(
                    f"{source}: python module '{spec.name}' requires 'callable'"
                )

            ctor = getattr(mod, callable_attr, None)
            if not callable(ctor):
                raise TypeError(
                    f"{source}: '{spec.import_path}.{callable_attr}' is not callable"
                )

            _register_name(spec.name, ctor)
            for a in spec.aliases:
                _register_name(a, ctor)

        else:
            raise RuntimeError("unreachable")

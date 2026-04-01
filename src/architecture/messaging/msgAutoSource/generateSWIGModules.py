import json
import sys

# Maps numeric C/C++ types to numpy dtype enum names
C_TYPE_TO_NPY_ENUM = {
    # Signed integers
    "int8_t": "NPY_INT8",

    "short": "NPY_INT16",
    "short int": "NPY_INT16",
    "signed short": "NPY_INT16",
    "signed short int": "NPY_INT16",
    "int16_t": "NPY_INT16",

    "int": "NPY_INT32",
    "signed int": "NPY_INT32",
    "int32_t": "NPY_INT32",

    "long": "NPY_LONG",
    "long int": "NPY_LONG",
    "signed long": "NPY_LONG",
    "signed long int": "NPY_LONG",

    "long long": "NPY_INT64",
    "long long int": "NPY_INT64",
    "signed long long": "NPY_INT64",
    "signed long long int": "NPY_INT64",
    "int64_t": "NPY_INT64",

    # Unsigned integers
    "uint8_t": "NPY_UINT8",

    "unsigned short": "NPY_UINT16",
    "unsigned short int": "NPY_UINT16",
    "uint16_t": "NPY_UINT16",

    "unsigned": "NPY_UINT32",
    "unsigned int": "NPY_UINT32",
    "uint32_t": "NPY_UINT32",

    "unsigned long": "NPY_ULONG",
    "unsigned long int": "NPY_ULONG",

    "unsigned long long": "NPY_UINT64",
    "unsigned long long int": "NPY_UINT64",
    "uint64_t": "NPY_UINT64",

    # Platform-size signed/unsigned integers
    "intptr_t": "NPY_INTP",
    "uintptr_t": "NPY_UINTP",
    "ptrdiff_t": "NPY_INTP",
    "ssize_t": "NPY_INTP",
    "size_t": "NPY_UINTP",

    # Floating point types
    "float16": "NPY_FLOAT16",
    "half": "NPY_FLOAT16",
    "float": "NPY_FLOAT32",
    "float32": "NPY_FLOAT32",
    "double": "NPY_FLOAT64",
    "float64": "NPY_FLOAT64",
    "long double": "NPY_LONGDOUBLE",
}

# Integer C types (maps to Python int)
_INT_CTYPES = {
    "int8_t", "int16_t", "int32_t", "int64_t",
    "short", "short int", "signed short", "signed short int",
    "int", "signed int",
    "long", "long int", "signed long", "signed long int",
    "long long", "long long int", "signed long long", "signed long long int",
    "uint8_t", "uint16_t", "uint32_t", "uint64_t",
    "unsigned short", "unsigned short int",
    "unsigned", "unsigned int",
    "unsigned long", "unsigned long int",
    "unsigned long long", "unsigned long long int",
    "intptr_t", "uintptr_t", "ptrdiff_t", "ssize_t", "size_t",
}

# Floating-point C types (maps to Python float)
_FLOAT_CTYPES = {
    "float16", "half", "float", "float32",
    "double", "float64", "long double",
}


def _fieldToMacro(field: dict, structName: str, rollback: bool) -> str:
    """
    Convert one JSON field descriptor into the appropriate RECORDER_PROPERTY macro line.

    Macro selection:
      - Numeric scalar  -> RECORDER_PROPERTY_NUMERIC_0
      - Numeric 1-D array -> RECORDER_PROPERTY_NUMERIC_1
      - Numeric 2-D array -> RECORDER_PROPERTY_NUMERIC_2
      - Everything else   -> RECORDER_PROPERTY  (skipped when rollback is True)
    """
    kind = field["kind"]
    name = field["name"]

    if kind == "primitive":
        ctype = field["ctype"]
        if ctype in C_TYPE_TO_NPY_ENUM:
            npy = C_TYPE_TO_NPY_ENUM[ctype]
            return f"RECORDER_PROPERTY_NUMERIC_0({structName}, {name}, {ctype}, {npy});\n"
        if not rollback:
            return f"RECORDER_PROPERTY({structName}, {name}, ({ctype}));\n"

    elif kind == "pointer":
        if not rollback:
            return f"RECORDER_PROPERTY({structName}, {name}, ({field['ctype']}));\n"

    elif kind == "array":
        shape     = field["shape"]
        nDims     = len(shape)
        elem      = field["element"]
        # Primitive/pointer/unknown elements carry "ctype"; struct/enum elements
        # carry "typeName" instead.  Fall back gracefully.
        elemCtype = elem.get("ctype") or elem.get("typeName", "")
        if elemCtype in C_TYPE_TO_NPY_ENUM and nDims < 3:
            npy = C_TYPE_TO_NPY_ENUM[elemCtype]
            return f"RECORDER_PROPERTY_NUMERIC_{nDims}({structName}, {name}, {elemCtype}, {npy});\n"
        if not rollback:
            dims = "".join(f"[{d}]" for d in shape)
            return f"RECORDER_PROPERTY({structName}, {name}, ({elemCtype}{dims}));\n"

    elif kind in ("struct", "enum"):
        if not rollback:
            typeName = field.get("typeName") or field.get("ctype", "")
            return f"RECORDER_PROPERTY({structName}, {name}, ({typeName}));\n"

    elif kind == "unknown":
        if not rollback:
            ctype = field.get("ctype", "")
            if ctype:
                return f"RECORDER_PROPERTY({structName}, {name}, ({ctype}));\n"

    return ""


def parseMetaJson(meta: dict, targetStructName: str, recorderPropertyRollback: bool) -> str:
    """
    Emit RECORDER_PROPERTY macros for all struct fields described in *meta*.

    Parameters:
        meta (dict): Parsed JSON metadata produced by generatePayloadMetaJson.py.
        targetStructName (str): Name of the payload (e.g. 'MTBMsgPayload').
        recorderPropertyRollback (bool): If True, non-numeric fields are omitted
            (recovering the legacy output format).

    Returns:
        str: macro declarations to be pasted into msgInterfacePy.i.in
    """
    return "".join(
        _fieldToMacro(field, targetStructName, recorderPropertyRollback)
        for field in meta.get("fields", [])
    )


def _fieldRwTypes(field: dict) -> tuple[str, str] | None:
    """
    Return the (read_type, write_type) Python type annotation strings for *field*,
    or None if the field has no meaningful Python type annotation.

    read_type  is used as the @property getter return annotation.
    write_type is used as the @property setter parameter annotation and as the
               __init__ keyword-argument annotation.

    Arrays use asymmetric types: List[T] for reads, Sequence[T] for writes.
    """
    kind = field["kind"]

    if kind == "primitive":
        ctype = field["ctype"].split(" /*")[0].strip()
        if ctype == "bool":
            return ("bool", "bool")
        if ctype in _INT_CTYPES:
            return ("int", "int")
        if ctype in _FLOAT_CTYPES:
            return ("float", "float")
        return None

    if kind == "array":
        elemCtype = field["element"].get("ctype", "")
        if elemCtype in _INT_CTYPES:
            return ("List[int]", "Sequence[int]")
        if elemCtype in _FLOAT_CTYPES:
            return ("List[float]", "Sequence[float]")
        return None

    if kind == "enum":
        return ("int", "int")

    if kind == "struct":
        typeName = field.get("typeName") or field.get("ctype", "")
        return (f'"{typeName}"', f'"{typeName}"') if typeName else None

    return None  # pointer, unknown


def _shapeStr(shape: list[int]) -> str:
    """Format an array shape as a Python tuple literal string."""
    if len(shape) == 1:
        return f"({shape[0]},)"
    return "(" + ", ".join(str(d) for d in shape) + ")"


def _generateShadowInit(meta: dict, payloadType: str) -> str:
    """
    Generate a ``%feature("shadow")`` block that replaces the SWIG-generated
    constructor with a typed, keyword-only ``__init__``.

    Must be emitted BEFORE ``%include "...Payload.h"`` so SWIG sees it when
    it parses the struct declaration.
    """
    fields = meta.get("fields", [])
    mod = f"_{payloadType}"
    sortedFields = sorted(fields, key=lambda f: f["name"])

    lines = [f'%feature("shadow") {payloadType}::{payloadType}() %{{']

    # --- signature ---
    paramParts = []
    for f in sortedFields:
        rw = _fieldRwTypes(f)
        if rw:
            paramParts.append(f"{f['name']}: {rw[1]} = ...")
        else:
            paramParts.append(f"{f['name']} = ...")

    if paramParts:
        paramsStr = (",\n" + " " * 13).join(paramParts)
        lines.append(f"def __init__(self, *,")
        lines.append(f"             {paramsStr}) -> None:")
    else:
        lines.append("def __init__(self) -> None:")

    # --- docstring ---
    docLines = []
    for f in sortedFields:
        rw = _fieldRwTypes(f)
        c  = f.get("comment", "").strip()
        if rw:
            t = rw[1]
            if f["kind"] == "array":
                shapeInfo = f", shape {_shapeStr(f['shape'])}"
                docLines.append(f"        {f['name']} ({t}): {c + shapeInfo if c else 'shape ' + _shapeStr(f['shape'])}")
            else:
                docLines.append(f"        {f['name']} ({t}){': ' + c if c else ''}")
        else:
            docLines.append(f"        {f['name']}{': ' + c if c else ''}")

    lines.append('    """Constructs a new payload, zero\'d by default.')
    lines.append("")
    lines.append("    Keyword arguments can be passed to initialize the fields of this payload.")
    if docLines:
        lines.append("")
        lines.append("    Args:")
        lines.extend(docLines)
    lines.append('    """')

    # --- body ---
    lines.append(f"    {mod}.{payloadType}_swiginit(")
    lines.append(f"        self, {mod}.new_{payloadType}())")
    for f in sortedFields:
        n = f["name"]
        lines.append(f"    if {n} is not ...:")
        lines.append(f"        self.{n} = {n}")

    lines.append("%}")
    lines.append("")

    return "\n".join(lines)


def _generateExtendBlock(meta: dict, payloadType: str) -> str:
    """
    Generate a SWIG ``%extend PayloadType { %pythoncode %{ ... %} }`` block
    that augments the SWIG-generated proxy class with:

    - Typed ``@property`` descriptors (getter -> List/scalar, setter ← Sequence/scalar)
      with field descriptions as docstrings.  Fields that carry neither a type
      annotation nor a comment are left as-is (SWIG's ``_swig_property``).
    - A typed ``__init__`` that accepts keyword arguments for all annotatable fields.
    - A precomputed ``__fields__`` classmethod returning an alphabetically sorted
      tuple of all field names (replacing the inspect.getmembers scan).
    """
    fields = meta.get("fields", [])
    mod = f"_{payloadType}"
    sortedFields = sorted(fields, key=lambda f: f["name"])

    lines = [f"%extend {payloadType} {{", "%pythoncode %{"]

    # --- @property descriptors ---
    for f in sortedFields:
        name    = f["name"]
        comment = f.get("comment", "").strip()
        rw      = _fieldRwTypes(f)

        if rw is None and not comment:
            continue  # nothing to add, leave SWIG's _swig_property intact

        readT, writeT = rw if rw else (None, None)

        if f["kind"] == "array":
            doc = f"{comment}, shape {_shapeStr(f['shape'])}" if comment \
                  else f"shape {_shapeStr(f['shape'])}"
        else:
            doc = comment

        retAnn = f" -> {readT}" if readT else ""
        valAnn = f": {writeT}" if writeT else ""

        lines.append(f"    @property")
        lines.append(f"    def {name}(self){retAnn}:")
        if doc:
            lines.append(f'        """{doc}"""')
        lines.append(f"        return {mod}.{payloadType}_{name}_get(self)")
        lines.append(f"    @{name}.setter")
        lines.append(f"    def {name}(self, value{valAnn}) -> None:")
        lines.append(f"        {mod}.{payloadType}_{name}_set(self, value)")
        lines.append("")

    # --- __fields__ ---
    if not sortedFields:
        namesTuple = "()"
    elif len(sortedFields) == 1:
        namesTuple = f"({sortedFields[0]['name']!r},)"
    else:
        namesTuple = "(" + ", ".join(repr(f["name"]) for f in sortedFields) + ")"

    lines.append("    @classmethod")
    lines.append("    def __fields__(cls) -> tuple:")
    lines.append('        """Returns a tuple with all the payload\'s field names (alphabetical)."""')
    lines.append(f"        return {namesTuple}")
    lines.append("")
    lines.append("%}")
    lines.append("}")
    lines.append("")

    return "\n".join(lines)


if __name__ == "__main__":
    moduleOutputPath = sys.argv[1]
    headerinputPath = sys.argv[2]
    payloadTypeName = sys.argv[3]
    structType = payloadTypeName.split('Payload')[0]
    baseDir = sys.argv[4]
    generateCInfo = sys.argv[5] == 'True'
    metaJsonPath = sys.argv[6]
    recorderPropertyRollback = bool(int(sys.argv[7]))

    with open('msgInterfacePy.i.in', 'r') as f:
        swigTemplateData = f.read()

    with open('cMsgCInterfacePy.i.in', 'r') as f:
        swigCTemplateData = f.read()

    with open(metaJsonPath) as f:
        meta = json.load(f)

    extraContent = parseMetaJson(meta, payloadTypeName, recorderPropertyRollback)
    extraPythonContent = _generateExtendBlock(meta, payloadTypeName)
    extraShadowFeature = _generateShadowInit(meta, payloadTypeName)

    with open(moduleOutputPath, 'w') as moduleFileOut:
        moduleFileOut.write(swigTemplateData.format(
            type=structType, baseDir=baseDir,
            extraContent=extraContent,
            extraPythonContent=extraPythonContent,
            extraShadowFeature=extraShadowFeature,
        ))
        if generateCInfo:
            moduleFileOut.write(swigCTemplateData.format(type=structType))

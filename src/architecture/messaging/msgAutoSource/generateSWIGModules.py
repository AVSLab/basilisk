import sys
import re
from typing import Optional, Tuple, Callable, List
import xml.etree.ElementTree as ET

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

def cleanSwigType(cppType: str) -> str:
    """
    Cleans and normalizes a C++ template type string extracted from SWIG-generated XML,
    restoring valid syntax and formatting for use in generated C++ or Python bindings.

    This function removes redundant parentheses and correctly reconstructs:
    - Nested template structures (e.g., std::vector<std::pair<int, float>>)
    - Function signatures within std::function (e.g., std::function<void(int)>)
    - Multi-word C++ types (e.g., unsigned long long, const float&)
    - Pointer and reference types (e.g., T*, T&, T&&)
    - Fixed-size C-style arrays (e.g., int[10], const float(&)[3])

    Parameters:
        cppType (str): The raw type string from SWIG XML, possibly containing spurious
                       parentheses and HTML-escaped characters (&lt;, &gt;).

    Returns:
        str: A cleaned-up and properly formatted C++ type string.
    """
    import html
    import re

    # Decode any XML-escaped characters (e.g., &lt; to <)
    cppType = html.unescape(cppType)

    # Tokenize while preserving C++ symbols
    tokens = re.findall(r'\w+::|::|\w+|<|>|,|\(|\)|\[|\]|\*|&|&&|\S', cppType)

    def stripParensIfWrapped(typeStr: str) -> str:
        """
        Remove surrounding parentheses if they are redundant and balanced.

        Args:
            typeStr: A potentially parenthesized type string

        Returns:
            str: The string with parentheses removed if they are unnecessary
        """
        typeStr = typeStr.strip()
        if typeStr.startswith('(') and typeStr.endswith(')'):
            depth = 0
            for i, ch in enumerate(typeStr):
                if ch == '(':
                    depth += 1
                elif ch == ')':
                    depth -= 1
                    # If we close early, parentheses are internal – keep them
                    if depth == 0 and i != len(typeStr) - 1:
                        return typeStr
            return typeStr[1:-1].strip()
        return typeStr

    def isWord(token: str) -> bool:
        """Return True if the token is a C++ identifier (e.g., 'int', 'const')"""
        return re.fullmatch(r'\w+', token) is not None

    def parseType(index: int) -> Tuple[str, int]:
        """
        Recursively parses a C++ type expression from tokens.

        Args:
            index: Current token index

        Returns:
            tuple:
                str: parsed type string
                int: next token index
        """
        parts = []
        while index < len(tokens):
            token = tokens[index]

            if token == '<':
                index += 1
                inner, index = parseBracketBlock(index, '<', '>', parseType)
                parts.append(f"<{inner}>")

            elif token == '(':
                index += 1
                inner, index = parseBracketBlock(index, '(', ')', parseType)
                parts.append(f"({inner})")

            elif token in [')', '>']:
                break

            elif token == ',':
                index += 1
                break

            else:
                # Add space only between adjacent words
                if parts:
                    prev = parts[-1]
                    if isWord(prev) and isWord(token):
                        parts.append(' ')
                parts.append(token)
                index += 1

        return ''.join(parts), index

    def parseBracketBlock(index: int, openSym: str, closeSym: str,
                          parseFunc: Callable[[int], Tuple[str, int]]) -> Tuple[str, int]:
        """
        Parses a bracketed block like <...> or (...) or [...] with nested content.

        Args:
            index: Current token index (after the opening symbol)
            openSym: Opening symbol, e.g. '<'
            closeSym: Closing symbol, e.g. '>'
            parseFunc: Recursive parse function (e.g., parseType)

        Returns:
            tuple:
                str: parsed type string
                int: next token in
        """
        items: List[str] = []
        while index < len(tokens):
            if tokens[index] == closeSym:
                cleaned = [stripParensIfWrapped(i) for i in items]
                return ', '.join(cleaned), index + 1
            elif tokens[index] == ',':
                index += 1  # skip separator
            else:
                item, index = parseFunc(index)
                items.append(item)
        return ', '.join(items), index

    cleaned, _ = parseType(0)
    return cleaned.strip()

def parseSwigDecl(decl: str):
    """
    Parses a SWIG `decl` string and converts it into components of a C++ declarator.

    SWIG represents C++ type modifiers (pointers, references, arrays, functions) using a dot-delimited
    postfix syntax attached to the base type. This function interprets those tokens and produces
    the corresponding C++-style declarator suffix elements.

    SWIG encoding conventions:
      - `p.`       : pointer (adds a `*`)
      - `r.`       : reference (adds a `&`)
      - `a(N).`    : fixed-size array of N elements (adds `[N]`)
      - `f().`     : function type (currently ignored by this parser)

    Modifier order is postfix but applies from inside-out:
        Example: `a(3).p.` means "pointer to array of 3" -> `*[3]`
                 `p.a(3).` means "array of 3 pointers" -> `[3]*`

    Parameters:
        decl (str): The SWIG-encoded declarator string. Examples:
                    - "p."       -> pointer
                    - "r."       -> reference
                    - "a(5)."    -> array of 5 elements
                    - "a(2).a(2).p.p." -> pointer to pointer to 2x2 array

    Returns:
        tuple:
            pointerPart (str): a string of '*' characters for pointer depth (e.g., '**')
            referencePart (str): '&' if this is a reference, else ''
            arrayParts (list[str]): array dimensions as strings, ordered from outermost to innermost.
                                    For example, ['2', '3'] for `a(2).a(3).` means `[2][3]`.

    Example:
        >>> parseSwigDecl("a(3).p.")
        ('*', '', ['3'])      # pointer to array of 3 -> '*[3]'

        >>> parseSwigDecl("p.a(3).")
        ('*', '', ['3'])      # array of 3 pointers -> '[3]*'

        >>> parseSwigDecl("p.p.a(2).a(2).r.")
        ('**', '&', ['2', '2'])  # reference to pointer to pointer to 2x2 array
    """
    pointerPart = ''
    referencePart = ''
    arrayParts = []

    # Match each declarator component
    tokens = re.findall(r'(a\([^)]+\)|p\.|r\.|f\(\)\.)', decl)

    for token in tokens:
        if token.startswith('a('):
            # Array: a(N) -> N
            size = re.match(r'a\(([^)]+)\)', token).group(1)
            arrayParts.append(size)
        elif token == 'p.':
            pointerPart += '*'
        elif token == 'r.':
            referencePart = '&'  # References appear after pointer
        # Note: We skip f(). (function pointer) for now

    return pointerPart, referencePart, arrayParts

def parseSwigXml(xmlPath: str, targetStructName: str, cpp: bool, recorderPropertyRollback: bool):
    """
    Parses a SWIG-generated XML file and emits RECORDER_PROPERTY macros
    for all struct/class member fields.

    Parameters:
        xmlPath (str): Path to the XML file generated with `swig -xml`
        targetStructName (str): Name of the payload (e.g. `MTBMsgPayload`).
        cpp (bool): whether this is a C++ payload (we need to be extra
            careful with the type since it might be templated.)
        recorderPropertyRollback (bool): If true, non-numeric properties
            are not given a special RECORDER_PROPERTY macro, thus recovering
            the legacy output format for these fields.

    Returns:
        str: macro declarations to be pasted into `msgInterfacePy.i.in`
    """
    result = ""

    tree = ET.parse(xmlPath)
    root = tree.getroot()

    # Iterate over all classes (C++ classes and structs)
    for classNode in root.findall(".//class"):
        classAttrs = extractAttributeMap(classNode.find("attributelist"))
        structName = classAttrs.get("name") or classAttrs.get("tdname")
        if structName != targetStructName:
            continue

        # Each field appears as a <cdecl> child with ismember="1"
        for cdecl in classNode.findall("cdecl"):
            fieldAttrs = extractAttributeMap(cdecl.find("attributelist"))
            if fieldAttrs.get("ismember") != "1":
                continue  # Skip non-member declarations

            fieldName = fieldAttrs.get("name")
            baseType = fieldAttrs.get("type")
            decl = fieldAttrs.get("decl", "")

            if not fieldName or not baseType:
                continue

            if cpp:
                baseType = cleanSwigType(baseType)

            typePointerPart, typeReferencePart, typeArrayParts = parseSwigDecl(decl)
            typeWithPointerRef = f"{baseType}{typePointerPart}{typeReferencePart}"

            if typeWithPointerRef in C_TYPE_TO_NPY_ENUM and len(typeArrayParts) < 3:
                npyType = C_TYPE_TO_NPY_ENUM[typeWithPointerRef]
                macroName = f"RECORDER_PROPERTY_NUMERIC_{len(typeArrayParts)}"
                dimensions = "".join(f", {dim}" for dim in typeArrayParts)
                result += f"{macroName}({targetStructName}, {fieldName}, {typeWithPointerRef}, {npyType} {dimensions});\n"
            elif not recorderPropertyRollback:
                fullType = f"{typeWithPointerRef}{''.join(f'[{i}]' for i in typeArrayParts)}"
                result += f"RECORDER_PROPERTY({targetStructName}, {fieldName}, ({fullType}));\n"

    return result

def extractAttributeMap(attributeListNode: Optional[ET.Element]):
    """
    Converts an <attributelist> XML node into a Python dict.

    Parameters:
        attributeListNode (Element | None): The <attributelist> element

    Returns:
        dict[str, str]: Mapping of attribute name -> value
    """
    if attributeListNode is None:
        return {}
    return {
        attr.attrib['name']: attr.attrib['value']
        for attr in attributeListNode.findall("attribute")
        if 'name' in attr.attrib and 'value' in attr.attrib
    }

if __name__ == "__main__":
    moduleOutputPath = sys.argv[1]
    headerinputPath = sys.argv[2]
    payloadTypeName = sys.argv[3]
    structType = payloadTypeName.split('Payload')[0]
    baseDir = sys.argv[4]
    generateCInfo = sys.argv[5] == 'True'
    xmlWrapPath = sys.argv[6]
    recorderPropertyRollback = bool(int(sys.argv[7]))

    with open('msgInterfacePy.i.in', 'r') as f:
        swigTemplateData = f.read()

    with open('cMsgCInterfacePy.i.in', 'r') as f:
        swigCTemplateData = f.read()

    extraContent = parseSwigXml(xmlWrapPath, payloadTypeName, not generateCInfo, recorderPropertyRollback)

    with open(moduleOutputPath, 'w') as moduleFileOut:
        moduleFileOut.write(swigTemplateData.format(type=structType, baseDir=baseDir, extraContent=extraContent))
        if(generateCInfo):
            moduleFileOut.write(swigCTemplateData.format(type=structType))

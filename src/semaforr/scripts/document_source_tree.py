#!/usr/bin/env python3
"""SemaFORR module overview.

Summary:
    This file implements document source tree behavior for developer tooling and experiment automation. It centers on `parse_cpp_callable`, `write_file`, `humanize`, `subsystem_for`, `symbol_names`, `overview`, `wrap_words`, `cpp_file_header`. Its package-relative location is `scripts/document_source_tree.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""



from __future__ import annotations

import argparse
import ast
import json
import re
import subprocess
import time
from pathlib import Path
from typing import Iterable, Sequence


TEXT_SUFFIXES = {
    ".cpp", ".hpp", ".py", ".md", ".yaml", ".yml", ".conf", ".txt",
    ".xml", ".rviz", ".sh", ".json",
}
CPP_SUFFIXES = {".cpp", ".hpp"}
CONTROL_NAMES = {
    "if", "for", "while", "switch", "catch", "return", "throw",
    "sizeof", "alignof", "decltype", "static_assert", "requires",
}
CPP_DECLARATION = re.compile(
    r"^(?P<prefix>.*?)"
    r"(?P<name>(?:[A-Za-z_]\w*::)*~?[A-Za-z_]\w*|operator\s*[^\s(]+)"
    r"\s*\((?P<params>.*?)\)"
    r"(?P<suffix>.*?)(?:;|\{\s*$)",
    re.DOTALL,
)
CPP_TYPE = re.compile(
    r"\b(?P<kind>class|struct|enum\s+class|enum)\s+(?P<name>[A-Za-z_]\w*)"
)


def parse_cpp_callable(candidate: str) -> tuple[str, str, str, str] | None:
    """Summary:
        Parses cpp callable for this subsystem.

    Args:
        candidate (str): Supplies candidate input to the operation.

    Returns:
        tuple[str, str, str, str] | None

    Raises:
        None documented; dependency failures may propagate.
    """
    opening = candidate.find("(")
    if opening < 0:
        return None
    depth = 0
    closing = -1
    for index in range(opening, len(candidate)):
        if candidate[index] == "(":
            depth += 1
        elif candidate[index] == ")":
            depth -= 1
            if depth == 0:
                closing = index
                break
    if closing < 0:
        return None
    head = candidate[:opening].strip()
    name_match = re.search(
        r"((?:[A-Za-z_]\w*::)*~?[A-Za-z_]\w*|operator\s*[^\s(]+)\s*$",
        head,
    )
    if not name_match:
        return None
    name = name_match.group(1).strip()
    prefix = head[:name_match.start()].strip()
    parameters = candidate[opening + 1:closing]
    suffix = candidate[closing + 1:].strip()
    constructor_initializer = suffix.startswith(":")
    parentheses = 0
    brackets = 0
    braces = 0
    terminates = False
    for index, character in enumerate(suffix):
        if character == "(":
            parentheses += 1
        elif character == ")":
            parentheses = max(0, parentheses - 1)
        elif character == "[":
            brackets += 1
        elif character == "]":
            brackets = max(0, brackets - 1)
        elif character == "{" and parentheses == 0 and brackets == 0:
            previous = suffix[:index].rstrip()[-1:] or " "
            if braces == 0 and (not constructor_initializer or
                                previous in {")", "}"}):
                terminates = True
                break
            braces += 1
        elif character == "}" and parentheses == 0 and brackets == 0:
            braces = max(0, braces - 1)
        elif character == ";" and parentheses == 0 and brackets == 0 and braces == 0:
            terminates = True
            break
    if not terminates:
        return None
    return name, parameters, prefix, suffix


def write_file(path: Path, text: str) -> None:
    """Summary:
        Writes file for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        None; effects are applied to owned state or outputs.

    Raises:
        error: If required input or state is invalid.
    """
    error: OSError | None = None
    for attempt in range(5):
        try:
            path.write_text(text, encoding="utf-8", newline="\n")
            return
        except OSError as caught:
            error = caught
            time.sleep(0.05 * (attempt + 1))
    assert error is not None
    raise error


def humanize(name: str) -> str:
    """Summary:
        Performs the humanize operation for this subsystem.

    Args:
        name (str): Supplies name input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    value = re.sub(r"([a-z0-9])([A-Z])", r"\1 \2", name)
    value = re.sub(r"[^A-Za-z0-9]+", " ", value).strip().lower()
    return value or "package content"


def subsystem_for(path: Path, root: Path) -> str:
    """Summary:
        Performs the subsystem for operation for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        root (Path): Supplies root input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    relative = path.relative_to(root)
    parts = relative.parts
    if "test" in parts:
        return "automated verification and regression testing"
    if "decision" in parts:
        return "tiered decision making and action arbitration"
    if "planning" in parts:
        return "path planning and hierarchical plan construction"
    if "exploration" in parts:
        return "initial or reactive exploration"
    if "spatial" in parts:
        return "learned spatial representations and their lifecycle"
    if "social" in parts:
        return "social observation processing and crowd learning"
    if "ros" in parts:
        return "the ROS 2 composition and message-adaptation boundary"
    if "validation" in parts:
        return "replay, experimental validation, and performance measurement"
    if "domain" in parts:
        return "ROS-independent domain state and value types"
    if "config" in parts:
        return "runtime configuration and reproducible experiment setup"
    if "launch" in parts:
        return "ROS 2 launch composition"
    if "scripts" in parts:
        return "developer tooling and experiment automation"
    if "docs" in parts:
        return "the maintained architecture and user documentation"
    return "the SemaFORR navigation package"


def symbol_names(text: str, suffix: str) -> list[str]:
    """Summary:
        Performs the symbol names operation for this subsystem.

    Args:
        text (str): Supplies text input to the operation.
        suffix (str): Supplies suffix input to the operation.

    Returns:
        list[str]

    Raises:
        None documented; dependency failures may propagate.
    """
    names: list[str] = []
    if suffix == ".py":
        tree = ast.parse(text)
        names = [
            node.name for node in ast.walk(tree)
            if isinstance(node, (ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef))
        ]
    elif suffix in CPP_SUFFIXES:
        names.extend(match.group("name") for match in CPP_TYPE.finditer(text))
        names.extend(
            match.group(1).split("::")[-1]
            for match in re.finditer(
                r"\b(?:TEST|TEST_F|TEST_P)\s*\(\s*[^,]+,\s*([A-Za-z_]\w*)",
                text,
            )
        )
    return list(dict.fromkeys(names))[:8]


def overview(path: Path, root: Path, text: str) -> str:
    """Summary:
        Performs the overview operation for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        root (Path): Supplies root input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    relative = path.relative_to(root).as_posix()
    subject = humanize(path.stem if path.name != "CMakeLists.txt" else "build")
    subsystem = subsystem_for(path, root)
    names = symbol_names(text, path.suffix.lower())
    detail = (
        f"It centers on {', '.join(f'`{name}`' for name in names)}."
        if names else
        "It records the declarations, settings, fixtures, or guidance needed "
        "by that responsibility."
    )
    if "/test/" in f"/{relative}":
        role = "exercises"
    elif path.suffix.lower() in {".hpp", ".yaml", ".conf", ".json", ".xml"}:
        role = "defines"
    elif path.suffix.lower() in {".md", ".txt", ".rviz"}:
        role = "documents or configures"
    else:
        role = "implements"
    return (
        f"This file {role} {subject} behavior for {subsystem}. {detail} "
        f"Its package-relative location is `{relative}`."
    )


def wrap_words(text: str, width: int = 76) -> list[str]:
    """Summary:
        Performs the wrap words operation for this subsystem.

    Args:
        text (str): Supplies text input to the operation.
        width (int): Supplies width input to the operation.

    Returns:
        list[str]

    Raises:
        None documented; dependency failures may propagate.
    """
    lines: list[str] = []
    current = ""
    for word in text.split():
        candidate = f"{current} {word}".strip()
        if current and len(candidate) > width:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    return lines


def cpp_file_header(path: Path, root: Path, text: str) -> str:
    """Summary:
        Performs the cpp file header operation for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        root (Path): Supplies root input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    lines = ["/**", f" * @file {path.name}",
             f" * @brief {humanize(path.stem).capitalize()} responsibilities.", " *"]
    for line in wrap_words(overview(path, root, text), 72):
        lines.append(f" * @details {line}" if lines[-1] == " *" else f" * {line}")
    lines.extend([" */", ""])
    return "\n".join(lines)


def clean_cpp(text: str) -> str:
    """Summary:
        Performs the clean cpp operation for this subsystem.

    Args:
        text (str): Supplies text input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    result = list(text)
    index = 0
    state = "code"
    quote = ""
    while index < len(text):
        pair = text[index:index + 2]
        if state == "code" and pair == "//":
            state = "line"
            result[index:index + 2] = "  "
            index += 2
            continue
        if state == "code" and pair == "/*":
            state = "block"
            result[index:index + 2] = "  "
            index += 2
            continue
        if state == "code" and text[index] in {'"', "'"}:
            state = "string"
            quote = text[index]
            result[index] = " "
            index += 1
            continue
        if state == "line":
            if text[index] == "\n":
                state = "code"
            else:
                result[index] = " "
            index += 1
            continue
        if state == "block":
            if pair == "*/":
                result[index:index + 2] = "  "
                index += 2
                state = "code"
            else:
                if text[index] != "\n":
                    result[index] = " "
                index += 1
            continue
        if state == "string":
            if text[index] == "\\" and index + 1 < len(text):
                if text[index] != "\n":
                    result[index] = " "
                if text[index + 1] != "\n":
                    result[index + 1] = " "
                index += 2
            elif text[index] == quote:
                result[index] = " "
                index += 1
                state = "code"
            else:
                if text[index] != "\n":
                    result[index] = " "
                index += 1
            continue
        index += 1
    return "".join(result)


def split_cpp_params(parameters: str) -> list[str]:
    """Summary:
        Performs the split cpp params operation for this subsystem.

    Args:
        parameters (str): Supplies parameters input to the operation.

    Returns:
        list[str]

    Raises:
        None documented; dependency failures may propagate.
    """
    if not parameters.strip() or parameters.strip() == "void":
        return []
    result: list[str] = []
    start = 0
    depth = 0
    for index, character in enumerate(parameters):
        if character in "<([{":
            depth += 1
        elif character in ">)]}":
            depth = max(0, depth - 1)
        elif character == "," and depth == 0:
            result.append(parameters[start:index].strip())
            start = index + 1
    result.append(parameters[start:].strip())
    return [item for item in result if item]


def cpp_param_name(parameter: str, index: int) -> str:
    """Summary:
        Performs the cpp param name operation for this subsystem.

    Args:
        parameter (str): Supplies parameter input to the operation.
        index (int): Supplies index input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    declaration = parameter.split("=", 1)[0].strip()
    function_pointer = re.search(r"\(\s*[*&]\s*([A-Za-z_]\w*)\s*\)", declaration)
    if function_pointer:
        return function_pointer.group(1)
    identifiers = re.findall(r"[A-Za-z_]\w*", declaration)
    ignored = {"const", "volatile", "unsigned", "signed", "long", "short",
               "typename", "class", "struct", "auto"}
    candidates = [item for item in identifiers if item not in ignored]
    if len(candidates) >= 2:
        return candidates[-1]
    return f"argument_{index + 1}"


def cpp_summary(name: str, kind: str = "function") -> str:
    """Summary:
        Performs the cpp summary operation for this subsystem.

    Args:
        name (str): Supplies name input to the operation.
        kind (str): Supplies kind input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    short = re.sub(r"^.*::", "", name).replace("~", "")
    words = humanize(short)
    if kind in {"class", "struct"}:
        return f"Encapsulates {words} state and behavior for this subsystem."
    if kind.startswith("enum"):
        return f"Enumerates the supported {words} values used by this subsystem."
    lowered = short.lower()
    if lowered.startswith("operator delete"):
        return "Releases dynamically allocated memory for this subsystem."
    if lowered == "deleteall":
        return "Clears all visualization markers for this subsystem."
    verbs = {
        "get": "Returns", "has": "Reports whether", "is": "Reports whether",
        "set": "Sets", "update": "Updates", "observe": "Processes",
        "validate": "Validates", "load": "Loads", "save": "Serializes",
        "read": "Reads", "write": "Writes", "parse": "Parses",
        "create": "Creates", "make": "Creates", "build": "Builds",
        "publish": "Publishes", "record": "Records", "clear": "Clears",
        "reset": "Resets", "plan": "Constructs", "decide": "Selects",
        "evaluate": "Evaluates", "apply": "Applies", "register": "Registers",
        "to": "Converts", "from": "Constructs",
    }
    for prefix, verb in verbs.items():
        if lowered.startswith(prefix):
            remainder = humanize(short[len(prefix):]) or words
            return f"{verb} {remainder} for this subsystem."
    return f"Performs the {words} operation for this subsystem."


def cpp_doc(name: str, params: str, return_type: str, indent: str,
            kind: str = "function") -> str:
    """Summary:
        Performs the cpp doc operation for this subsystem.

    Args:
        name (str): Supplies name input to the operation.
        params (str): Supplies params input to the operation.
        return_type (str): Supplies return type input to the operation.
        indent (str): Supplies indent input to the operation.
        kind (str): Supplies kind input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    lines = [f"{indent}/**"]

    def append_wrapped(content: str) -> None:
        """Summary:
            Performs the append wrapped operation for this subsystem.

        Args:
            content (str): Supplies content input to the operation.

        Returns:
            None; effects are applied to owned state or outputs.

        Raises:
            None documented; dependency failures may propagate.
        """
        wrapped = wrap_words(content, max(36, 76 - len(indent) - 3))
        lines.extend(f"{indent} * {line}" for line in wrapped)

    append_wrapped(f"@brief {cpp_summary(name, kind)}")
    lines.extend([f"{indent} *", f"{indent} * Arguments:"])
    arguments = split_cpp_params(params) if kind == "function" else []
    if arguments:
        for index, parameter in enumerate(arguments):
            param_name = cpp_param_name(parameter, index)
            append_wrapped(
                f"- @p {param_name}: Supplies {humanize(param_name)} input "
                "to the operation."
            )
    else:
        lines.append(f"{indent} * - None.")
    lines.extend([f"{indent} *", f"{indent} * Returns:"])
    normalized_return = re.sub(r"\s+", " ", return_type).strip()
    if kind != "function":
        lines.append(f"{indent} * - Not applicable to this declaration.")
    elif not normalized_return or normalized_return in {"void", "explicit"}:
        lines.append(f"{indent} * - No value; effects are applied to owned state or outputs.")
    else:
        append_wrapped(
            f"- `{normalized_return}` containing the operation result."
        )
    lines.extend([f"{indent} *", f"{indent} * Exceptions:"])
    append_wrapped(
        "- None documented; validation or dependency failures may propagate."
    )
    lines.append(f"{indent} */")
    return "\n".join(lines)


def cpp_entities(text: str) -> list[tuple[int, str]]:
    """Summary:
        Performs the cpp entities operation for this subsystem.

    Args:
        text (str): Supplies text input to the operation.

    Returns:
        list[tuple[int, str]]

    Raises:
        None documented; dependency failures may propagate.
    """
    clean = clean_cpp(text)
    lines = clean.splitlines(keepends=True)
    offsets: list[int] = []
    running = 0
    for line in lines:
        offsets.append(running)
        running += len(line)
    insertions: dict[int, str] = {}
    depth = 0
    function_depths: list[int] = []
    class_depths: list[tuple[int, str]] = []
    index = 0
    while index < len(lines):
        advance_to = index
        line = lines[index]
        stripped = line.strip()
        leading_closes = len(stripped) - len(stripped.lstrip("}"))
        effective_depth = max(0, depth - leading_closes)
        while function_depths and effective_depth < function_depths[-1]:
            function_depths.pop()
        while class_depths and effective_depth < class_depths[-1][0]:
            class_depths.pop()

        if not function_depths:
            type_match = CPP_TYPE.search(stripped)
            if type_match and not stripped.startswith(("//", "*")):
                start = index
                while start > 0 and lines[start - 1].strip().startswith(
                        ("template", "[[")):
                    start -= 1
                indent = re.match(r"\s*", lines[start]).group(0)
                insertions.setdefault(
                    offsets[start],
                    cpp_doc(type_match.group("name"), "", "", indent,
                            type_match.group("kind")),
                )
                if "{" in stripped:
                    class_depths.append((effective_depth + 1,
                                         type_match.group("name")))

            if "(" in stripped and not stripped.startswith("#"):
                end = index
                parsed = None
                candidate = ""
                while end < len(lines) and end < index + 200:
                    candidate = " ".join(
                        item.strip() for item in lines[index:end + 1]
                    )
                    parsed = parse_cpp_callable(candidate)
                    if parsed:
                        break
                    end += 1
                if end < len(lines):
                    if parsed:
                        name, parameters, raw_prefix, suffix = parsed
                        prefix = re.sub(r"\b(template\s*<.*?>|inline|static|virtual|"
                                        r"constexpr|consteval|explicit|friend)\b", " ",
                                        raw_prefix, flags=re.DOTALL).strip()
                        short = name.split("::")[-1].replace("~", "")
                        enclosing = class_depths[-1][1] if class_depths else ""
                        stripped_prefix = raw_prefix.strip()
                        invalid_prefix = (
                            stripped_prefix.startswith((":", "return ", "throw ")) or
                            any(token in raw_prefix for token in
                                ("=", ".", "->", "<<", ">>"))
                        )
                        valid_constructor = (
                            bool(enclosing and short == enclosing) or
                            (short[:1].isupper() and stripped_prefix in
                             {"", "explicit", "constexpr", "consteval"})
                        )
                        valid_qualified = "::" in name
                        valid_return = bool(prefix)
                        if (short not in CONTROL_NAMES and not short.isupper() and
                                not invalid_prefix and
                                (valid_constructor or valid_qualified or valid_return)):
                            start = index
                            while start > 0 and lines[start - 1].strip().startswith(
                                    ("template", "[[")):
                                start -= 1
                            indent = re.match(r"\s*", lines[start]).group(0)
                            insertions.setdefault(
                                offsets[start],
                                cpp_doc(name, parameters, prefix, indent),
                            )
                            advance_to = end
                            if "{" in suffix:
                                function_depths.append(effective_depth + 1)
                        elif short.startswith("TEST") and "{" in suffix:
                            advance_to = end
                            function_depths.append(effective_depth + 1)
        depth += sum(item.count("{") - item.count("}")
                     for item in lines[index:advance_to + 1])
        depth = max(0, depth)
        index = advance_to + 1
    return sorted(insertions.items())


def apply_cpp(path: Path, root: Path, text: str) -> str:
    """Summary:
        Applies cpp for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        root (Path): Supplies root input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    if "@file " in text[:1000]:
        return text
    insertions = cpp_entities(text)
    result = text
    for offset, comment in reversed(insertions):
        result = result[:offset] + comment + "\n" + result[offset:]
    return cpp_file_header(path, root, text) + result


def annotation_name(annotation: ast.expr | None) -> str:
    """Summary:
        Performs the annotation name operation for this subsystem.

    Args:
        annotation (ast.expr | None): Supplies annotation input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    if annotation is None:
        return "Any"
    return ast.unparse(annotation)


def python_raises(node: ast.AST) -> list[str]:
    """Summary:
        Performs the python raises operation for this subsystem.

    Args:
        node (ast.AST): Supplies node input to the operation.

    Returns:
        list[str]

    Raises:
        None documented; dependency failures may propagate.
    """
    result: list[str] = []
    for child in ast.walk(node):
        if child is node:
            continue
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        if isinstance(child, ast.Raise) and child.exc is not None:
            expression = child.exc.func if isinstance(child.exc, ast.Call) else child.exc
            try:
                result.append(ast.unparse(expression))
            except ValueError:
                result.append("Exception")
    return list(dict.fromkeys(result))


def python_summary(name: str, kind: str) -> str:
    """Summary:
        Performs the python summary operation for this subsystem.

    Args:
        name (str): Supplies name input to the operation.
        kind (str): Supplies kind input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    return cpp_summary(name, "class" if kind == "class" else "function")


def python_doc(node: ast.AST, indent: str, existing: str | None = None) -> str:
    """Summary:
        Performs the python doc operation for this subsystem.

    Args:
        node (ast.AST): Supplies node input to the operation.
        indent (str): Supplies indent input to the operation.
        existing (str | None): Supplies existing input to the operation.

    Returns:
        str

    Raises:
        TypeError: If required input or state is invalid.
    """
    if isinstance(node, ast.ClassDef):
        arguments: list[tuple[str, str]] = []
        returns = "Not applicable; classes construct instances."
        kind = "class"
    elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        all_arguments = [*node.args.posonlyargs, *node.args.args,
                         *node.args.kwonlyargs]
        arguments = [
            (argument.arg, annotation_name(argument.annotation))
            for argument in all_arguments if argument.arg not in {"self", "cls"}
        ]
        if node.args.vararg:
            arguments.append((f"*{node.args.vararg.arg}",
                              annotation_name(node.args.vararg.annotation)))
        if node.args.kwarg:
            arguments.append((f"**{node.args.kwarg.arg}",
                              annotation_name(node.args.kwarg.annotation)))
        returns = annotation_name(node.returns)
        if returns in {"None", "NoneType"}:
            returns = "None; effects are applied to owned state or outputs."
        kind = "function"
    else:
        raise TypeError("unsupported Python documentation node")
    summary = (existing or python_summary(node.name, kind)).strip()
    lines = [f'{indent}"""Summary:',
             *[f"{indent}    {line}" for line in summary.splitlines()], "",
             f"{indent}Args:"]
    if arguments:
        for name, annotation in arguments:
            lines.append(f"{indent}    {name} ({annotation}): Supplies "
                         f"{humanize(name)} input to the operation.")
    else:
        lines.append(f"{indent}    None.")
    lines.extend(["", f"{indent}Returns:", f"{indent}    {returns}", "",
                  f"{indent}Raises:"])
    raised = python_raises(node)
    if raised:
        for exception in raised:
            lines.append(f"{indent}    {exception}: If required input or state is invalid.")
    else:
        lines.append(f"{indent}    None documented; dependency failures may propagate.")
    lines.append(f'{indent}"""')
    return "\n".join(lines)


def apply_python(path: Path, root: Path, text: str) -> str:
    """Summary:
        Applies python for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        root (Path): Supplies root input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        str

    Raises:
        SyntaxError: If required input or state is invalid.
    """
    tree = ast.parse(text)
    lines = text.splitlines(keepends=True)
    insertions: list[tuple[int, str]] = []
    replacements: list[tuple[int, int, str]] = []
    module_doc = (
        '"""SemaFORR module overview.\n\n'
        f'Summary:\n    {overview(path, root, text)}\n\n'
        'Arguments:\n    Not applicable at module scope.\n\n'
        'Returns:\n    Not applicable at module scope.\n\n'
        'Raises:\n    Import-time dependency errors may propagate.\n"""\n\n'
    )
    existing_module = ast.get_docstring(tree, clean=False)
    if existing_module and tree.body and isinstance(tree.body[0], ast.Expr):
        expression = tree.body[0]
        start = expression.lineno - 1
        end = expression.end_lineno or expression.lineno
        replacements.append((start, end, module_doc))
    else:
        insertion_line = 0
        if lines and lines[0].startswith("#!"):
            insertion_line = 1
        if insertion_line < len(lines) and "coding" in lines[insertion_line][:40]:
            insertion_line += 1
        insertions.append((insertion_line, module_doc))

    entities = [node for node in ast.walk(tree)
                if isinstance(node, (ast.ClassDef, ast.FunctionDef,
                                     ast.AsyncFunctionDef))]
    for node in entities:
        if not node.body:
            continue
        body_indent = " " * node.body[0].col_offset
        existing = ast.get_docstring(node, clean=False)
        generated = python_doc(node, body_indent, existing) + "\n"
        if existing and isinstance(node.body[0], ast.Expr):
            expression = node.body[0]
            replacements.append((expression.lineno - 1,
                                 expression.end_lineno or expression.lineno,
                                 generated))
        else:
            insertion_line = node.body[0].lineno - 1
            if insertion_line == node.lineno - 1:
                raise SyntaxError(
                    f"one-line API declarations are unsupported: {path}:{node.lineno}"
                )
            insertions.append((insertion_line, generated))

    edits = [
        (start, end, replacement) for start, end, replacement in replacements
    ]
    edits.extend((line, line, insertion) for line, insertion in insertions)
    for start, end, replacement in sorted(edits, reverse=True):
        lines[start:end] = [replacement]
    return "".join(lines)


def apply_non_code(path: Path, root: Path, text: str) -> str:
    """Summary:
        Applies non code for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        root (Path): Supplies root input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        str

    Raises:
        None documented; dependency failures may propagate.
    """
    description = overview(path, root, text)
    suffix = path.suffix.lower()
    if suffix == ".json":
        data = json.loads(text)
        if isinstance(data, dict) and "$comment" not in data:
            data = {"$comment": description, **data}
        return json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    if suffix == ".md":
        if text.startswith("<!-- File overview:"):
            return text
        return f"<!-- File overview: {description} -->\n\n{text}"
    if suffix == ".xml" or path.name == "package.xml":
        if "File overview:" in text[:1000]:
            return text
        comment = f"<!-- File overview: {description} -->\n"
        if text.startswith("<?xml"):
            first_newline = text.find("\n") + 1
            return text[:first_newline] + comment + text[first_newline:]
        return comment + text
    if path.name == "CMakeLists.txt" or suffix in {
            ".yaml", ".yml", ".conf", ".txt", ".rviz"}:
        if text.startswith("# File overview:"):
            return text
        return f"# File overview: {description}\n{text}"
    if suffix == ".sh":
        if "# File overview:" in text[:500]:
            return text
        header = f"# File overview: {description}\n"
        if text.startswith("#!"):
            first_newline = text.find("\n") + 1
            return text[:first_newline] + header + text[first_newline:]
        return header + text
    return text


def package_files(root: Path) -> Iterable[Path]:
    """Summary:
        Performs the package files operation for this subsystem.

    Args:
        root (Path): Supplies root input to the operation.

    Returns:
        Iterable[Path]

    Raises:
        None documented; dependency failures may propagate.
    """
    for path in sorted(root.rglob("*")):
        if not path.is_file() or any(part.startswith(".") for part in
                                     path.relative_to(root).parts):
            continue
        if path.name == "package.xml" or path.name == "CMakeLists.txt" or \
                path.suffix.lower() in TEXT_SUFFIXES:
            yield path


def documented(path: Path, text: str) -> bool:
    """Summary:
        Performs the documented operation for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        bool

    Raises:
        None documented; dependency failures may propagate.
    """
    suffix = path.suffix.lower()
    if suffix in CPP_SUFFIXES:
        return "@file " in text[:1000] and "@details" in text[:1000]
    if suffix == ".py":
        module = ast.parse(text)
        doc = ast.get_docstring(module, clean=False) or ""
        return "Summary:" in doc and "Arguments:" in doc and "Returns:" in doc
    if suffix == ".json":
        data = json.loads(text)
        return isinstance(data, dict) and bool(data.get("$comment"))
    return "File overview:" in text[:1500]


def structured_api_documented(path: Path, text: str) -> bool:
    """Summary:
        Performs the structured api documented operation for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.
        text (str): Supplies text input to the operation.

    Returns:
        bool

    Raises:
        None documented; dependency failures may propagate.
    """
    suffix = path.suffix.lower()
    if suffix in CPP_SUFFIXES:
        for offset, _ in cpp_entities(text):
            prefix = text[:offset].rstrip()
            if not prefix.endswith("*/"):
                return False
            start = prefix.rfind("/**")
            if start < 0:
                return False
            block = prefix[start:]
            if "@brief" not in block or not all(
                    f"{section}:" in block
                    for section in ("Arguments", "Returns", "Exceptions")):
                return False
        return True
    if suffix == ".py":
        tree = ast.parse(text)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.ClassDef, ast.FunctionDef,
                                     ast.AsyncFunctionDef)):
                continue
            doc = ast.get_docstring(node, clean=False) or ""
            python_required = ("Summary:", "Args:", "Returns:", "Raises:")
            if not all(section in doc for section in python_required):
                return False
        return True
    return True


def clean_generated_code(root: Path) -> None:
    """Summary:
        Performs the clean generated code operation for this subsystem.

    Args:
        root (Path): Supplies root input to the operation.

    Returns:
        None; effects are applied to owned state or outputs.

    Raises:
        None documented; dependency failures may propagate.
    """
    generated_cpp = re.compile(
        r"^[ \t]*/\*\*\n(?:^[ \t]*\*.*\n)*?^[ \t]*\*/\n?",
        re.MULTILINE,
    )
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if path.suffix.lower() in CPP_SUFFIXES:
            text = path.read_text(encoding="utf-8")
            write_file(path, generated_cpp.sub("", text))
        elif path.suffix.lower() == ".py" and path.name != Path(__file__).name:
            text = path.read_text(encoding="utf-8")
            tree = ast.parse(text)
            relative = path.relative_to(root.parent.parent).as_posix()
            baseline = ""
            completed = subprocess.run(
                ["git", "show", f"HEAD:{relative}"], cwd=root.parent.parent,
                check=False, capture_output=True, text=True, encoding="utf-8",
            )
            if completed.returncode == 0:
                baseline = completed.stdout
            baseline_docs: dict[str, str] = {}
            if baseline:
                baseline_tree = ast.parse(baseline)
                module_doc = ast.get_docstring(baseline_tree, clean=False)
                if module_doc:
                    baseline_docs["<module>"] = module_doc

                def collect(nodes: list[ast.stmt], prefix: str = "") -> None:
                    """Summary:
                        Performs the collect operation for this subsystem.

                    Args:
                        nodes (list[ast.stmt]): Supplies nodes input to the operation.
                        prefix (str): Supplies prefix input to the operation.

                    Returns:
                        None; effects are applied to owned state or outputs.

                    Raises:
                        None documented; dependency failures may propagate.
                    """
                    for item in nodes:
                        if isinstance(item, (ast.ClassDef, ast.FunctionDef,
                                             ast.AsyncFunctionDef)):
                            qualified = f"{prefix}.{item.name}" if prefix else item.name
                            value = ast.get_docstring(item, clean=False)
                            if value:
                                baseline_docs[qualified] = value
                            collect(item.body, qualified)

                collect(baseline_tree.body)
            replacements: list[tuple[int, int, str]] = []

            def restore(nodes: list[ast.stmt], prefix: str = "") -> None:
                """Summary:
                    Performs the restore operation for this subsystem.

                Args:
                    nodes (list[ast.stmt]): Supplies nodes input to the operation.
                    prefix (str): Supplies prefix input to the operation.

                Returns:
                    None; effects are applied to owned state or outputs.

                Raises:
                    None documented; dependency failures may propagate.
                """
                for item in nodes:
                    if not isinstance(item, (ast.ClassDef, ast.FunctionDef,
                                             ast.AsyncFunctionDef)):
                        continue
                    qualified = f"{prefix}.{item.name}" if prefix else item.name
                    if item.body and ast.get_docstring(item, clean=False) is not None:
                        expression = item.body[0]
                        indent = " " * expression.col_offset
                        original = baseline_docs.get(qualified)
                        replacement = ""
                        if original is not None:
                            escaped = original.replace('"""', '\\"\\"\\"')
                            replacement = f'{indent}"""{escaped}"""\n'
                        replacements.append((expression.lineno - 1,
                                             expression.end_lineno or expression.lineno,
                                             replacement))
                    restore(item.body, qualified)

            restore(tree.body)
            if tree.body and ast.get_docstring(tree, clean=False) is not None:
                expression = tree.body[0]
                original = baseline_docs.get("<module>")
                replacement = ""
                if original is not None:
                    escaped = original.replace('"""', '\\"\\"\\"')
                    replacement = f'"""{escaped}"""\n'
                replacements.append((expression.lineno - 1,
                                     expression.end_lineno or expression.lineno,
                                     replacement))
            lines = text.splitlines(keepends=True)
            for start, end, replacement in sorted(replacements, reverse=True):
                lines[start:end] = [replacement] if replacement else []
            write_file(path, "".join(lines))


def process(root: Path, audit: bool) -> list[Path]:
    """Summary:
        Performs the process operation for this subsystem.

    Args:
        root (Path): Supplies root input to the operation.
        audit (bool): Supplies audit input to the operation.

    Returns:
        list[Path]

    Raises:
        None documented; dependency failures may propagate.
    """
    missing: list[Path] = []
    for path in package_files(root):
        text = path.read_text(encoding="utf-8")
        if not audit and not documented(path, text):
            if path.suffix.lower() in CPP_SUFFIXES:
                updated = apply_cpp(path, root, text)
            elif path.suffix.lower() == ".py":
                updated = apply_python(path, root, text)
            else:
                updated = apply_non_code(path, root, text)
            write_file(path, updated)
            text = updated
        if not documented(path, text) or not structured_api_documented(path, text):
            missing.append(path)
    return missing


def main(argv: Sequence[str] | None = None) -> int:
    """Summary:
        Performs the main operation for this subsystem.

    Args:
        argv (Sequence[str] | None): Supplies argv input to the operation.

    Returns:
        int

    Raises:
        None documented; dependency failures may propagate.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument("--audit", action="store_true")
    parser.add_argument("--clean-generated", action="store_true")
    arguments = parser.parse_args(argv)
    if arguments.clean_generated:
        clean_generated_code(arguments.root.resolve())
        return 0
    missing = process(arguments.root.resolve(), arguments.audit)
    for path in missing:
        print(f"missing structured documentation: {path}")
    print(f"documentation audit: {len(missing)} missing file(s)")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())

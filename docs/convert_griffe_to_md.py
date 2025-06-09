import json
import sys
import re
import html

def sanitize(text: str) -> str:
    return html.escape(text).replace("{", "&#123;").replace("}", "&#125;")

def get_param_summary(parameters: list) -> str:
    """
    Build a parameter summary string for the function/method title.
    It will list parameters (except 'self') with default values if provided.
    """
    summary_list = []
    for p in parameters:
        name = p["name"]
        if name == "self":
            continue
        default = p.get("default")
        if default is not None and str(default) != "None":
            summary_list.append(f"**{name}**=**{default}**")
        else:
            summary_list.append(f"**{name}**")
    return "(" + ", ".join(summary_list) + ")"

def format_docstring(docstring: str, param_defaults: dict) -> str:
    """
    Format a docstring by converting sections such as Args, Raises, and Returns
    into condensed bullet lists.
    
    For Args, each line will look like:
        - **param** (**datatype**, default=**value**) – description
    For Raises and Returns, similar condensed formatting is applied.
    """
    lines = docstring.splitlines()
    new_lines = []
    section = None

    for line in lines:
        header = re.match(r"^\s*(Args|Parameters|Raises|Returns?)\s*:\s*$", line, re.IGNORECASE)
        if header:
            section = header.group(1).lower()
            # Normalize sections for consistency
            if section == "return":
                section = "returns"
            elif section == "args":
                section = "parameters"
            new_lines.append(f"**{section.capitalize()}:**\n")
            continue

        if section in ["args", "parameters"]:
            # Match lines like: param_name (datatype): description
            m = re.match(r"^\s*(\w+)\s*\(([^)]+)\):\s*(.*)$", line)
            if m:
                name, datatype, desc = m.groups()
                bullet = f"- **{name}** (**{datatype}**"
                if name in param_defaults and param_defaults[name] is not None and str(param_defaults[name]) != "None":
                    bullet += f", default=**{param_defaults[name]}**"
                bullet += f") – {desc}"
                new_lines.append(bullet)
            else:
                # Handle lines that don't match the pattern but might still be parameter descriptions
                param_match = re.match(r"^\s*-\s*(\w+)\s*[:\(]", line)
                if param_match:
                    line_with_bold = re.sub(r"^(\s*-\s*)(\w+)", r"\1**\2**", line)
                    new_lines.append(line_with_bold)
                else:
                    new_lines.append("  " + line.strip() if line.strip() else "")
        elif section == "raises":
            # Match lines like: ExceptionType: description
            m = re.match(r"^\s*(\w+)\s*:\s*(.*)$", line)
            if m:
                ex_type, desc = m.groups()
                bullet = f"- **{ex_type}** – {desc}"
                new_lines.append(bullet)
            else:
                new_lines.append("  " + line.strip() if line.strip() else "")
        elif section == "returns":
            # Match lines like: datatype: description
            m = re.match(r"^\s*([^:]+):\s*(.*)$", line)
            if m:
                ret_type, desc = m.groups()
                bullet = f"- **{ret_type.strip()}** – {desc.strip()}"
                new_lines.append(bullet)
            else:
                new_lines.append("  " + line.strip() if line.strip() else "")
        else:
            new_lines.append(line)
    return "\n".join(new_lines)

def render_member_mdx(name: str, member: dict, level: int = 2) -> str:
    kind = member.get("kind", "").lower()

    if kind == "alias":
        return ""
    if name.startswith("__") and name.endswith("__") and kind != "class":
        return ""
    # Skip private functions/methods (starting with _)
    if kind in {"function", "method"} and name.startswith("_"):
        return ""
    if kind == "attribute" and not member.get("docstring", {}).get("value"):
        return ""

    raw_docstring = member.get("docstring", {}).get("value", "")
    docstring = sanitize(raw_docstring).strip()

    param_defaults = {}
    parameters = member.get("parameters", [])
    for p in parameters:
        param_defaults[p["name"]] = p.get("default")

    lines = []

    if kind == "class":
        lines.append(f"## **{name}**\n")
        if docstring:
            lines.append(docstring + "\n")

    # elif kind in {"function", "method"}:
    #     param_summary = get_param_summary(parameters)
    #     lines.append("<details>\n<summary><strong>")
    #     lines.append(f"{name}{param_summary}")
    #     lines.append("</strong></summary>\n")

    elif kind in {"function", "method"}:
        param_summary = get_param_summary(parameters)

        # Add real heading for sidebar navigation:
        lines.append(f"### {name}{param_summary}\n")

        # Add collapsible details (optional — nice UI):
        lines.append("<details>\n<summary>")
        lines.append("Info")
        lines.append("</summary>\n")
    
        if docstring:
            formatted_doc = format_docstring(docstring, param_defaults)
            lines.append(formatted_doc + "\n")
        lines.append("</details>\n")

    elif kind == "attribute":
        lines.append(f"<p><strong>{name}</strong></p>\n")
        if docstring:
            lines.append(f"<p>{docstring}</p>\n")

    if "members" in member:
        for sub_name, sub_member in member["members"].items():
            rendered = render_member_mdx(sub_name, sub_member, level + 1)
            if rendered.strip():
                lines.append(rendered)

    return "\n".join(lines).strip()

def generate_orcahand_mdx(json_path: str, output_mdx_path: str):
    with open(json_path) as f:
        data = json.load(f)

    with open(output_mdx_path, "w") as f:
        f.write("---\n")
        f.write("title: OrcaHand API\n")
        f.write("sidebar_position: 2\n")
        f.write("---\n\n")

        f.write("# OrcaHand Class Documentation\n\n")

        for module_name, module in data.items():
            if not module.get("members"):
                continue
            f.write(f"## Module: `{module_name}`\n\n")
            if "docstring" in module and module["docstring"]["value"].strip():
                f.write(sanitize(module["docstring"]["value"]) + "\n\n")
            for name, member in module["members"].items():
                if name != "core":
                    continue
                rendered = render_member_mdx(name, member, level=2)
                if rendered.strip():
                    f.write(rendered)
                    f.write("\n\n")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert_griffe_to_mdx.py <orca_core_api.json> <output.mdx>")
        sys.exit(1)
    generate_orcahand_mdx(sys.argv[1], sys.argv[2])

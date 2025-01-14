import os
import ast
import textwrap
import re
import os
from os.path import isdir
import shutil
import sys
import code2flow
import re

def parse_restructuredtext(docstring):
    """
    Парсит docstring в формате reStructuredText и возвращает структурированные данные.
    """
    if not docstring:
        return {}

    lines = docstring.strip().split("\n")

    description_lines = []
    rest_lines = []
    in_description = True

    for line in lines:
        if in_description and re.match(r":\w+\s+\w+:|\.\.\s+\w+::", line):
            in_description = False
        if in_description:
            description_lines.append(line)
        else:
            rest_lines.append(line)

    description = " ".join(description_lines).strip()
    rest = "\n".join(rest_lines)

    params = []
    attributes = []
    raises = []
    examples = None
    deprecated = None
    returns = None
    return_type = None
    notes = None
    warnings = None
    see_also = None

    # Регулярные выражения
    param_pattern = re.compile(r":param\s+(\w+):\s+(.*?)\n")
    type_pattern = re.compile(r":type\s+(\w+):\s+(.*?)\n")
    return_pattern = re.compile(r":return:\s+(.*?)\n")
    rtype_pattern = re.compile(r":rtype:\s+(.*?)\n")
    raises_pattern = re.compile(r":raises\s+(\w+):\s+(.*?)\n")
    attr_pattern = re.compile(r":ivar\s+(\w+):\s+(.*?)\n")
    attr_type_pattern = re.compile(r":vartype\s+(\w+):\s+(.*?)\n")
    deprecated_pattern = re.compile(r"\.\.\s+deprecated::\s+(.*?)\n(.*?)$", re.DOTALL)
    note_pattern = re.compile(r"\.\.\s+note::\s+(.*?)(?=\n\n|$)", re.DOTALL)
    warning_pattern = re.compile(r"\.\.\s+warning::\s+(.*?)(?=\n\n|$)", re.DOTALL)
    example_pattern = re.compile(r"\.\.\s+example::\s+(.*?)(?=\n\n|$)", re.DOTALL)
    see_also_pattern = re.compile(r"\.\.\s+seealso::\s+(.*?)(?=\n\n|$)", re.DOTALL)

    # Параметры
    for match in param_pattern.finditer(rest):
        param_name = match.group(1)
        param_desc = match.group(2)
        param_type = None
        type_match = type_pattern.search(rest)
        if type_match and type_match.group(1) == param_name:
            param_type = type_match.group(2)
        params.append({"name": param_name, "type": param_type, "description": param_desc})

    # Исключения
    for match in raises_pattern.finditer(rest):
        raises.append({"type": match.group(1), "description": match.group(2)})

    # Атрибуты
    for match in attr_pattern.finditer(rest):
        attr_name = match.group(1)
        attr_desc = match.group(2)
        attr_type = None
        type_match = attr_type_pattern.search(rest)
        if type_match and type_match.group(1) == attr_name:
            attr_type = type_match.group(2)
        attributes.append({"name": attr_name, "type": attr_type, "description": attr_desc})

    # Возвращаемое значение
    return_match = return_pattern.search(rest)
    if return_match:
        returns = return_match.group(1)

    # Тип возвращаемого значения
    rtype_match = rtype_pattern.search(rest)
    if rtype_match:
        return_type = rtype_match.group(1)

    # Другие директивы
    deprecated_match = deprecated_pattern.search(rest)
    if deprecated_match:
        deprecated = f"С версии {deprecated_match.group(1)}: {deprecated_match.group(2).strip()}"

    note_match = note_pattern.search(rest)
    if note_match:
        notes = note_match.group(1).strip()

    warning_match = warning_pattern.search(rest)
    if warning_match:
        warnings = warning_match.group(1).strip()

    example_match = example_pattern.search(rest)
    if example_match:
        examples = example_match.group(1).strip()

    see_also_match = see_also_pattern.search(rest)
    if see_also_match:
        see_also = see_also_match.group(1).strip()

    return {
        "description": description,
        "params": params,
        "returns": returns,
        "return_type": return_type,
        "raises": raises,
        "attributes": attributes,
        "deprecated": deprecated,
        "notes": notes,
        "warnings": warnings,
        "examples": examples,
        "see_also": see_also,
    }


def extract_docstrings(file_path):
    """
    Извлекает docstrings из Python файла.
    """
    with open(file_path, "r", encoding="utf-8") as file:
        tree = ast.parse(file.read(), filename=file_path)

    docstrings = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            class_doc = parse_restructuredtext(ast.get_docstring(node) or "")
            docstrings[f"Class: {node.name}"] = class_doc
            # Добавляем docstrings для методов класса
            for child in node.body:
                if isinstance(child, ast.FunctionDef):
                    func_doc = parse_restructuredtext(ast.get_docstring(child) or "")
                    docstrings[f"Method: {node.name}.{child.name}"] = func_doc
        elif isinstance(node, ast.FunctionDef):
            func_doc = parse_restructuredtext(ast.get_docstring(node) or "")
            docstrings[f"Function: {node.name}"] = func_doc
    return docstrings


def create_readme(docstrings, output_path, article):
    """
    Создает README файл на основе извлеченных docstrings.
    """
    content = [f"# {article.capitalize()}", "\n## Описание\n"]
    for key, doc in docstrings.items():
        content.append(f"### {key}\n")
        if "description" in doc and doc["description"]:
            content.append(f"**Описание:** {doc['description']}\n")
        if "params" in doc and doc["params"]:
            content.append("#### Параметры:\n")
            for param in doc["params"]:
                param_type = f" ({param['type']})" if param["type"] else ""
                content.append(f"- **{param['name']}**{param_type}: {param['description']}")
        if "returns" in doc and doc["returns"]:
            content.append(f"\n#### Возвращает:\n- {doc['returns']}")
        if "notes" in doc and doc["notes"]:
            content.append(f"\n#### Заметка:\n{textwrap.indent(doc['notes'], '    ')}")

    content.append("\n Диаграмма потока")
    content.append(f"\n ![Диаграмма потока](../img/graph_{article}.png)")
    readme_content = "\n".join(content)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as file:
        file.write(readme_content)

    print(f"README создан: {output_path}")


def main(source_file, output_readme="README.md", article="README"):
    """
    Главная функция для обработки файла и создания README.
    """
    if not os.path.exists(source_file):
        print(f"Файл {source_file} не найден.")
        return

    docstrings = extract_docstrings(source_file)
    create_readme(docstrings, output_readme, article)


if __name__ == "__main__":
    dirs = ["img", "description"]
    for di in dirs:
        if not (os.path.exists(di) and os.path.isdir(di)):
            os.mkdir(di)

    code2flow.code2flow(['./pion/spion.py',
                         './pion/pion.py',
                         './pion/pio.py',
                         './pion/apion.py',
                         './pion/functions.py',
                         './pion/simulator.py'],
                        './img/graph.png')

    code2flow.code2flow(['./pion/pion.py',
                         './pion/pio.py',
                         './pion/functions.py'],
                        './img/graph_pion.png')
    code2flow.code2flow(['./pion/pion.py',
                         './pion/pio.py',
                         './pion/functions.py',
                         './pion/apion.py'],
                        './img/graph_apion.png')

    code2flow.code2flow(['./pion/spion.py',
                         './pion/pio.py'],
                        './img/graph_spion.png')

    code2flow.code2flow(['./pion/simulator.py'],
                        './img/graph_simulator.png')

    code2flow.code2flow(['./pion/pio.py'],
                        './img/graph_pio.png')

    code2flow.code2flow(['./pion/functions.py'],
                        './img/graph_functions.png')
    path = './pion/'
    files = [file for file in os.listdir(path)
             if (file.endswith('.py') and
                 file not in ['__init__.py', 'annotation.py'])]
    for file in files:
        path_until_md = f'./description/{file[:-3]}.md'
        main(path + file, f'./description/{file[:-3]}.md', file[:-3])



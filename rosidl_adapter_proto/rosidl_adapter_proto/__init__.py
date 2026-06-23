# ================================= Apache 2.0 =================================
#
# Copyright (C) 2021 Continental
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ================================= Apache 2.0 =================================

import subprocess
import zlib

from rosidl_cmake import generate_files
import rosidl_parser.definition as rosidl

# A postfix for the protobuf package name / the c++ namespace
PROTO_PACKAGE_POSTFIX = 'pb'

# The rpc-function name for service calls. As ros services can only offer a
# single function, this function gets a static name in the protobuf service
PROTO_SERVICE_CALL_NAME = 'Call'

# The rpc function name for sending an action goal
PROTO_ACTION_SEND_GOAL_CALL_NAME = 'SendGoal'

# The rpc function name for retrieving the action result
PROTO_ACTION_GET_RESULT_CALL_NAME = 'GetResult'

# The rpc function name for canceling an action goal
PROTO_ACTION_CANCEL_CALL_NAME = 'CancelGoal'

# A Mapping from IDL -> Protobuf type
MSG_TYPE_TO_PROTO = {
    'boolean':     'bool',
    'octet':       'uint32',
    'char':        'uint32',
    'wchar':       'uint32',
    'float':       'float',
    'double':      'double',
    'long double': 'double',
    'uint8':       'uint32',
    'int8':        'int32',
    'uint16':      'uint32',
    'int16':       'int32',
    'uint32':      'fixed32',
    'int32':       'sfixed32',
    'uint64':      'fixed64',
    'int64':       'sfixed64',
    'string':      'string',
    'wstring':     'bytes',
}

field_val = 0


def compute_proto_field_number(variable_name):
    # Field number rules (https://developers.google.com/protocol-buffers/docs/
    # proto#assigning_field_numbers)
    #
    # Smallest: 1
    # Largest:  536870911 (= 2^29 - 1)
    #
    # Reserved Range: 19000 to 19999 (=> 1000 values)

    # Create a 32 bit hash from the variable name
    field_number = zlib.crc32(bytearray(variable_name, 'utf8'))
    # Reduce to the correct amount of values
    field_number = (field_number % (536870911 - 1000))
    # Account for the fact that we must not use 0
    field_number += 1
    # Account for the fact that we must not use 19000 to 19999
    if field_number >= 19000:
        field_number += 1000

    return field_number


def to_proto_import(namespaced_type):
    assert isinstance(namespaced_type, rosidl.NamespacedType)
    return '/'.join(namespaced_type.namespaces + [namespaced_type.name]) + '.proto'


def collect_proto_imports(rosidl_message):
    assert isinstance(rosidl_message, rosidl.Message)
    import_set = set()

    for member in rosidl_message.structure.members:
        if isinstance(member.type, rosidl.NamespacedType):
            namespaced_type = member.type
        elif isinstance(member.type, rosidl.AbstractNestedType) \
                and isinstance(member.type.value_type, rosidl.NamespacedType):
            namespaced_type = member.type.value_type
        else:
            continue

        import_set.add(to_proto_import(namespaced_type))

    return import_set



def _to_pascal_case(value):
    return ''.join(token.capitalize() for token in value.split('_') if token) or 'Enum'


def _make_unique_enum_name(candidate_name, message_name, used_enum_names):
    if candidate_name == message_name:
        candidate_name = f"{candidate_name}Value"

    base_name = candidate_name
    suffix = 1
    while candidate_name == message_name or candidate_name in used_enum_names:
        candidate_name = f"{base_name}{suffix}"
        suffix += 1
    return candidate_name


def _enum_constant_prefix(constant_name):
    return constant_name.split('_', 1)[0] if '_' in constant_name else constant_name


def _member_matches_enum_prefix(member_name, prefix):
    member_tokens = [token for token in member_name.lower().split('_') if token]
    prefix_tokens = [token for token in prefix.lower().split('_') if token]
    if not member_tokens or not prefix_tokens:
        return False
    if len(member_tokens) >= len(prefix_tokens) and member_tokens[-len(prefix_tokens):] == prefix_tokens:
        return True
    if len(member_tokens) >= len(prefix_tokens) and member_tokens[:len(prefix_tokens)] == prefix_tokens:
        return True
    return False


def _collect_constant_groups(message):
    enum_supported_types = {'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32'}
    enum_min_value = -(2 ** 31)
    enum_max_value = (2 ** 31) - 1

    constant_groups = {}
    for constant in message.constants:
        prefix = _enum_constant_prefix(constant.name)

        if isinstance(constant.type, rosidl.BasicType):
            if constant.type.typename not in enum_supported_types:
                continue
            if not isinstance(constant.value, int):
                continue
            if constant.value < enum_min_value or constant.value > enum_max_value:
                continue
            key = ('numeric', constant.type.typename, prefix)
            constant_groups.setdefault(key, []).append(constant)
            continue

        if isinstance(constant.type, rosidl.AbstractString) and isinstance(constant.value, str):
            key = ('string', 'string', prefix)
            constant_groups.setdefault(key, []).append(constant)

    return constant_groups


def _collect_members_by_kind(message):
    members_by_kind = {}

    for member in message.structure.members:
        if isinstance(member.type, rosidl.BasicType):
            members_by_kind.setdefault(('numeric', member.type.typename), []).append(member)
        elif isinstance(member.type, rosidl.AbstractString):
            members_by_kind.setdefault(('string', 'string'), []).append(member)

    return members_by_kind


def _build_numeric_entries(constants, enum_name):
    entries = [
        {
            'name': constant.name,
            'value': constant.value,
            'annotations': constant.annotations,
        }
        for constant in constants
    ]

    zero_entry_index = next(
        (
            index
            for index, enum_value in enumerate(entries)
            if enum_value['value'] == 0
        ),
        None,
    )

    if zero_entry_index is None:
        synthetic_name = f"{enum_name.upper()}_UNSPECIFIED"
        existing_names = {entry['name'] for entry in entries}
        synthetic_suffix = 1
        while synthetic_name in existing_names:
            synthetic_name = f"{enum_name.upper()}_UNSPECIFIED_{synthetic_suffix}"
            synthetic_suffix += 1

        entries.insert(
            0,
            {
                'name': synthetic_name,
                'value': 0,
                'annotations': [],
            },
        )
    elif zero_entry_index != 0:
        entries.insert(0, entries.pop(zero_entry_index))

    return entries


def _build_string_entries(constants):
    value_to_number = {}
    next_value = 0
    entries = []

    for constant in constants:
        if constant.value not in value_to_number:
            value_to_number[constant.value] = next_value
            next_value += 1

        entries.append(
            {
                'name': constant.name,
                'value': value_to_number[constant.value],
                'annotations': constant.annotations,
                'ros_value': constant.value,
            }
        )

    return entries


def collect_message_enum_info(message):
    assert isinstance(message, rosidl.Message)

    message_name = message.structure.namespaced_type.name
    constant_groups = _collect_constant_groups(message)
    members_by_kind = _collect_members_by_kind(message)

    enum_definitions = []
    member_enum_map = {}
    used_enum_names = set()

    for (enum_kind, type_name, prefix), constants in constant_groups.items():
        if not constants:
            continue

        candidate_members = [
            member for member in members_by_kind.get((enum_kind, type_name), [])
            if _member_matches_enum_prefix(member.name, prefix)
        ]

        mapped_member = candidate_members[0] if len(candidate_members) == 1 else None
        if mapped_member is not None:
            enum_name = _to_pascal_case(mapped_member.name) + 'Enum'
        else:
            enum_name = _to_pascal_case(prefix.lower()) + 'Enum'

        enum_name = _make_unique_enum_name(enum_name, message_name, used_enum_names)
        used_enum_names.add(enum_name)

        if mapped_member is not None and enum_kind == 'numeric':
            member_enum_map[mapped_member.name] = enum_name

        if enum_kind == 'numeric':
            entries = _build_numeric_entries(constants, enum_name)
        else:
            entries = _build_string_entries(constants)

        values = [entry['value'] for entry in entries]
        has_alias = len(values) != len(set(values))

        enum_definitions.append(
            {
                'name': enum_name,
                'entries': entries,
                'allow_alias': has_alias,
            }
        )

    return {
        'definitions': enum_definitions,
        'member_enum_map': member_enum_map,
    }

def generate_proto(generator_arguments_file):
    mapping = {
        'idl.proto.em': '%s.proto',
    }
    generate_files(generator_arguments_file, mapping, keep_case=True)
    return 0


def compile_proto(protoc_path, proto_path_list, cpp_out_dir, proto_files, package_name):
    protoc_cmd = [protoc_path]

    for path in proto_path_list:
        protoc_cmd.append('--proto_path=' + path)

    protoc_cmd.append(
        f'--cpp_out=dllexport_decl=ROSIDL_ADAPTER_PROTO_PUBLIC__{package_name}:{cpp_out_dir}')
    protoc_cmd = protoc_cmd + proto_files

    subprocess.check_call(protoc_cmd)

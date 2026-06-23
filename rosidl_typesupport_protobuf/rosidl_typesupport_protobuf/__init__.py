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

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
import rosidl_parser.definition as rosidl

# A postfix for the protobuf package name / the c++ namespace
PROTO_PACKAGE_POSTFIX = 'pb'

_TYPE_SUPPORT_NAME = ''
_NAMESPACE_DELIMETER = ''


def set_type_support_name(val):
    global _TYPE_SUPPORT_NAME
    _TYPE_SUPPORT_NAME = val


def set_namespace_delimeter(val):
    global _NAMESPACE_DELIMETER
    _NAMESPACE_DELIMETER = val


def typesupport_message_header(package_name, interface_path):
  include_parts = [package_name] + list(interface_path.parents[0].parts)
  include_parts += [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
  include_base = '/'.join(include_parts)

  return f"{include_base}__rosidl_typesupport_protobuf_cpp.hpp"

def ros_message_header(package_name, interface_path):
    include_parts = [package_name] + list(interface_path.parents[0].parts)
    include_parts += ['detail']
    include_parts += [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
    include_base = '/'.join(include_parts)

    return f'{include_base}__struct.hpp'


def ros_message_header_c(package_name, interface_path):
    include_parts = [package_name] + list(interface_path.parents[0].parts)
    include_parts += ['detail']
    include_parts += [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
    include_base = '/'.join(include_parts)

    return f'{include_base}__struct.h'


def ros_message_functions_header_c(package_name, interface_path):
    include_parts = [package_name] + list(interface_path.parents[0].parts)
    include_parts += ['detail']
    include_parts += [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
    include_base = '/'.join(include_parts)

    return f'{include_base}__functions.h'


def ros_message_functions_header_c_from_namespace(namespace, name):
    include_parts = list(namespace)
    include_parts += ['detail']
    include_parts += [convert_camel_case_to_lower_case_underscore(name)]
    include_base = '/'.join(include_parts)

    return f'{include_base}__functions.h'


def protobuf_message_header(package_name, interface_path):
    include_parts = [package_name] + list(interface_path.parents[0].parts)
    include_prefix = interface_path.stem

    return '/'.join(include_parts + [include_prefix + '.pb.h'])


def typesupport_header(package_name, interface_path):
    include_parts = [package_name] + list(interface_path.parents[0].parts) + \
        [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
    include_base = '/'.join(include_parts)

    return f'{include_base}__{_TYPE_SUPPORT_NAME}.hpp'


def visibility_control_header(package_name):
    return f'{package_name}/{_TYPE_SUPPORT_NAME}__visibility_control.h'


def adapter_visibility_control_header(package_name):
    return f'{package_name}/rosidl_adapter_proto__visibility_control.h'


def ros_type_namespace(package_name, interface_path):
    return _NAMESPACE_DELIMETER.join([package_name] + list(interface_path.parents[0].parts))


def ros_type_name(message):
    return message.structure.namespaced_type.name


def ros_type(package_name, interface_path, message):
    ros_type_ns = ros_type_namespace(package_name, interface_path)
    ros_type_nm = ros_type_name(message)
    return '::' + _NAMESPACE_DELIMETER.join([ros_type_ns, ros_type_nm])


def ros_type_from_namespaced_type(namespaced_type):
    return '::' + _NAMESPACE_DELIMETER.join(namespaced_type.namespaces + [namespaced_type.name])


def ros_type_from_namespaced_type_c(namespaced_type):
    return '::' + _NAMESPACE_DELIMETER.join(namespaced_type.namespaces + [namespaced_type.name])


def ros_service_namespace(package_name, interface_path):
    return _NAMESPACE_DELIMETER.join([package_name] + list(interface_path.parents[0].parts))


def ros_service_name(service):
    return service.namespaced_type.name


def ros_service_type(package_name, interface_path, service):
    ros_type_ns = ros_service_namespace(package_name, interface_path)
    ros_type_nm = ros_service_name(service)
    return '::' + _NAMESPACE_DELIMETER.join([ros_type_ns, ros_type_nm])




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


def message_enum_member_names(message):
    assert isinstance(message, rosidl.Message)

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
            constant_groups.setdefault(('numeric', constant.type.typename, prefix), []).append(constant)
            continue

        if isinstance(constant.type, rosidl.AbstractString) and isinstance(constant.value, str):
            constant_groups.setdefault(('string', 'string', prefix), []).append(constant)

    members_by_kind = {}
    for member in message.structure.members:
        if isinstance(member.type, rosidl.BasicType):
            members_by_kind.setdefault(('numeric', member.type.typename), []).append(member.name)
        elif isinstance(member.type, rosidl.AbstractString):
            members_by_kind.setdefault(('string', 'string'), []).append(member.name)

    enum_member_names = set()
    for (enum_kind, type_name, prefix), constants in constant_groups.items():
        if not constants:
            continue

        candidate_names = [
            member_name for member_name in members_by_kind.get((enum_kind, type_name), [])
            if _member_matches_enum_prefix(member_name, prefix)
        ]

        if len(candidate_names) != 1:
            continue

        if enum_kind == 'numeric':
            enum_member_names.add(candidate_names[0])

    return enum_member_names

def protobuf_type(package_name, interface_path, message):
    namespace = '::'.join([package_name] + list(interface_path.parents[0].parts))
    return '::' + '::'.join([namespace, PROTO_PACKAGE_POSTFIX, ros_type_name(message)])


def protobuf_type_from_namespaced_type(namespaced_type):
    return '::' + '::'.join(namespaced_type.namespaces +
                            [PROTO_PACKAGE_POSTFIX, namespaced_type.name])


def protobuf_type_from_namespaced_type_c(namespaced_type):
    return '::' + '::'.join(namespaced_type.namespaces +
                            [PROTO_PACKAGE_POSTFIX, namespaced_type.name])

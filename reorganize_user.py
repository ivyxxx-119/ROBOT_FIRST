from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "User"
OUTPUT = ROOT / "User_refactored"

if not SOURCE.is_dir():
    raise SystemExit("没有找到User目录，请在工程根目录运行脚本。")

if OUTPUT.exists():
    raise SystemExit(
        "User_refactored已存在，请先检查、移走或删除后再运行。"
    )


def read_source(relative_path):
    path = SOURCE / relative_path

    if not path.is_file():
        raise RuntimeError(f"缺少文件：{path}")

    return path.read_text(encoding="utf-8-sig")


robot_source = read_source("Application/robot.cpp")
read_source("Application/robot.hpp")

imu_source = read_source("Application/bmi_test.c")
imu_header = read_source("Application/bmi_test.h")


# ==============================================================
# 查找函数的结束位置。
# 跳过注释和字符串，避免注释中的括号影响结果。
# ==============================================================

def closing_brace(text, opening):
    depth = 0
    state = "code"
    quote = None
    i = opening

    while i < len(text):
        ch = text[i]
        pair = text[i:i + 2]

        if state == "line_comment":
            if ch == "\n":
                state = "code"

        elif state == "block_comment":
            if pair == "*/":
                state = "code"
                i += 1

        elif state == "string":
            if ch == "\\":
                i += 1
            elif ch == quote:
                state = "code"

        else:
            if pair == "//":
                state = "line_comment"
                i += 1

            elif pair == "/*":
                state = "block_comment"
                i += 1

            elif ch in ('"', "'"):
                state = "string"
                quote = ch

            elif ch == "{":
                depth += 1

            elif ch == "}":
                depth -= 1

                if depth == 0:
                    return i + 1

        i += 1

    raise RuntimeError("函数花括号不匹配。")


def extract_function(text, name, member=True):
    prefix = r"RobotController::" if member else ""

    pattern = (
        r'(?:extern\s+"C"\s+)?'
        r"(?:void|float|uint32_t)\s+"
        + prefix
        + re.escape(name)
        + r"\s*\([^)]*\)\s*\{"
    )

    matches = list(re.finditer(pattern, text))

    if len(matches) != 1:
        raise RuntimeError(
            f"{name}：应找到1个函数，实际找到{len(matches)}个。"
        )

    match = matches[0]
    opening = match.end() - 1
    end = closing_brace(text, opening)

    return text[match.start():end]


# ==============================================================
# 业务接口名称更新，只改接口名，不改业务数据变量名。
# ==============================================================

def update_imu_names(text):
    replacements = {
        "bmi_test_init": "Imu_Init",
        "bmi_test_update": "Imu_Update",
        "bmi_test_is_usable": "Imu_IsUsable",
    }

    for old, new in replacements.items():
        text = re.sub(r"\b" + old + r"\b", new, text)

    return text


groups = {
    "robot": [
        "init",
        "task",
        "onStateEnter",
        "updateStateMachine",
        "checkOnlineStatus",
        "faultStop",
        "clampPulse",
        "applyDeadband",
        "clampF",
    ],

    "chassis": [
        "resolveChassisCmd",
        "applyChassisMotors",
    ],

    "manipulator": [
        "resolveLeftStickAxis",
        "resolveJointCmd",
        "resolveYawCmd",
        "resolveLiftCmd",
        "resolveServoCmd",
        "applyJointMotors",
        "applyYawMotor",
        "applyLiftMotor",
        "applyServos",
        "setServoPulse",
    ],

    "communication": [
        "can1RxDispatch",
        "can2RxDispatch",
        "onUartRxEvent",
        "startCan",
    ],
}

expected_members = {
    name
    for names in groups.values()
    for name in names
}

actual_members = set(
    re.findall(r"RobotController::(\w+)\s*\(", robot_source)
)

if actual_members != expected_members:
    raise RuntimeError(
        "Robot成员函数与本次整理方案不一致。\n"
        f"未分配函数：{sorted(actual_members - expected_members)}\n"
        f"缺少函数：{sorted(expected_members - actual_members)}"
    )

functions = {}

for names in groups.values():
    for name in names:
        functions[name] = update_imu_names(
            extract_function(robot_source, name)
        )

# 确保非Stair模式不会残留速度前馈。
joint_function = functions["resolveJointCmd"]

if not re.search(
    r"jointCmd_\.velDegPerS\s*=\s*0\.0f",
    joint_function
):
    opening = joint_function.index("{") + 1

    joint_function = (
        joint_function[:opening]
        + "\n    jointCmd_.velDegPerS = 0.0f;\n"
        + joint_function[opening:]
    )

functions["resolveJointCmd"] = joint_function


# ==============================================================
# 调试变量提取
# ==============================================================

variable_pattern = re.compile(
    r"^volatile\s+"
    r"(?P<type>uint8_t|uint16_t|uint32_t|int16_t|float)\s+"
    r"(?P<name>\w+)\s*=\s*"
    r"(?P<value>[^;\n]+);",
    re.MULTILINE
)

robot_debug_variables = [
    match
    for match in variable_pattern.finditer(robot_source)
    if match.group("name").startswith("debug")
]

imu_debug_names = {
    "bmi_test_init_error",
    "bmi_test_init_attempts",
    "bmi_test_read_count",
}

imu_debug_variables = [
    match
    for match in variable_pattern.finditer(imu_source)
    if match.group("name") in imu_debug_names
]

if len(imu_debug_variables) != len(imu_debug_names):
    raise RuntimeError("BMI调试变量提取不完整。")

all_debug_variables = (
    robot_debug_variables + imu_debug_variables
)

debug_names = [
    match.group("name")
    for match in all_debug_variables
]

if len(debug_names) != len(set(debug_names)):
    raise RuntimeError("发现重复调试变量名。")

debug_declarations = "\n".join(
    f"extern volatile {match.group('type')} "
    f"{match.group('name')};"
    for match in all_debug_variables
)

debug_definitions = "\n".join(
    f"volatile {match.group('type')} "
    f"{match.group('name')} = "
    f"{match.group('value').strip()};"
    for match in all_debug_variables
)


# ==============================================================
# 提取底盘内部配置
# ==============================================================

assist_names = [
    "kBmiStraightAssistEnabled",
    "kBmiStraightKp",
    "kBmiStraightLimitRatio",
    "kBmiStraightCorrectionSign",
]

assist_definitions = []

for name in assist_names:
    pattern = (
        r"static\s+constexpr\s+"
        r"(?:bool|float)\s+"
        + re.escape(name)
        + r"\s*=\s*[^;]+;"
    )

    match = re.search(pattern, robot_source)

    if match is None:
        raise RuntimeError(f"缺少底盘配置：{name}")

    assist_definitions.append(match.group(0))


# ==============================================================
# 在内存中建立输出文件，完成检查后再写入。
# ==============================================================

files = {}

for path in SOURCE.rglob("*"):
    if not path.is_file():
        continue

    if path.suffix not in {".c", ".h", ".cpp", ".hpp"}:
        raise RuntimeError(
            f"User中发现非源码文件：{path}\n"
            "请先移到User外，或自行决定其存放位置。"
        )

    relative = path.relative_to(SOURCE)

    files[relative] = path.read_text(
        encoding="utf-8-sig"
    )


def put(relative_path, text):
    files[Path(relative_path)] = text.strip() + "\n"


def paired_header(name):
    guard = name.upper() + "_HPP"

    return f"""\
#ifndef {guard}
#define {guard}

#include "robot.hpp"

/*
 * 本模块实现RobotController的对应成员函数。
 * 类及成员函数声明统一保留在robot.hpp。
 */

#endif
"""


def member_source(name, preamble=""):
    body = "\n\n".join(
        functions[function_name]
        for function_name in groups[name]
    )

    return f"""\
#include "{name}.hpp"

{preamble}

namespace Robot
{{

{body}

}} // namespace Robot
"""


# --------------------------------------------------------------
# robot.cpp
# --------------------------------------------------------------

put(
    "Application/robot.cpp",
    member_source(
        "robot",
        """\
#include "main.h"
#include "imu_service.hpp"
#include "manipulator.hpp"
#include "debug.hpp"

extern "C"
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
    extern TIM_HandleTypeDef htim1;
    extern UART_HandleTypeDef huart3;
}"""
    )
)


# --------------------------------------------------------------
# chassis
# --------------------------------------------------------------

put(
    "Application/chassis.hpp",
    paired_header("chassis")
)

assist_block = "\n".join(assist_definitions)

put(
    "Application/chassis.cpp",
    member_source(
        "chassis",
        f"""\
#include "imu_service.hpp"
#include "debug.hpp"

#include <cmath>

namespace
{{

/* 保持原有参数，直行辅助默认关闭。 */
{assist_block}

}} // namespace"""
    )
)


# --------------------------------------------------------------
# manipulator
# --------------------------------------------------------------

put(
    "Application/manipulator.hpp",
    """\
#ifndef MANIPULATOR_HPP
#define MANIPULATOR_HPP

#include "robot.hpp"
#include "main.h"

namespace Robot
{

/*
 * 初始化和舵机输出共用的通道定义。
 * 配置直接归入机械臂模块，不建立robot_config。
 */
static constexpr uint32_t kServoFoldCh =
    TIM_CHANNEL_2;

static constexpr uint32_t kServoGripperCh =
    TIM_CHANNEL_3;

} // namespace Robot

#endif
"""
)

put(
    "Application/manipulator.cpp",
    member_source(
        "manipulator",
        """\
#include "debug.hpp"

#include <cmath>"""
    )
)


# --------------------------------------------------------------
# communication
# --------------------------------------------------------------

put(
    "Application/communication.hpp",
    paired_header("communication")
)

put(
    "Application/communication.cpp",
    member_source(
        "communication",
        """\
#include "main.h"
#include "debug.hpp"

extern "C"
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
}"""
    )
)


# --------------------------------------------------------------
# robot_api
# --------------------------------------------------------------

put(
    "Application/robot_api.hpp",
    """\
#ifndef ROBOT_API_HPP
#define ROBOT_API_HPP

/*
 * C/C++共享接口。
 * 虽然后缀是hpp，内容仍然兼容C。
 */
#ifdef __cplusplus
extern "C" {
#endif

void Robot_Init(void);
void Robot_Task(void);

#ifdef __cplusplus
}
#endif

#endif
"""
)

callback_names = [
    "HAL_CAN_RxFifo0MsgPendingCallback",
    "HAL_CAN_RxFifo1MsgPendingCallback",
    "HAL_UARTEx_RxEventCallback",
]

callbacks = "\n\n".join(
    extract_function(
        robot_source,
        callback_name,
        member=False
    )
    for callback_name in callback_names
)

put(
    "Application/robot_api.cpp",
    f"""\
#include "robot_api.hpp"

#include "robot.hpp"
#include "main.h"

extern "C"
{{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
}}

namespace
{{

/* 构造阶段只能设置初值，不能访问硬件。 */
Robot::RobotController g_robot;

}} // namespace

extern "C" void Robot_Init(void)
{{
    g_robot.init();
}}

extern "C" void Robot_Task(void)
{{
    g_robot.task();
}}

{callbacks}
"""
)


# --------------------------------------------------------------
# Debug
# --------------------------------------------------------------

put(
    "Debug/debug.hpp",
    f"""\
#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <stdint.h>

/*
 * 调试观测变量声明。
 * 所有定义集中在debug.cpp。
 */

{debug_declarations}

#endif
"""
)

put(
    "Debug/debug.cpp",
    f"""\
#include "debug.hpp"

{debug_definitions}
"""
)


# --------------------------------------------------------------
# IMU服务
# --------------------------------------------------------------

# 删除迁移到debug.cpp的变量定义。
imu_source = variable_pattern.sub(
    lambda match: (
        ""
        if match.group("name") in imu_debug_names
        else match.group(0)
    ),
    imu_source
)

imu_source = imu_source.replace(
    '#include "bmi_test.h"',
    '#include "imu_service.hpp"\n#include "debug.hpp"'
)

imu_source = imu_source.replace(
    "#include <math.h>",
    "#include <cmath>"
)

imu_source = re.sub(
    r"(?<![\w:])isfinite\s*\(",
    "std::isfinite(",
    imu_source
)

imu_source = update_imu_names(imu_source)

# 服务只由C++业务层调用，移除原来的C链接包装。
imu_header = imu_header.replace(
    '#ifdef __cplusplus\nextern "C" {\n#endif',
    ""
)

imu_header = imu_header.replace(
    "#ifdef __cplusplus\n}\n#endif",
    ""
)

imu_header = imu_header.replace(
    "BMI_TEST_H",
    "IMU_SERVICE_HPP"
)

for name in imu_debug_names:
    imu_header = re.sub(
        r"^extern\s+volatile\s+\w+\s+"
        + re.escape(name)
        + r"\s*;\s*$",
        "",
        imu_header,
        flags=re.MULTILINE
    )

imu_header = update_imu_names(imu_header)

put(
    "Device/imu_service.cpp",
    imu_source
)

put(
    "Device/imu_service.hpp",
    imu_header
)

# 移除旧文件。
files.pop(Path("Application/bmi_test.c"), None)
files.pop(Path("Application/bmi_test.h"), None)
files.pop(Path("Application/robot_api.h"), None)


# ==============================================================
# 统一User中的c/h后缀，并更新所有内部include。
# ==============================================================

header_renames = {}

for relative in files:
    if relative.suffix == ".h":
        old_name = relative.name
        new_name = relative.with_suffix(".hpp").name

        if (
            old_name in header_renames
            and header_renames[old_name] != new_name
        ):
            raise RuntimeError(
                f"头文件名称冲突：{old_name}"
            )

        header_renames[old_name] = new_name

# 即使旧文件已移除，也处理残留include。
header_renames["robot_api.h"] = "robot_api.hpp"
header_renames["bmi_test.h"] = "imu_service.hpp"


def rewrite_include(match):
    include_path = match.group(1)

    parts = include_path.rsplit("/", 1)
    old_name = parts[-1]

    if old_name not in header_renames:
        return match.group(0)

    new_name = header_renames[old_name]

    if len(parts) == 2:
        new_path = parts[0] + "/" + new_name
    else:
        new_path = new_name

    return f'#include "{new_path}"'


renamed_files = {}

for relative, content in files.items():
    if relative.suffix == ".c":
        new_relative = relative.with_suffix(".cpp")

    elif relative.suffix == ".h":
        new_relative = relative.with_suffix(".hpp")

    else:
        new_relative = relative

    content = re.sub(
        r'#include\s+"([^"]+)"',
        rewrite_include,
        content
    )

    if new_relative in renamed_files:
        raise RuntimeError(
            f"新旧文件重名：{new_relative}"
        )

    renamed_files[new_relative] = content


# 保证每个头文件都有对应实现文件。
# 纯类型/寄存器头的cpp只包含自己的头文件。
for relative in list(renamed_files):
    if relative.suffix != ".hpp":
        continue

    implementation = relative.with_suffix(".cpp")

    if implementation not in renamed_files:
        renamed_files[implementation] = (
            f'#include "{relative.name}"\n'
        )


# 每个实现文件必须有同名头文件。
for relative in renamed_files:
    if relative.suffix != ".cpp":
        continue

    header = relative.with_suffix(".hpp")

    if header not in renamed_files:
        raise RuntimeError(
            f"实现文件缺少同名头文件：{relative}"
        )


# ==============================================================
# 输出，不修改原User。
# ==============================================================

for relative, content in renamed_files.items():
    destination = OUTPUT / relative
    destination.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    destination.write_text(
        content.rstrip() + "\n",
        encoding="utf-8"
    )

print("整理完成：", OUTPUT)
print("原User目录未修改。")
print()
print("注意：还需要更新Core中的include和CMake源码路径。")
print("还需要检查bsp_delay.hpp等C接口的extern C保护。")
#!/usr/bin/env python3
"""Build offline C++ checks in a temporary directory and remove all outputs."""
import argparse
from pathlib import Path
import signal
import subprocess
import tempfile


CPP = Path(__file__).resolve().parents[1]
ROOT = CPP.parents[1]
TESTS = {
    'joint-monitor': ('g1_joint_monitor_test.cpp', ['unitree_sdk2', 'yaml-cpp', 'pthread'],
                      [ROOT / 'simulate/build/joint_monitor']),
    'suspension-sim': ('g1_passive_suspension_test.cpp', ['yaml-cpp', 'mujoco'], [ROOT]),
    'simulation': ('simulation_test.cpp', ['yaml-cpp', 'mujoco'], [ROOT]),
    'gamepad': ('g1_gamepad_test.cpp', ['yaml-cpp', 'mujoco'],
                [ROOT / 'simulate/gamepad.yaml']),
    'hardware': ('g1_hardware_test.cpp', ['unitree_sdk2', 'yaml-cpp'],
                 [ROOT / 'example/exported/g1_locomotion/g1_locomotion.yaml']),
    'locomotion': ('g1_locomotion_policy_test.cpp', ['yaml-cpp', 'onnxruntime'],
                   [ROOT / 'example/exported/g1_locomotion/policy.onnx',
                    ROOT / 'example/exported/g1_locomotion/g1_locomotion.yaml']),
    'parkour': ('g1_parkour_policy_test.cpp', ['yaml-cpp', 'onnxruntime'],
                [ROOT / 'example/exported/g1_parkour',
                 ROOT / 'example/exported/g1_parkour/g1_torsobase_cfg.json']),
    'ca-pe': ('g1_ca_pe_amp_vae_policy_test.cpp', ['yaml-cpp', 'onnxruntime'],
              [ROOT / 'example/exported/g1_CA_PE_AMP_VAE',
               ROOT / 'example/exported/g1_CA_PE_AMP_VAE/g1_ca_pe_amp_vae_cfg.json']),
    'ca-pe-observation': ('g1_ca_pe_observation_test.cpp', ['yaml-cpp', 'onnxruntime', 'mujoco'],
                          [ROOT, ROOT / 'example/exported/g1_CA_PE_AMP_VAE']),
    'ca-pe-sim': ('g1_parkour_cpp_mujoco_smoke.cpp', ['unitree_sdk2', 'yaml-cpp', 'mujoco', 'pthread'],
                  [ROOT, CPP / 'build/g1_ca_pe_amp_vae',
                   ROOT / 'example/exported/g1_CA_PE_AMP_VAE/g1_ca_pe_amp_vae_cfg.json']),
    'parkour-reference': ('g1_parkour_reference.cpp', ['yaml-cpp', 'onnxruntime', 'mujoco'], []),
    'parkour-sim': ('g1_parkour_cpp_mujoco_smoke.cpp', ['unitree_sdk2', 'yaml-cpp', 'mujoco', 'pthread'],
                    [ROOT, CPP / 'build/g1_parkour']),
}


def read_cache(directory):
    path = directory / 'CMakeCache.txt'
    values = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if line and not line.startswith(('#', '//')) and '=' in line:
                key, value = line.split('=', 1)
                values[key.split(':', 1)[0]] = value
    return values


def quote(value):
    # CMake bracket arguments preserve spaces, dollar signs and backslashes.
    value = str(value)
    delimiter = '='
    while ']' + delimiter + ']' in value:
        delimiter += '='
    return '[' + delimiter + '[' + value + ']' + delimiter + ']'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', choices=TESTS, action='append', help='Repeat to select checks; default: all offline policy checks')
    parser.add_argument('--cache-dir', type=Path, default=CPP / 'build', help='Read dependency paths only; never write here')
    parser.add_argument('--onnxruntime-root', type=Path)
    parser.add_argument('--sdk-prefix', type=Path)
    parser.add_argument('--reference-python', type=Path, help='Python with numpy, mujoco and onnxruntime for parkour-reference')
    parser.add_argument('--sim-mode', action='append', choices=['fixed','ground','stale','reset','band','terrain','ground-reset','jitter','walk'])
    parser.add_argument('--sim-scene', type=Path, help='Alternative MJCF scene for simulation diagnostics')
    parser.add_argument('--sim-config', type=Path, help='Alternative deployment JSON for simulation diagnostics')
    parser.add_argument('--task-dir', type=Path, help='Task model/config directory for ca-pe, ca-pe-sim and ca-pe-observation')
    parser.add_argument('--ctrlz-root', type=Path, help='Use actual CtrlZ SO3 headers in ca-pe-observation')
    parser.add_argument('--verbose', action='store_true', help='Show successful test telemetry')
    parser.add_argument('-j', '--jobs', type=int, default=4)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    selected = list(dict.fromkeys(args.test or [name for name in TESTS if name not in ('joint-monitor', 'suspension-sim', 'parkour-sim', 'ca-pe-sim', 'parkour-reference', 'ca-pe-observation')]))
    cache = read_cache(args.cache_dir)
    lines = ['cmake_minimum_required(VERSION 3.16)', 'project(g1_offline_checks LANGUAGES CXX)',
             'set(CMAKE_CXX_STANDARD 17)', 'set(CMAKE_BUILD_TYPE Release)',
             'find_package(yaml-cpp REQUIRED)', 'enable_testing()']
    if any(name in selected for name in ('hardware', 'parkour-sim', 'ca-pe-sim', 'joint-monitor')):
        prefixes = [str(args.sdk_prefix)] if args.sdk_prefix else [
            cache.get('CMAKE_PREFIX_PATH', ''), str(ROOT / '.local/unitree_robotics'), '/opt/unitree_robotics']
        lines += [f'list(APPEND CMAKE_PREFIX_PATH {quote(prefix)})' for prefix in prefixes if prefix]
        if not args.sdk_prefix and cache.get('unitree_sdk2_DIR'):
            lines.append(f'set(unitree_sdk2_DIR {quote(cache["unitree_sdk2_DIR"])})')
        lines.append('find_package(unitree_sdk2 REQUIRED)')
    if any(name in selected for name in ('locomotion', 'parkour', 'ca-pe', 'parkour-reference', 'ca-pe-observation')):
        ort_root = args.onnxruntime_root or cache.get('ONNXRUNTIME_ROOT', '')
        if args.onnxruntime_root:
            ort_include = args.onnxruntime_root / 'include'
            ort_library = ''
        else:
            ort_include = cache.get('ONNXRUNTIME_INCLUDE_DIR', '')
            ort_library = cache.get('ONNXRUNTIME_LIBRARY', '')
        lines += [f'find_path(ORT_INCLUDE onnxruntime_cxx_api.h HINTS {quote(ort_include)} {quote(str(ort_root)+"/include")})',
                  f'find_library(ORT_LIBRARY NAMES onnxruntime libonnxruntime.so.1 HINTS {quote(str(ort_root)+"/lib")})']
        if ort_library and Path(ort_library).is_file():
            lines.append(f'set(ORT_LIBRARY {quote(ort_library)})')
        lines += ['if(NOT ORT_INCLUDE OR NOT ORT_LIBRARY)',
                  'message(FATAL_ERROR "Set --onnxruntime-root to a matching ONNX Runtime C/C++ distribution")', 'endif()',
                  'add_library(onnxruntime SHARED IMPORTED)',
                  'set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${ORT_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${ORT_INCLUDE}")']
    if any(name in selected for name in ('suspension-sim', 'simulation', 'gamepad', 'parkour-sim', 'ca-pe-sim', 'parkour-reference', 'ca-pe-observation')):
        mujoco = ROOT / 'simulate/mujoco'
        lines += ['add_library(mujoco SHARED IMPORTED)',
                  f'set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION {quote(mujoco / "lib/libmujoco.so")} INTERFACE_INCLUDE_DIRECTORIES {quote(mujoco / "include")})']
    for name in selected:
        source, libraries, inputs = TESTS[name]
        if args.task_dir and name in ('ca-pe', 'ca-pe-sim'):
            task = args.task_dir.resolve()
            inputs = ([task, task / 'g1_ca_pe_amp_vae_cfg.json'] if name=='ca-pe'
                      else [*inputs[:2], task / 'g1_ca_pe_amp_vae_cfg.json'])
        if args.task_dir and name=='ca-pe-observation':
            inputs = [ROOT, args.task_dir.resolve()]
        target = 'g1_' + name + '_test'
        lines += [f'add_executable({target} {quote(CPP / "tests" / source)})',
                  f'target_include_directories({target} PRIVATE {quote(CPP)} {quote(ROOT / "include")} {quote(ROOT / "simulate/src")})',
                  f'target_link_libraries({target} PRIVATE {" ".join(libraries)})',
                  'if(UNIX AND NOT APPLE)',
                  f'target_link_options({target} PRIVATE "-Wl,--disable-new-dtags")', 'endif()']
        if name=='ca-pe-observation' and args.ctrlz_root:
            headers = args.ctrlz_root.resolve() / 'CtrlZ'
            if not (headers / 'Utils/so3.hpp').is_file():
                parser.error('--ctrlz-root must contain CtrlZ/Utils/so3.hpp')
            lines += [f'target_include_directories({target} PRIVATE {quote(headers)})',
                      f'target_compile_definitions({target} PRIVATE G1_HAVE_CTRLZ_SO3=1)']
        if name in ('parkour-sim','ca-pe-sim'):
            if args.sim_config or args.sim_scene:
                config = args.sim_config or (inputs[2] if len(inputs)>2 else ROOT / 'example/exported/g1_parkour/g1_torsobase_cfg.json')
                inputs = [*inputs[:2], config.resolve()]
            if args.sim_scene:
                inputs = [*inputs, args.sim_scene.resolve()]
            if args.task_dir and name=='ca-pe-sim':
                if not args.sim_scene:
                    inputs = [*inputs, '']
                inputs = [*inputs, args.task_dir.resolve()]
            lines.append(f'target_sources({target} PRIVATE {quote(ROOT / "simulate/src/joystick/joystick.cc")})')
            for mode in (args.sim_mode or (['fixed','ground','stale','reset','ground-reset','jitter','walk'] if name=='ca-pe-sim' else ['fixed', 'ground', 'stale', 'reset', 'band', 'terrain', 'ground-reset', 'jitter'])):
                lines.append(f'add_test(NAME {name}_{mode} COMMAND {target} {" ".join(quote(path) for path in inputs[:2])} {mode} {" ".join(quote(path) for path in inputs[2:])})')
        elif name=='parkour-reference':
            if not args.reference_python:
                parser.error('parkour-reference requires --reference-python')
            lines.append(f'add_test(NAME parkour_reference COMMAND {quote(args.reference_python)} {quote(CPP / "tests/check_parkour_reference.py")} $<TARGET_FILE:{target}> {quote(ROOT)})')
        else:
            lines.append(f'add_test(NAME {name} COMMAND {target} {" ".join(quote(path) for path in inputs)})')
    # The temporary CMakeLists, binaries, objects, cache and CTest logs share one lifetime.
    with tempfile.TemporaryDirectory(prefix='unitree-g1-tests-') as directory:
        temporary = Path(directory)
        print(f'Temporary test directory: {temporary}', flush=True)
        (temporary / 'CMakeLists.txt').write_text('\n'.join(lines) + '\n')
        try:
            subprocess.run(['cmake', '-S', str(temporary), '-B', str(temporary / 'build')], check=True)
            subprocess.run(['cmake', '--build', str(temporary / 'build'), '-j', str(args.jobs)], check=True)
            subprocess.run(['ctest', '--test-dir', str(temporary / 'build'), '--verbose' if args.verbose else '--output-on-failure'], check=True)
        finally:
            print('Removing temporary test project and all build outputs on exit.', flush=True)
    return 0


if __name__ == '__main__':
    def terminate(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, terminate)
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode)

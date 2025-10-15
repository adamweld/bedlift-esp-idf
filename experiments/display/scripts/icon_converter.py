#!/usr/bin/env python3
"""
Icon Converter for ESP32 Display
Converts PNG icons to monochrome bitmap C arrays for embedded use.

Usage:
    python icon_converter.py

Reads MODE_CONFIGS from ../main/config.hpp and converts corresponding
PNG files from ../icons/ directory to monochrome bitmaps.
"""

import os
import sys
from PIL import Image
import re

# Configuration
ICONS_DIR = "../icons"
CONFIG_FILE = "../main/config.hpp"
OUTPUT_FILE = "../main/assets/icons.hpp"
MODE_ICON_SIZE = 64   # 64x64 pixels for mode icons
BUTTON_ICON_SIZE = 48 # 48x48 pixels for button icons
MONITOR_ICON_SIZE = 24 # 24x24 pixels for monitor icons
THRESHOLD = 128       # Brightness threshold for monochrome conversion (0-255)

def parse_mode_configs(config_path):
    """Parse MODE_CONFIGS array from config.hpp"""
    with open(config_path, 'r') as f:
        content = f.read()

    # Find MODE_CONFIGS array
    match = re.search(r'static const ModeConfig MODE_CONFIGS\[\]\s*=\s*\{(.*?)\};',
                     content, re.DOTALL)
    if not match:
        print("ERROR: Could not find MODE_CONFIGS in config.hpp")
        sys.exit(1)

    configs_text = match.group(1)

    # Parse each config entry
    configs = []
    # Match each struct entry - now includes button files
    pattern = r'\{[^}]*?\.name\s*=\s*"([^"]*)"[^}]*?\.icon_file\s*=\s*"([^"]*)"[^}]*?\.rotation\s*=\s*(\d+)[^}]*?\.button_up_file\s*=\s*"([^"]*)"[^}]*?\.button_mode_file\s*=\s*"([^"]*)"[^}]*?\.button_down_file\s*=\s*"([^"]*)"[^}]*?\}'

    for match in re.finditer(pattern, configs_text):
        name = match.group(1)
        icon_file = match.group(2)
        rotation = int(match.group(3))
        button_up_file = match.group(4)
        button_mode_file = match.group(5)
        button_down_file = match.group(6)
        configs.append({
            'name': name,
            'icon_file': icon_file,
            'rotation': rotation,
            'button_up_file': button_up_file,
            'button_mode_file': button_mode_file,
            'button_down_file': button_down_file
        })

    return configs

def parse_monitor_configs(config_path):
    """Parse MONITOR_CONFIGS array from config.hpp"""
    with open(config_path, 'r') as f:
        content = f.read()

    # Find MONITOR_CONFIGS array
    match = re.search(r'static const MonitorConfig MONITOR_CONFIGS\[\]\s*=\s*\{(.*?)\};',
                     content, re.DOTALL)
    if not match:
        print("WARNING: Could not find MONITOR_CONFIGS in config.hpp")
        return []

    configs_text = match.group(1)

    # Parse each monitor config entry
    configs = []
    pattern = r'\{[^}]*?\.name\s*=\s*"([^"]*)"[^}]*?\.icon_true_file\s*=\s*(?:"([^"]*)"|nullptr)[^}]*?\.icon_false_file\s*=\s*(?:"([^"]*)"|nullptr)[^}]*?\}'

    for match in re.finditer(pattern, configs_text):
        name = match.group(1)
        icon_true_file = match.group(2) if match.group(2) else None
        icon_false_file = match.group(3) if match.group(3) else None
        configs.append({
            'name': name,
            'icon_true_file': icon_true_file,
            'icon_false_file': icon_false_file
        })

    return configs

def convert_to_monochrome_bitmap(image_path, size, rotation, threshold=THRESHOLD):
    """
    Convert PNG to monochrome bitmap array.

    Args:
        image_path: Path to PNG file
        size: Target size (width and height)
        rotation: Rotation in steps (0=0°, 1=90°, 2=180°, 3=270°)
        threshold: Brightness threshold for black/white conversion

    Returns:
        List of bytes representing the monochrome bitmap
    """
    try:
        img = Image.open(image_path)
    except FileNotFoundError:
        print(f"WARNING: Icon file not found: {image_path}")
        return None

    # Handle transparency: use alpha channel as the grayscale value
    # (assumes dark icon on transparent background)
    if img.mode == 'RGBA':
        # Extract alpha channel - opaque pixels = 255, transparent = 0
        alpha = img.split()[3]
        img = alpha
    elif img.mode == 'LA':
        # Extract alpha from grayscale+alpha
        img = img.split()[1]
    else:
        # Convert to grayscale normally
        img = img.convert('L')

    # Resize to target size
    img = img.resize((size, size), Image.Resampling.LANCZOS)

    # Apply rotation (PIL uses degrees, we use steps)
    rotation_degrees = rotation * 90
    if rotation_degrees != 0:
        img = img.rotate(rotation_degrees, expand=False)

    # Convert to monochrome bitmap
    # Each byte contains 8 pixels (MSB first)
    bitmap = []
    pixels = list(img.getdata())

    for y in range(size):
        for x in range(0, size, 8):
            byte = 0
            for bit in range(8):
                if x + bit < size:
                    pixel_idx = y * size + x + bit
                    # If pixel is brighter than threshold, set bit to 1 (white)
                    if pixels[pixel_idx] > threshold:
                        byte |= (1 << (7 - bit))
            bitmap.append(byte)

    return bitmap

def format_bitmap_array(bitmap_data, bytes_per_line=12):
    """Format bitmap data as C array with proper indentation"""
    lines = []
    for i in range(0, len(bitmap_data), bytes_per_line):
        chunk = bitmap_data[i:i+bytes_per_line]
        hex_values = ', '.join(f'0x{b:02X}' for b in chunk)
        lines.append(f"    {hex_values}")

    return ',\n'.join(lines)

def generate_variable_name(filename, rotation):
    """Generate C variable name from filename with rotation"""
    # Remove extension and convert to snake_case
    name = os.path.splitext(filename)[0]
    name = name.replace('-', '_').replace(' ', '_').lower()

    # Add rotation suffix to make each variation unique
    rotation_suffix = f"_r{rotation * 90}" if rotation != 0 else ""

    return f"icon_mode_{name}{rotation_suffix}"

def main():
    print("Icon Converter for ESP32 Display")
    print("=" * 60)

    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Parse config file
    print(f"\nParsing {CONFIG_FILE}...")
    configs = parse_mode_configs(CONFIG_FILE)
    print(f"Found {len(configs)} mode configurations")

    monitor_configs = parse_monitor_configs(CONFIG_FILE)
    print(f"Found {len(monitor_configs)} monitor configurations")

    # Generate header file
    header_lines = [
        "#pragma once",
        "",
        "#include <cstdint>",
        "",
        "// ============================================================================",
        "// Mode Icon Bitmap Data (64x64 monochrome)",
        "// ============================================================================",
        "// Auto-generated by scripts/icon_converter.py",
        "// Mode icons: 64x64 pixels, 1 bit per pixel = 512 bytes",
        "// Button icons: 48x48 pixels, 1 bit per pixel = 288 bytes",
        "// Monitor icons: 24x24 pixels, 1 bit per pixel = 72 bytes",
        "// Format: MSB first, row by row",
        "//",
        "// To draw: gfx->drawBitmap(x, y, icon_data, width, height, foreground_color);",
        "// ============================================================================",
        ""
    ]

    mode_icon_names = []
    button_up_icons = {}   # Track unique button up icons
    button_mode_icons = {} # Track unique button mode icons
    button_down_icons = {} # Track unique button down icons

    # Process each mode icon
    for idx, config in enumerate(configs):
        icon_file = config['icon_file']
        icon_path = os.path.join(ICONS_DIR, icon_file)
        rotation = config['rotation']
        var_name = generate_variable_name(icon_file, rotation)

        print(f"\n[{idx}] Processing: {config['name']}")
        print(f"    Mode icon: {icon_file}")
        print(f"    Rotation: {config['rotation'] * 90}°")

        # Convert mode icon to bitmap
        bitmap = convert_to_monochrome_bitmap(icon_path, MODE_ICON_SIZE, rotation)

        if bitmap is None:
            print(f"    Mode icon SKIPPED (file not found)")
            # Generate placeholder
            bitmap = [0x00] * ((MODE_ICON_SIZE * MODE_ICON_SIZE) // 8)
            header_lines.append(f"// Mode {idx}: {config['name']} (PLACEHOLDER - file not found)")
        else:
            print(f"    Mode icon: {len(bitmap)} bytes")
            header_lines.append(f"// Mode {idx}: {config['name']}")

        header_lines.append(f"constexpr uint8_t {var_name}[] = {{")
        header_lines.append(format_bitmap_array(bitmap))
        header_lines.append("};")
        header_lines.append("")

        mode_icon_names.append(var_name)

        # Process button icons (unique ones only)
        button_up_file = config['button_up_file']
        button_mode_file = config['button_mode_file']
        button_down_file = config['button_down_file']

        # Button up icon
        if button_up_file not in button_up_icons:
            button_up_path = os.path.join(ICONS_DIR, button_up_file)
            button_up_var = generate_variable_name(button_up_file, 0).replace("icon_mode_", "icon_button_up_")
            button_bitmap = convert_to_monochrome_bitmap(button_up_path, BUTTON_ICON_SIZE, 0)

            if button_bitmap is None:
                print(f"    Button up SKIPPED (file not found)")
                button_bitmap = [0x00] * ((BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8)
            else:
                print(f"    Button up: {len(button_bitmap)} bytes")

            button_up_icons[button_up_file] = button_up_var

        # Button mode icon
        if button_mode_file not in button_mode_icons:
            button_mode_path = os.path.join(ICONS_DIR, button_mode_file)
            button_mode_var = generate_variable_name(button_mode_file, 0).replace("icon_mode_", "icon_button_mode_")
            button_bitmap = convert_to_monochrome_bitmap(button_mode_path, BUTTON_ICON_SIZE, 0)

            if button_bitmap is None:
                print(f"    Button mode SKIPPED (file not found)")
                button_bitmap = [0x00] * ((BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8)
            else:
                print(f"    Button mode: {len(button_bitmap)} bytes")

            button_mode_icons[button_mode_file] = button_mode_var

        # Button down icon
        if button_down_file not in button_down_icons:
            button_down_path = os.path.join(ICONS_DIR, button_down_file)
            button_down_var = generate_variable_name(button_down_file, 0).replace("icon_mode_", "icon_button_down_")
            button_bitmap = convert_to_monochrome_bitmap(button_down_path, BUTTON_ICON_SIZE, 0)

            if button_bitmap is None:
                print(f"    Button down SKIPPED (file not found)")
                button_bitmap = [0x00] * ((BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8)
            else:
                print(f"    Button down: {len(button_bitmap)} bytes")

            button_down_icons[button_down_file] = button_down_var

    # Generate button icon arrays
    header_lines.append("// ============================================================================")
    header_lines.append("// Button Icon Bitmap Data (48x48 monochrome)")
    header_lines.append("// ============================================================================")
    header_lines.append("")

    for button_file, var_name in button_up_icons.items():
        button_path = os.path.join(ICONS_DIR, button_file)
        bitmap = convert_to_monochrome_bitmap(button_path, BUTTON_ICON_SIZE, 0)
        if bitmap is None:
            bitmap = [0x00] * ((BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8)

        header_lines.append(f"// Button up: {button_file}")
        header_lines.append(f"constexpr uint8_t {var_name}[] = {{")
        header_lines.append(format_bitmap_array(bitmap))
        header_lines.append("};")
        header_lines.append("")

    for button_file, var_name in button_mode_icons.items():
        button_path = os.path.join(ICONS_DIR, button_file)
        bitmap = convert_to_monochrome_bitmap(button_path, BUTTON_ICON_SIZE, 0)
        if bitmap is None:
            bitmap = [0x00] * ((BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8)

        header_lines.append(f"// Button mode: {button_file}")
        header_lines.append(f"constexpr uint8_t {var_name}[] = {{")
        header_lines.append(format_bitmap_array(bitmap))
        header_lines.append("};")
        header_lines.append("")

    for button_file, var_name in button_down_icons.items():
        button_path = os.path.join(ICONS_DIR, button_file)
        bitmap = convert_to_monochrome_bitmap(button_path, BUTTON_ICON_SIZE, 0)
        if bitmap is None:
            bitmap = [0x00] * ((BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8)

        header_lines.append(f"// Button down: {button_file}")
        header_lines.append(f"constexpr uint8_t {var_name}[] = {{")
        header_lines.append(format_bitmap_array(bitmap))
        header_lines.append("};")
        header_lines.append("")

    # Generate monitor icon arrays
    header_lines.append("// ============================================================================")
    header_lines.append("// Monitor Icon Bitmap Data (24x24 monochrome)")
    header_lines.append("// ============================================================================")
    header_lines.append("")

    monitor_icons = {}  # Track unique monitor icons
    print(f"\nProcessing monitor icons...")

    for idx, config in enumerate(monitor_configs):
        print(f"\n[Monitor {idx}] {config['name']}")

        # Process true icon
        if config['icon_true_file']:
            icon_file = config['icon_true_file']
            if icon_file not in monitor_icons:
                icon_path = os.path.join(ICONS_DIR, icon_file)
                var_name = f"icon_monitor_{os.path.splitext(icon_file)[0].replace('-', '_').replace(' ', '_').lower()}"
                bitmap = convert_to_monochrome_bitmap(icon_path, MONITOR_ICON_SIZE, 0)

                if bitmap is None:
                    print(f"    True icon SKIPPED (file not found): {icon_file}")
                    bitmap = [0x00] * ((MONITOR_ICON_SIZE * MONITOR_ICON_SIZE) // 8)
                else:
                    print(f"    True icon: {icon_file} ({len(bitmap)} bytes)")

                monitor_icons[icon_file] = var_name
                header_lines.append(f"// Monitor: {icon_file}")
                header_lines.append(f"constexpr uint8_t {var_name}[] = {{")
                header_lines.append(format_bitmap_array(bitmap))
                header_lines.append("};")
                header_lines.append("")

        # Process false icon
        if config['icon_false_file']:
            icon_file = config['icon_false_file']
            if icon_file not in monitor_icons:
                icon_path = os.path.join(ICONS_DIR, icon_file)
                var_name = f"icon_monitor_{os.path.splitext(icon_file)[0].replace('-', '_').replace(' ', '_').lower()}"
                bitmap = convert_to_monochrome_bitmap(icon_path, MONITOR_ICON_SIZE, 0)

                if bitmap is None:
                    print(f"    False icon SKIPPED (file not found): {icon_file}")
                    bitmap = [0x00] * ((MONITOR_ICON_SIZE * MONITOR_ICON_SIZE) // 8)
                else:
                    print(f"    False icon: {icon_file} ({len(bitmap)} bytes)")

                monitor_icons[icon_file] = var_name
                header_lines.append(f"// Monitor: {icon_file}")
                header_lines.append(f"constexpr uint8_t {var_name}[] = {{")
                header_lines.append(format_bitmap_array(bitmap))
                header_lines.append("};")
                header_lines.append("")

    # Generate lookup functions
    header_lines.extend([
        "// ============================================================================",
        "// Icon Lookup Functions",
        "// ============================================================================",
        "",
        "// Get mode icon data for a specific mode index",
        "inline const uint8_t* getModeIcon(int mode_index) {",
        "    switch (mode_index) {"
    ])

    for idx, var_name in enumerate(mode_icon_names):
        header_lines.append(f"        case {idx}: return {var_name};")

    header_lines.extend([
        "        default: return nullptr;",
        "    }",
        "}",
        ""
    ])

    # Button up lookup
    header_lines.extend([
        "// Get button up icon for a specific mode index",
        "inline const uint8_t* getButtonUpIcon(int mode_index) {",
        "    if (mode_index < 0 || mode_index >= " + str(len(configs)) + ") return nullptr;",
        "    const char* button_files[] = {"
    ])

    for config in configs:
        header_lines.append(f'        "{config["button_up_file"]}",')

    header_lines.extend([
        "    };",
        "    const char* file = button_files[mode_index];",
        ""
    ])

    for button_file, var_name in button_up_icons.items():
        header_lines.append(f'    if (strcmp(file, "{button_file}") == 0) return {var_name};')

    header_lines.extend([
        "    return nullptr;",
        "}",
        ""
    ])

    # Button mode lookup
    header_lines.extend([
        "// Get button mode icon for a specific mode index",
        "inline const uint8_t* getButtonModeIcon(int mode_index) {",
        "    if (mode_index < 0 || mode_index >= " + str(len(configs)) + ") return nullptr;",
        "    const char* button_files[] = {"
    ])

    for config in configs:
        header_lines.append(f'        "{config["button_mode_file"]}",')

    header_lines.extend([
        "    };",
        "    const char* file = button_files[mode_index];",
        ""
    ])

    for button_file, var_name in button_mode_icons.items():
        header_lines.append(f'    if (strcmp(file, "{button_file}") == 0) return {var_name};')

    header_lines.extend([
        "    return nullptr;",
        "}",
        ""
    ])

    # Button down lookup
    header_lines.extend([
        "// Get button down icon for a specific mode index",
        "inline const uint8_t* getButtonDownIcon(int mode_index) {",
        "    if (mode_index < 0 || mode_index >= " + str(len(configs)) + ") return nullptr;",
        "    const char* button_files[] = {"
    ])

    for config in configs:
        header_lines.append(f'        "{config["button_down_file"]}",')

    header_lines.extend([
        "    };",
        "    const char* file = button_files[mode_index];",
        ""
    ])

    for button_file, var_name in button_down_icons.items():
        header_lines.append(f'    if (strcmp(file, "{button_file}") == 0) return {var_name};')

    header_lines.extend([
        "    return nullptr;",
        "}",
        ""
    ])

    # Monitor icon lookups
    header_lines.extend([
        "// Get monitor icon for true state",
        "inline const uint8_t* getMonitorIconTrue(int monitor_index) {",
        "    if (monitor_index < 0 || monitor_index >= " + str(len(monitor_configs)) + ") return nullptr;",
        "    const char* icon_files[] = {"
    ])

    for config in monitor_configs:
        if config['icon_true_file']:
            header_lines.append(f'        "{config["icon_true_file"]}",')
        else:
            header_lines.append('        nullptr,')

    header_lines.extend([
        "    };",
        "    const char* file = icon_files[monitor_index];",
        "    if (file == nullptr) return nullptr;",
        ""
    ])

    for icon_file, var_name in monitor_icons.items():
        header_lines.append(f'    if (strcmp(file, "{icon_file}") == 0) return {var_name};')

    header_lines.extend([
        "    return nullptr;",
        "}",
        ""
    ])

    header_lines.extend([
        "// Get monitor icon for false state",
        "inline const uint8_t* getMonitorIconFalse(int monitor_index) {",
        "    if (monitor_index < 0 || monitor_index >= " + str(len(monitor_configs)) + ") return nullptr;",
        "    const char* icon_files[] = {"
    ])

    for config in monitor_configs:
        if config['icon_false_file']:
            header_lines.append(f'        "{config["icon_false_file"]}",')
        else:
            header_lines.append('        nullptr,')

    header_lines.extend([
        "    };",
        "    const char* file = icon_files[monitor_index];",
        "    if (file == nullptr) return nullptr;",
        ""
    ])

    for icon_file, var_name in monitor_icons.items():
        header_lines.append(f'    if (strcmp(file, "{icon_file}") == 0) return {var_name};')

    header_lines.extend([
        "    return nullptr;",
        "}",
        "",
        "// Icon dimensions",
        f"constexpr int MODE_ICON_SIZE = {MODE_ICON_SIZE};",
        f"constexpr int BUTTON_ICON_SIZE = {BUTTON_ICON_SIZE};",
        f"constexpr int MONITOR_ICON_SIZE = {MONITOR_ICON_SIZE};",
        ""
    ])

    # Write output file
    output_path = OUTPUT_FILE
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, 'w') as f:
        f.write('\n'.join(header_lines))

    print("\n" + "=" * 60)
    print(f"SUCCESS: Generated {output_path}")
    print(f"Mode icons: {len(mode_icon_names)} @ {MODE_ICON_SIZE}x{MODE_ICON_SIZE} = {len(mode_icon_names) * (MODE_ICON_SIZE * MODE_ICON_SIZE) // 8} bytes")
    print(f"Button up icons: {len(button_up_icons)} @ {BUTTON_ICON_SIZE}x{BUTTON_ICON_SIZE} = {len(button_up_icons) * (BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8} bytes")
    print(f"Button mode icons: {len(button_mode_icons)} @ {BUTTON_ICON_SIZE}x{BUTTON_ICON_SIZE} = {len(button_mode_icons) * (BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8} bytes")
    print(f"Button down icons: {len(button_down_icons)} @ {BUTTON_ICON_SIZE}x{BUTTON_ICON_SIZE} = {len(button_down_icons) * (BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8} bytes")
    print(f"Monitor icons: {len(monitor_icons)} @ {MONITOR_ICON_SIZE}x{MONITOR_ICON_SIZE} = {len(monitor_icons) * (MONITOR_ICON_SIZE * MONITOR_ICON_SIZE) // 8} bytes")
    total_bytes = (len(mode_icon_names) * (MODE_ICON_SIZE * MODE_ICON_SIZE) // 8) + \
                  (len(button_up_icons) * (BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8) + \
                  (len(button_mode_icons) * (BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8) + \
                  (len(button_down_icons) * (BUTTON_ICON_SIZE * BUTTON_ICON_SIZE) // 8) + \
                  (len(monitor_icons) * (MONITOR_ICON_SIZE * MONITOR_ICON_SIZE) // 8)
    print(f"Total size: {total_bytes} bytes")
    print("=" * 60)

if __name__ == '__main__':
    main()

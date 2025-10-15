#!/usr/bin/env python3
"""
Icon Converter for ESP32 Display
Converts image files (PNG, SVG, etc.) to C byte arrays for LovyanGFX

Usage:
    python icon_converter.py input_icon.png --name icon_manual --size 32

Output:
    Generates C array code to paste into assets/icons.hpp
"""

import argparse
import sys
from PIL import Image
import numpy as np

def image_to_bitmap_array(image_path, target_size=32, threshold=128):
    """
    Convert image to monochrome bitmap array

    Args:
        image_path: Path to input image
        target_size: Target width/height (assumes square)
        threshold: Brightness threshold for black/white (0-255)

    Returns:
        tuple: (width, height, byte_array)
    """
    try:
        # Load and convert to grayscale
        img = Image.open(image_path).convert('L')

        # Resize to target dimensions
        img = img.resize((target_size, target_size), Image.Resampling.LANCZOS)

        # Convert to numpy array
        img_array = np.array(img)

        # Threshold to binary (0 or 1)
        binary_array = (img_array > threshold).astype(np.uint8)

        # Pack bits into bytes (8 pixels per byte, row-major)
        width, height = target_size, target_size
        bytes_per_row = (width + 7) // 8
        byte_array = []

        for y in range(height):
            for x_byte in range(bytes_per_row):
                byte_val = 0
                for bit in range(8):
                    x = x_byte * 8 + bit
                    if x < width:
                        if binary_array[y, x]:
                            byte_val |= (1 << (7 - bit))
                byte_array.append(byte_val)

        return width, height, byte_array

    except FileNotFoundError:
        print(f"Error: File '{image_path}' not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error processing image: {e}")
        sys.exit(1)

def generate_c_code(name, width, height, byte_array):
    """Generate C code for the icon"""

    code = f"// {name} icon ({width}x{height})\n"
    code += f"constexpr int {name}_width = {width};\n"
    code += f"constexpr int {name}_height = {height};\n"
    code += f"constexpr uint8_t {name}_data[] = {{\n"

    # Format bytes in rows of 8
    for i in range(0, len(byte_array), 8):
        row = byte_array[i:i+8]
        code += "    " + ", ".join(f"0x{b:02X}" for b in row)
        if i + 8 < len(byte_array):
            code += ","
        code += "\n"

    code += "};\n"

    return code

def main():
    parser = argparse.ArgumentParser(
        description='Convert icons to C arrays for ESP32 display'
    )
    parser.add_argument('input', help='Input image file (PNG, SVG, etc.)')
    parser.add_argument('--name', required=True, help='Icon name (e.g., icon_manual)')
    parser.add_argument('--size', type=int, default=32, help='Icon size in pixels (default: 32)')
    parser.add_argument('--threshold', type=int, default=128,
                       help='Brightness threshold 0-255 (default: 128)')
    parser.add_argument('--output', help='Output file (default: stdout)')
    parser.add_argument('--invert', action='store_true',
                       help='Invert colors (black to white, white to black)')

    args = parser.parse_args()

    # Convert image
    width, height, byte_array = image_to_bitmap_array(
        args.input,
        args.size,
        args.threshold
    )

    # Invert if requested
    if args.invert:
        byte_array = [~b & 0xFF for b in byte_array]

    # Generate C code
    c_code = generate_c_code(args.name, width, height, byte_array)

    # Output
    if args.output:
        with open(args.output, 'w') as f:
            f.write(c_code)
        print(f"Generated icon code written to {args.output}")
    else:
        print(c_code)

    print(f"\nIcon: {args.name}", file=sys.stderr)
    print(f"Size: {width}x{height}", file=sys.stderr)
    print(f"Bytes: {len(byte_array)}", file=sys.stderr)

if __name__ == '__main__':
    main()

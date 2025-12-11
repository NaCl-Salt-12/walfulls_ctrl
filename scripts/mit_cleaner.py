#!/usr/bin/env python3
"""
MIT Dataset Cleaner
-------------------
Cleans and reformats MIT dataset CSV files by selecting specific columns
and renaming them for better readability.
"""

import polars as pl
import argparse
import sys
from pathlib import Path


def fix_mit(file_path: str, output_path: str | None = None) -> None:
    """
    Fix the MIT dataset by selecting and renaming specific columns.

    Args:
        file_path (str): Path to the MIT dataset CSV file.
        output_path (str, optional): Path to save the cleaned file.
                                     If None, overwrites the input file.
    """
    # Required columns for the dataset
    REQUIRED_COLUMNS = ["time", "data[0]", "data[1]", "data[2]", "data[3]", "data[4]"]

    try:
        # Validate input file exists
        if not Path(file_path).exists():
            raise FileNotFoundError(f"Input file not found: {file_path}")

        print(f"Reading dataset from: {file_path}")

        # Read the CSV to check columns first
        df = pl.read_csv(file_path, n_rows=1)
        available_columns = df.columns

        # Validate that all required columns exist
        missing_columns = [
            col for col in REQUIRED_COLUMNS if col not in available_columns
        ]

        if missing_columns:
            raise ValueError(
                f"Missing required columns: {', '.join(missing_columns)}\n"
            )

        print("✓ All required columns found")

        # Read and process the dataset
        q = (
            pl.scan_csv(file_path)
            .select("time", "data[0]", "data[1]", "data[2]", "data[3]", "data[4]")
            .rename(
                {
                    "data[0]": "position",
                    "data[1]": "velocity",
                    "data[2]": "kp",
                    "data[3]": "kd",
                    "data[4]": "torque",
                }
            )
            .collect()
        )

        # Determine output path
        save_path = output_path if output_path else file_path

        # Save cleaned file
        q.write_csv(save_path)
        print(f"✓ Successfully cleaned dataset saved to: {save_path}")

    except Exception as e:
        print(f"✗ Error processing file: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description="Clean and reformat MIT dataset CSV files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s data.csv                    # Clean data.csv (overwrites)
  %(prog)s data.csv -o cleaned.csv     # Save to cleaned.csv
        """,
    )

    parser.add_argument("input_file", help="Path to the input MIT dataset CSV file")

    parser.add_argument(
        "-o",
        "--output",
        dest="output_file",
        help="Path to save the cleaned file (default: overwrites input file)",
    )

    args = parser.parse_args()

    # Process the file
    fix_mit(args.input_file, args.output_file)


if __name__ == "__main__":
    main()

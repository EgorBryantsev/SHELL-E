#!/bin/bash

# Script to delete all .meta files in a specified directory and its subdirectories.
#
# WARNING: THIS IS A DESTRUCTIVE OPERATION.
# Deleting .meta files can break Unity projects or remove essential import
# settings if you are dealing with Unity assets.
#
# USE WITH EXTREME CAUTION. ALWAYS BACK UP YOUR DATA FIRST.
# TEST ON A COPY OF YOUR DIRECTORY BEFORE RUNNING ON VALUABLE DATA.

if [ -z "$1" ]; then
  echo "Usage: $0 <directory_path>"
  echo "Example: $0 ./MyFolderToClean"
  exit 1
fi

TARGET_DIR="$1"

if [ ! -d "$TARGET_DIR" ]; then
  echo "Error: Directory '$TARGET_DIR' not found."
  exit 1
fi

echo "You are about to recursively delete all .meta files in and under the directory:"
echo "$TARGET_DIR"
echo ""
echo "This action CANNOT be undone easily and might be HARMFUL if this is a Unity project folder."
echo ""
read -p "Are you ABSOLUTELY SURE you want to proceed? (Type 'yes' to confirm): " CONFIRMATION

if [ "$CONFIRMATION" == "yes" ]; then
  echo "Searching for and deleting .meta files in '$TARGET_DIR'..."
  # Use find to locate and delete .meta files
  # -type f: only files
  # -name "*.meta": files ending with .meta
  # -print: (optional) print the files being deleted
  # -delete: perform the deletion
  find "$TARGET_DIR" -type f -name "*.meta" -print -delete
  echo "Deletion of .meta files complete."
else
  echo "Operation aborted by the user."
fi

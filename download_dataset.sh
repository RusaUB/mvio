#!/bin/bash


DRIVE_ZIP_URL="https://drive.google.com/file/d/1yQyBhH8L-3Qb9hFjK4_hMqGgJIENU9zu/view?usp=sharing"

echo "------------------------------------------------"
echo "MVIO Dataset Downloader"
echo "------------------------------------------------"

# Check if dataset already exists
if [ -d "dataset" ]; then
    echo "Warning: The 'dataset' directory already exists."
    read -p "Do you want to overwrite it? (y/N): " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "Download cancelled."
        exit 0
    fi
fi

# Create a temporary virtual environment to bypass system package limits
echo "Creating temporary python environment for downloader..."
if [ -d ".temp_dl_venv" ]; then
    rm -rf .temp_dl_venv
fi

python3 -m venv .temp_dl_venv
if [ ! -d ".temp_dl_venv" ]; then
    echo "Error: Failed to create virtual environment."
    echo "Please ensure 'python3-venv' is installed (sudo apt install python3-venv)."
    exit 1
fi

# Activate venv and install gdown
source .temp_dl_venv/bin/activate

echo "Installing gdown in temporary environment..."
pip install --quiet gdown

# Download the Zip file
echo "Downloading dataset.zip from Google Drive..."

# Extract File ID from URL (more reliable)
# Matches content between /d/ and /view
FILE_ID=$(echo "$DRIVE_ZIP_URL" | sed -n 's/.*\/d\/\([^/]*\).*/\1/p')

if [ -z "$FILE_ID" ]; then
    echo "Error: Could not extract File ID from URL."
    echo "Using full URL with fuzzy matching..."
    gdown "$DRIVE_ZIP_URL" -O dataset.zip --fuzzy
else
    echo "Detected File ID: $FILE_ID"
    # Note: Modern gdown doesn't need --id, just the ID string
    gdown "$FILE_ID" -O dataset.zip
fi

# Validate Download
if [ ! -f "dataset.zip" ]; then
    echo "Error: dataset.zip was not downloaded."
    deactivate
    rm -rf .temp_dl_venv
    exit 1
fi

# Check if it's actually a zip file
FILE_TYPE=$(file -b --mime-type dataset.zip)
echo "Downloaded file type: $FILE_TYPE"

if [[ "$FILE_TYPE" != "application/zip" && "$FILE_TYPE" != "application/x-zip-compressed" ]]; then
    echo "Error: The downloaded file is not a zip archive."
    echo "It is likely an HTML page (Google Drive access denied or virus warning)."
    echo "Content preview:"
    head -n 5 dataset.zip
    
    echo ""
    echo "Please check your link permissions."
    deactivate
    rm -rf .temp_dl_venv
    # Keep the file for inspection
    exit 1
fi

# Unzip
echo "Extracting dataset..."
if unzip -q dataset.zip; then
    # Clean up zip only on success
    rm dataset.zip
    echo "Dataset extracted successfully."
else
    echo "Error: Unzip failed."
    echo "The file 'dataset.zip' has been kept for inspection."
    deactivate
    rm -rf .temp_dl_venv
    exit 1
fi

# Cleanup Virtual Environment
deactivate
rm -rf .temp_dl_venv

echo "------------------------------------------------"
echo "Done."
echo "------------------------------------------------"

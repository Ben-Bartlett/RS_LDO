#!/bin/bash


echo "Download the data manually from"
echo
echo "https://ulcampus-my.sharepoint.com/:f:/g/personal/17218187_studentmail_ul_ie/EtIYMD9JirNEpHhStkT-lnkBOESLD-65iAI_5KaVxzuHJg"
echo
echo "cancel the script now if you have not downloaded the bags."


# Function to check if Data.zip exists in specified directory and move it to current directory if found
move_data_zip() {
    local directory="$1"
    if [ -f "$directory/Data.zip" ]; then
        echo "Found Data.zip in $directory. Moving it to the current directory..."
        mv "$directory/Data.zip" .
        return 0
    else
        return 1
    fi
}

echo
echo

# Check if Data.zip exists in the current directory
if move_data_zip "."; then
    echo "Data.zip found in the current directory."
elif move_data_zip "$HOME/Downloads"; then
    echo "Data.zip found in ~/Downloads. Moved it to the current directory."
else
    echo "Data.zip not found in the current directory or ~/Downloads."
    echo "Please download Data.zip manually from the provided link and place it in either folder."
    exit 1
fi


# Adding blank lines for readability
echo
echo

echo "Downloading GIT Repo"
git clone https://github.com/pmqs/Fix-OneDrive-Zip.git
echo "Downloaded GIT repo"

echo

echo "Fixing ZIP"
perl Fix-OneDrive-Zip/fix-onedrive-zip Data.zip 
echo "Fixed ZIP"

echo

echo "Unzipping Folder"
unzip Data.zip
echo "Unzipped Folder"

echo

echo "Removing unnecessary files"
rm Data.zip
rm -rf Fix-OneDrive-Zip
echo "Removed unnecessary files"


#!/bin/bash

echo "Downloading data folder from Ben's OneDrive if the link is still valid."
echo "https://ulcampus-my.sharepoint.com/:f:/g/personal/17218187_studentmail_ul_ie/EtIYMD9JirNEpHhStkT-lnkBOESLD-65iAI_5KaVxzuHJg"

echo "Else email me, ben.bartlett@ul.ie."
echo "This folder currently has about 15GB of bags."
echo "Given this, if you download the folder manually, you need to run the fix also."
echo "Look at README.md Installation for more info."


#echo "Downloaded folder successfully."

echo "Downloading GIT Repo"
git clone https://github.com/pmqs/Fix-OneDrive-Zip.git
echo "Downloaded GIT repo"

#echo "Fixing ZIP"
#perl Fix-OneDrive-Zip/fix-onedrive-zip Data.zip 
#echo "Fixed ZIP"

#echo "Unzipping Folder"
#unzip Data.zip
#echo "Unzipped Folder"

#echo "Removing unnecessary files"
#rm Data.zip
#rm -rf Fix-OneDrive-Zip
#echo "Removed unnecessary files"


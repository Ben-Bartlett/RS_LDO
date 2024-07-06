#!/bin/bash

echo "Downloading data folder from Ben's OneDrive if the link is still valid."
echo "https://ulcampus-my.sharepoint.com/:f:/g/personal/17218187_studentmail_ul_ie/EtIYMD9JirNEpHhStkT-lnkBOESLD-65iAI_5KaVxzuHJg"
echo "Else email me, ben.bartlett@ul.ie."
echo "This folder currently has about 15GB of bags."
echo "Given this, if you download the folder manually, you need to run the fix also."
echo "Look at README.md Installation for more info."

# Download the zip file in the background and assign it job number 1
curl --header 'Host: northeurope1-mediap.svc.ms' --user-agent 'Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/113.0' --header 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,*/*;q=0.8' --header 'Accept-Language: en-US,en;q=0.5' --header 'Content-Type: application/x-www-form-urlencoded' --header 'Origin: https://ulcampus-my.sharepoint.com' --header 'Upgrade-Insecure-Requests: 1' --header 'Sec-Fetch-Dest: iframe' --header 'Sec-Fetch-Mode: navigate' --header 'Sec-Fetch-Site: cross-site' --request POST --data-urlencode 'zipFileName=Data.zip' --data-urlencode 'guid=78327c7e-797a-4e15-b809-267e0adc1629' --data-urlencode 'provider=spo' --data-urlencode 'files={"items":[{"name":"Data","size":0,"docId":"https://ulcampus-my.sharepoint.com:443/_api/v2.0/drives/b!7EBgFLJhfkqQMlvmC0Pekhg3anbWNclJvxleWEFgOTk1VjiWmEZxS74cD7NDhKX2/items/01PPBRUX6SDAYD6SMKWNCKI6CSWZCP5FTZ?version=Published&access_token=v1.eyJzaXRlaWQiOiIxNDYwNDBlYy02MWIyLTRhN2UtOTAzMi01YmU2MGI0M2RlOTIiLCJhdWQiOiIwMDAwMDAwMy0wMDAwLTBmZjEtY2UwMC0wMDAwMDAwMDAwMDAvdWxjYW1wdXMtbXkuc2hhcmVwb2ludC5jb21AMDA4NGI5MjQtM2FiNC00MTE2LTkyNTEtOTkzOWY2OTVlNTRjIiwiZXhwIjoiMTcyMDIzNDgwMCJ9.CiMKCXNoYXJpbmdpZBIWNHFQOVhSblNSRVcvMno5T2xmUzVDQQoICgNzdHASAXQKCgoEc25pZBICMzMSBgiaqjoQARonMmEwMTpjYjFkOjhjMTU6ZmUwMDoxZDFkOmNlMmU6M2M3MjoxNDQxIhRtaWNyb3NvZnQuc2hhcmVwb2ludCosckpGU3Juc0Q5QzlMb2Y0ZXNENlJWZXd2cFdCRUJ0YmZ6ZVdPK3lMRmRPYz0wdjgBShBoYXNoZWRwcm9vZnRva2VuYgR0cnVlcmEwaC5mfG1lbWJlcnNoaXB8dXJuJTNhc3BvJTNhYW5vbiNhNjg2ODMyZjVmMTBiYzU2OTlhZTJkZmY4YThjNzczZjIzODQ1MWNjNGY1N2ZlYjlhMjA1MzU0OTY5OWVjYTM2egEwwgFhMCMuZnxtZW1iZXJzaGlwfHVybiUzYXNwbyUzYWFub24jYTY4NjgzMmY1ZjEwYmM1Njk5YWUyZGZmOGE4Yzc3M2YyMzg0NTFjYzRmNTdmZWI5YTIwNTM1NDk2OTllY2EzNg.6jP79YyDfEAQc2J7gTAV-xEr7tzVfIlwRSV8OpQtdO0","isFolder":true}]}' --data-urlencode 'oAuthToken=' 'https://northeurope1-mediap.svc.ms/transform/zip?cs=fFNQTw' --output 'Data.zip' &
pid=$!

#wait for download
wait $pid 

echo "Downloaded folder successfully."

echo "Downloading GIT Repo"
git clone https://github.com/pmqs/Fix-OneDrive-Zip.git
echo "Downloaded GIT repo"

echo "Fixing ZIP"
perl Fix-OneDrive-Zip/fix-onedrive-zip Data.zip 
echo "Fixed ZIP"

echo "Unzipping Folder"
unzip Data.zip
echo "Unzipped Folder"

echo "Removing unnecessary files"
rm Data.zip
rm -rf Fix-OneDrive-Zip
echo "Removed unnecessary files"


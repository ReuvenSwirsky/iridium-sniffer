rm -rf r
./build/iridium-sniffer -i usrp-B200-321E16D  --usrp-gain=65     --clock-source=external --time-source=external     --save-bursts ./r --start-next-minute -t 10 

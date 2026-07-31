rm -rf d
rm -rf r

./build/iridium-sniffer -i usrp-B200-3229C91 --usrp-gain=65     --clock-source=external --time-source=external     --save-bursts ./d --start-next-minute -t 10 



echo -e "\n"
ps -axj | grep -E 'vcsmd|VCF'
echo -e "\n"
killall -9 VCFserver_pc.exe VCFserver_rpi.exe
echo -e "\n"
ps -axj | grep -E 'vcsmd|VCF'
echo -e "\n"

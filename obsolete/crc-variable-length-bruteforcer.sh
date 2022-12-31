#!/bin/bash
#./reveng -s -w 16 123468979E77
for xcut in $(seq 0 15); do 
for length in $(seq 0 15); do
cat values.txt | colrm 18 19 | while read line; do
  echo "${line:xcut:length}${line:15}"
  #CRC=${line:15}
  #echo -n "${line:xcut:length}"
  #printf '%x\n' "$(( (~ 0x$CRC) & 0xffff ))"
done > temp.txt
DATA=$(cat temp.txt | tr '\n' ' ')
#echo "gugu = ${DATA}"
CMD="./reveng -s -w 16 $DATA"
$CMD &>/dev/null
if [ $? == 0 ]; then
echo
echo "== Found match for ${DATA} =="
$CMD
echo
fi
done
done

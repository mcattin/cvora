#! /bin/sh

S_BIT='../syn/top_cvora.bit'
S_XSVF='../prom/cvora.xsvf'

D_BIT='cvora_v'$1'.bit'
D_XSVF='cvora_v'$1'.xsvf'

D_SCP='cs-ccr-dev3:/acc/local/share/firmware/cvora/'

if echo $1 | egrep -q '^[1-9]?[0-9]_[0-9]{2}$'
then
    echo 'Releasing version '$1
    cp -iv $S_BIT $D_BIT
    cp -iv $S_XSVF $D_XSVF
    scp $D_XSVF $D_SCP
    ls -l
else
    echo 'Wrong version format!'
fi
#! /bin/sh

wget https://edms.cern.ch/file/782366/1/EDA-01444-V1_fp.pdf

# extract first page
pdftk A=EDA-01444-V1_fp.pdf cat A1 output cvora_fp.pdf

# crop around front panel
#   bbox origin is in bottom left corner of the page
#   bbox unit is [points]
pdfcrop --bbox '400 290 500 970' cvora_fp.pdf cvora_fp.pdf